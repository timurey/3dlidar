#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2, Imu, JointState
from std_msgs.msg import String, Float64
from std_srvs.srv import Trigger
from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
import json
import serial
import threading
import time
import subprocess
import os
import shutil
from datetime import datetime


# Topics recorded per mode — must match (topic, ROS msg type string).
# Used by the in-process SequentialWriter to register channels at bag-open time.
RECORD_TOPICS = {
    'static': [
        ('/velodyne_points',              'sensor_msgs/msg/PointCloud2'),
        ('/rotating_platform/angle',      'std_msgs/msg/Float64'),
        ('/rotating_platform/velocity',   'std_msgs/msg/Float64'),
        ('/rotating_platform/joint_state', 'sensor_msgs/msg/JointState'),  # stamped angle for deskew
    ],
    'slam': [
        ('/velodyne_points',              'sensor_msgs/msg/PointCloud2'),
        ('/rotating_platform/angle',      'std_msgs/msg/Float64'),
        ('/rotating_platform/velocity',   'std_msgs/msg/Float64'),
        ('/rotating_platform/joint_state', 'sensor_msgs/msg/JointState'),  # stamped angle for deskew
        ('/imu',                          'sensor_msgs/msg/Imu'),
    ],
}


class HMIBridge(Node):
    def __init__(self):
        super().__init__('hmi_bridge')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('use_serial', False)
        self.declare_parameter('bag_directory', '/home/openclaw/bags')

        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.use_serial = self.get_parameter('use_serial').value
        self.bag_directory = self.get_parameter('bag_directory').value

        # Create bag directory if it doesn't exist
        os.makedirs(self.bag_directory, exist_ok=True)

        # Initialize serial connection
        self.serial_conn = None
        if self.use_serial:
            try:
                self.serial_conn = serial.Serial(
                    self.serial_port,
                    self.baud_rate,
                    timeout=0.1
                )
                self.get_logger().info(f'Serial connected on {self.serial_port} at {self.baud_rate} baud')
            except Exception as e:
                self.get_logger().warn(f'Failed to open serial port: {e}')
                self.get_logger().info('Running without serial connection')

        # Topic monitoring
        self.lidar_raw_hz = 0.0
        # self.lidar_corrected_hz = 0.0
        self.imu_hz = 0.0
        self.encoder_hz = 0.0
        self.lidar_raw_last_time = None
        # self.lidar_corrected_last_time = None
        self.imu_last_time = None
        self.encoder_last_time = None
        self.lidar_raw_msg_count = 0
        # self.lidar_corrected_msg_count = 0
        self.imu_msg_count = 0
        self.encoder_msg_count = 0
        self.encoder_angle = 0.0  # Current encoder angle (0-360)

        # System state
        self.sensors_running = False
        self.sensors_process = None

        # Lidar watchdog: the Hesai driver can wedge at 0 Hz while its process stays alive
        # (so launch respawn can't help). If /velodyne_points stays silent while sensors
        # are running, auto-restart the sensor stack. Tick-based (calculate_hz @1Hz) so it
        # is immune to NTP clock jumps. This is a safety net behind the DEVNULL fix.
        self.lidar_watchdog_enabled = True
        self.lidar_hz_min = 0.5          # below this Hz = considered dead
        self.lidar_zero_ticks = 0        # consecutive ~1s ticks with no lidar
        self.lidar_zero_limit = 8        # restart after this many silent ticks
        self.sensors_grace_ticks = 0     # warmup ticks before the watchdog arms
        self.lidar_watchdog_restarts = 0

        # Recording state
        self.is_recording = False
        self.recording_start_time = None
        self.current_bag_name = None
        self.current_mode = None
        # In-process rosbag2 SequentialWriter — replaces the spawned `ros2 bag record`
        # subprocess so there is exactly ONE persistent subscriber per topic on the
        # DDS bus (this node), rather than a churning subscribe/unsubscribe cycle.
        # See diagnosis docs around the empty-PointCloud2 burst.
        self._writer = None
        self._writer_lock = threading.Lock()

        # Subscribers
        self.lidar_raw_sub = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.lidar_raw_callback,
            10
        )

        # self.lidar_corrected_sub = self.create_subscription(
        # PointCloud2,
            # '/velodyne_points_corrected',
        # self.lidar_corrected_callback,
        # qos_profile_sensor_data  # BEST_EFFORT to match reconstructor publisher
        # )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        self.encoder_sub = self.create_subscription(
            Float64,
            '/rotating_platform/angle',
            self.encoder_callback,
            10
        )

        # Velocity subscription — needed for static-mode bag recording (Float64 RPM)
        self.velocity_sub = self.create_subscription(
            Float64,
            '/rotating_platform/velocity',
            self.velocity_callback,
            10
        )

        # Stamped encoder state — carries header.stamp (Pi-synced per-sample time) for
        # offline per-point deskew. Recorded alongside the bare Float64 /angle topic.
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/rotating_platform/joint_state',
            self.joint_state_callback,
            10
        )

        # Spin controller service clients
        self._spin_start = self.create_client(Trigger, '/spin_controller/start')
        self._spin_stop  = self.create_client(Trigger, '/spin_controller/stop')

        # Command interface for the Flask HMI (app.py).
        # Eliminates the prior pattern of spawning `ros2 bag record` as a subprocess
        # — instead, app.py publishes JSON commands here and we drive the in-process
        # SequentialWriter, with results acknowledged on /hmi_bridge/response.
        self.cmd_sub = self.create_subscription(
            String,
            '/hmi_bridge/cmd',
            self._on_cmd,
            10
        )
        self.response_pub = self.create_publisher(
            String,
            '/hmi_bridge/response',
            10
        )
        # Periodic full-status JSON for the web dashboard (app.py). hmi_bridge is
        # the single efficient subscriber to /velodyne_points etc.; republishing
        # the already-computed status here lets the dashboard show lidar Hz
        # without app.py opening its own heavy PointCloud2 subscriber.
        self.status_pub = self.create_publisher(
            String,
            '/hmi_bridge/status',
            10
        )

        # Timers
        self.status_timer = self.create_timer(0.5, self.send_status_update)
        self.hz_calc_timer = self.create_timer(1.0, self.calculate_hz)

        # Serial reading thread
        if self.serial_conn:
            self.serial_thread = threading.Thread(target=self.read_serial_commands, daemon=True)
            self.serial_thread.start()

        self.get_logger().info('HMI Bridge node started')

        # Auto-start sensors on boot (2 s delay to let ROS graph settle)
        self.create_timer(2.0, self.auto_start_sensors_once)
        self.get_logger().info('HMI Bridge node started — sensors will auto-start in 2 s')

    def auto_start_sensors_once(self):
        """Auto-start sensors on boot (runs once after 2 second delay)"""
        self.destroy_timer(self.get_clock().now())
        if not self.sensors_running:
            self.get_logger().info('Auto-starting sensor stack...')
            self.start_sensors()

    def _write_to_bag(self, topic, msg, ts_ns):
        """Write a serialized message to the active bag, if one is open. Thread-safe.
        Uses the receive-time (system clock) as the log_time, matching how the
        previous external `ros2 bag record` subprocess timestamped messages."""
        with self._writer_lock:
            w = self._writer
            if w is None:
                return
            try:
                w.write(topic, serialize_message(msg), ts_ns)
            except Exception as e:
                self.get_logger().error(
                    f'In-process bag write failed on {topic}: {e}',
                    throttle_duration_sec=2.0
                )

    def lidar_raw_callback(self, msg):
        self.lidar_raw_msg_count += 1
        current_time = self.get_clock().now()
        if self.lidar_raw_last_time is None:
            self.lidar_raw_last_time = current_time
        self._write_to_bag('/velodyne_points', msg, current_time.nanoseconds)

    # def lidar_corrected_callback(self, msg):
        # self.lidar_corrected_msg_count += 1
        # current_time = self.get_clock().now()
        # if self.lidar_corrected_last_time is None:
        # self.lidar_corrected_last_time = current_time

    def imu_callback(self, msg):
        self.imu_msg_count += 1
        current_time = self.get_clock().now()
        if self.imu_last_time is None:
            self.imu_last_time = current_time
        self._write_to_bag('/imu', msg, current_time.nanoseconds)

    def encoder_callback(self, msg):
        self.encoder_msg_count += 1
        self.encoder_angle = msg.data  # Capture current angle (0-360)
        current_time = self.get_clock().now()
        if self.encoder_last_time is None:
            self.encoder_last_time = current_time
        self._write_to_bag('/rotating_platform/angle', msg, current_time.nanoseconds)

    def velocity_callback(self, msg):
        """Pure pass-through — velocity is only used at recording time, not for HMI Hz."""
        current_time = self.get_clock().now()
        self._write_to_bag('/rotating_platform/velocity', msg, current_time.nanoseconds)

    def joint_state_callback(self, msg):
        """Pure pass-through — stamped encoder state, only used at recording time.
        The message's own header.stamp (not this log_time) is what the deskewer uses."""
        current_time = self.get_clock().now()
        self._write_to_bag('/rotating_platform/joint_state', msg, current_time.nanoseconds)

    def calculate_hz(self):
        """Calculate message rates for topics"""
        current_time = self.get_clock().now()

        # Calculate raw lidar Hz
        if self.lidar_raw_last_time is not None:
            duration = (current_time - self.lidar_raw_last_time).nanoseconds / 1e9
            if duration > 0:
                self.lidar_raw_hz = self.lidar_raw_msg_count / duration

        # Calculate corrected lidar Hz
        # if self.lidar_corrected_last_time is not None:
        # duration = (current_time - self.lidar_corrected_last_time).nanoseconds / 1e9
        # if duration > 0:
        # self.lidar_corrected_hz = self.lidar_corrected_msg_count / duration

        # Calculate IMU Hz
        if self.imu_last_time is not None:
            duration = (current_time - self.imu_last_time).nanoseconds / 1e9
            if duration > 0:
                self.imu_hz = self.imu_msg_count / duration

        # Calculate Encoder Hz
        if self.encoder_last_time is not None:
            duration = (current_time - self.encoder_last_time).nanoseconds / 1e9
            if duration > 0:
                self.encoder_hz = self.encoder_msg_count / duration

        # Reset counters
        self.lidar_raw_msg_count = 0
        # self.lidar_corrected_msg_count = 0
        self.imu_msg_count = 0
        self.encoder_msg_count = 0
        self.lidar_raw_last_time = current_time
        # self.lidar_corrected_last_time = current_time
        self.imu_last_time = current_time
        self.encoder_last_time = current_time

        self._lidar_watchdog_tick()

    def _lidar_watchdog_tick(self):
        """Auto-recover a wedged Hesai driver (0 Hz while the process is still alive)."""
        if not (self.sensors_running and self.lidar_watchdog_enabled):
            return
        if self.sensors_grace_ticks > 0:
            self.sensors_grace_ticks -= 1
            return
        if self.lidar_raw_hz < self.lidar_hz_min:
            self.lidar_zero_ticks += 1
            if self.lidar_zero_ticks >= self.lidar_zero_limit:
                self.lidar_watchdog_restarts += 1
                self.get_logger().error(
                    f'Lidar watchdog: /velodyne_points silent for {self.lidar_zero_ticks}s '
                    f'— restarting sensors (auto-restart #{self.lidar_watchdog_restarts}).')
                if self.is_recording:
                    self.get_logger().error(
                        'Watchdog restart aborts the active recording — re-record after recovery.')
                self.lidar_zero_ticks = 0
                self.stop_sensors()
                self.start_sensors()   # re-arms sensors_grace_ticks on success
        else:
            self.lidar_zero_ticks = 0

    def get_disk_space(self):
        """Get available disk space in GB"""
        try:
            stat = shutil.disk_usage(self.bag_directory)
            return stat.free / (1024**3)  # Convert to GB
        except Exception as e:
            self.get_logger().error(f'Failed to get disk space: {e}')
            return 0.0

    def get_recording_duration(self):
        """Get current recording duration in seconds"""
        if self.is_recording and self.recording_start_time:
            duration = (datetime.now() - self.recording_start_time).total_seconds()
            return int(duration)
        return 0

    def send_status_update(self):
        """Send status update to ESP32"""
        status = {
            'sensors_running': self.sensors_running,
            'lidar_raw_hz': round(self.lidar_raw_hz, 1),
        # 'lidar_corrected_hz': round(self.lidar_corrected_hz, 1),
            'imu_hz': round(self.imu_hz, 1),
            'encoder_hz': round(self.encoder_hz, 1),
            'encoder_angle': round(self.encoder_angle, 1),
            'lidar_raw_ok': self.lidar_raw_hz > 5.0,
        # 'lidar_corrected_ok': self.lidar_corrected_hz > 5.0,
            'imu_ok': self.imu_hz > 8.0,   # ybimu_driver publishes at 10 Hz (timer-driven)
            'encoder_ok': self.encoder_hz > 80.0,   # spin_controller publishes at ~100 Hz
            'recording': self.is_recording,
            'disk_gb': round(self.get_disk_space(), 1),
            'rec_duration': self.get_recording_duration(),
            'bag_name': self.current_bag_name or ''
        }

        # Publish full status JSON for the web dashboard (app.py subscribes).
        try:
            status_msg = String()
            status_msg.data = json.dumps(status)
            self.status_pub.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f'Status publish error: {e}')

        # Send via serial if connected
        if self.serial_conn and self.serial_conn.is_open:
            try:
                json_str = json.dumps(status) + '\n'
                self.serial_conn.write(json_str.encode())
                self.serial_conn.flush()  # Force send immediately
                # Log every 10 seconds to confirm sending
                if not hasattr(self, '_last_debug_log'):
                    self._last_debug_log = 0
                if (self.get_clock().now().nanoseconds / 1e9 - self._last_debug_log) > 10:
                    self.get_logger().info(f'Sent to CYD: {json_str[:50]}...')
                    self._last_debug_log = self.get_clock().now().nanoseconds / 1e9
            except Exception as e:
                self.get_logger().error(f'Serial write error: {e}')

        # Also log to console for debugging
        if not self.use_serial:
            self.get_logger().info(f'Status: {status}', throttle_duration_sec=5.0)

    def start_sensors(self):
        """Start all sensors via slam_scanner.launch.py (VLP-16 + IMU + spin_controller)."""
        if self.sensors_running:
            self.get_logger().warn('Sensors already running')
            return False

        from ament_index_python.packages import get_package_share_directory
        launch_file = os.path.join(
            get_package_share_directory('slam_bringup'),
            'launch',
            'slam_scanner.launch.py'
        )

        cmd = ['ros2', 'launch', launch_file, 'with_velodyne:=true']

        try:
            # stdout/stderr → DEVNULL: the VLP-16 driver prints at high frequency,
            # filling the 64KB OS pipe buffer and blocking its publish thread.
            self.sensors_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self.sensors_running = True
            self.sensors_grace_ticks = 25   # arm lidar watchdog only after driver warmup
            self.lidar_zero_ticks = 0
            self.get_logger().info('Started sensors (VLP-16 + IMU + spin_controller)')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to start sensors: {e}')
            return False

    def stop_sensors(self):
        """Stop all sensors"""
        if not self.sensors_running:
            self.get_logger().warn('Sensors not running')
            return False

        # Stop recording first if active
        if self.is_recording:
            self.stop_recording()

        try:
            if self.sensors_process:
                self.sensors_process.terminate()
                self.sensors_process.wait(timeout=10)
                self.get_logger().info('Stopped sensors')

            self.sensors_running = False
            self.sensors_process = None

            # Reset Hz counters
            self.lidar_raw_hz = 0.0
        # self.lidar_corrected_hz = 0.0
            self.imu_hz = 0.0
            self.encoder_hz = 0.0
            self.encoder_angle = 0.0
            self.lidar_raw_msg_count = 0
        # self.lidar_corrected_msg_count = 0
            self.imu_msg_count = 0
            self.encoder_msg_count = 0

            return True
        except Exception as e:
            self.get_logger().error(f'Failed to stop sensors: {e}')
            return False

    def _call_spin(self, client, name: str, timeout: float = 8.0) -> bool:
        """Call a spin_controller Trigger service from a background thread."""
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'spin_controller/{name} not available')
            return False
        future = client.call_async(Trigger.Request())
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        if not future.done():
            self.get_logger().error(f'spin_controller/{name} timed out')
            return False
        result = future.result()
        return result.success

    def start_recording(self, bag_name=None, mode='static'):
        """Start platform rotation, wait 2 s for speed, then start in-process rosbag2 recording.
        Replaces the previous spawn of `ros2 bag record` so that the bag's
        subscribers are this very node — no DDS subscribe/unsubscribe cycle
        per recording, which is what was triggering the empty-PointCloud2 burst.
        Returns (ok: bool, msg: str, name: str|None)."""
        with self._writer_lock:
            if self._writer is not None:
                return False, 'Already recording', self.current_bag_name

            if mode not in RECORD_TOPICS:
                mode = 'static'

            # Generate unique bag name if not provided
            if not bag_name:
                now = datetime.now()
                timestamp = now.strftime('%Y%m%d_%H%M%S')
                ms = now.microsecond // 1000
                bag_name = f'{mode}_{timestamp}_{ms:03d}'
                counter = 0
                while os.path.exists(os.path.join(self.bag_directory, bag_name)):
                    counter += 1
                    bag_name = f'{mode}_{timestamp}_{ms:03d}_{counter}'

            bag_path = os.path.join(self.bag_directory, bag_name)
            if os.path.exists(bag_path):
                return False, f'Bag {bag_name} already exists', None

            try:
                writer = SequentialWriter()
                storage_options = StorageOptions(uri=bag_path, storage_id='mcap')
                converter_options = ConverterOptions(
                    input_serialization_format='cdr',
                    output_serialization_format='cdr',
                )
                writer.open(storage_options, converter_options)
                # rosbag2_py.TopicMetadata in jazzy requires explicit positive `id`
                for idx, (topic, type_str) in enumerate(RECORD_TOPICS[mode], start=1):
                    writer.create_topic(TopicMetadata(
                        id=idx,
                        name=topic,
                        type=type_str,
                        serialization_format='cdr',
                    ))
                self._writer = writer
                self.is_recording = True
                self.recording_start_time = datetime.now()
                self.current_bag_name = bag_name
                self.current_mode = mode
                self.get_logger().info(
                    f'In-process recording started: {bag_name} mode={mode}'
                )
                return True, 'ok', bag_name
            except Exception as e:
                self.get_logger().error(f'Failed to open in-process writer: {e}')
                # Best-effort cleanup of partial bag dir
                try:
                    if os.path.isdir(bag_path) and not os.listdir(bag_path):
                        os.rmdir(bag_path)
                except Exception:
                    pass
                return False, str(e), None

    def stop_recording(self):
        """Stop in-process rosbag2 recording. Returns (ok, msg, name)."""
        with self._writer_lock:
            if self._writer is None:
                return False, 'Not currently recording', None
            name = self.current_bag_name
            try:
                # Closing the writer flushes the final mcap chunks + metadata.yaml.
                # rosbag2_py's writer doesn't expose an explicit .close() — destruction
                # closes it. Drop the reference and let GC finalize, but also forcibly
                # delete to make finalization synchronous.
                del self._writer
                self._writer = None
                self.is_recording = False
                self.recording_start_time = None
                self.get_logger().info(f'In-process recording stopped: {name}')
            except Exception as e:
                self._writer = None
                self.is_recording = False
                self.recording_start_time = None
                self.get_logger().error(f'Error stopping in-process writer: {e}')
                # Still stop motor even on error
                self._call_spin(self._spin_stop, 'stop', timeout=3.0)
                return False, str(e), name

        # Stop platform rotation after bag is closed
        self.get_logger().info('Stopping platform rotation...')
        self._call_spin(self._spin_stop, 'stop', timeout=3.0)
        return True, 'ok', name

    def _on_cmd(self, msg):
        """JSON command interface from app.py (Flask HMI).
        Recording commands are dispatched to a background thread so that
        _call_spin (which blocks on a service future) does not deadlock the
        single-threaded ROS executor."""
        try:
            cmd = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f'Invalid /hmi_bridge/cmd JSON: {e}')
            return

        action = cmd.get('action') or cmd.get('cmd')
        if action == 'start_record':
            threading.Thread(
                target=self._start_record_thread,
                args=(cmd.get('name'), cmd.get('mode', 'static')),
                daemon=True,
            ).start()
        elif action == 'stop_record':
            threading.Thread(
                target=self._stop_record_thread,
                daemon=True,
            ).start()
        elif action == 'ping':
            self._publish_response('ping', True, 'pong', None)
        else:
            self.get_logger().warn(f'Unknown /hmi_bridge/cmd action: {action!r}')

    def _start_record_thread(self, bag_name, mode):
        """Background thread: start motor → wait → open bag → publish response."""
        self.get_logger().info('Starting platform rotation...')
        self._call_spin(self._spin_start, 'start', timeout=8.0)
        self.get_logger().info('Waiting 2 s for platform to reach speed...')
        time.sleep(2.0)
        ok, m, name = self.start_recording(bag_name=bag_name, mode=mode)
        self._publish_response('start_record', ok, m, name)

    def _stop_record_thread(self):
        """Background thread: close bag → stop motor → publish response."""
        ok, m, name = self.stop_recording()
        self._publish_response('stop_record', ok, m, name)

    def _publish_response(self, action, ok, msg_str, name):
        out = String()
        out.data = json.dumps({
            'action': action,
            'ok':     bool(ok),
            'msg':    msg_str,
            'name':   name,
        })
        self.response_pub.publish(out)

    def read_serial_commands(self):
        """Read commands from serial port (runs in separate thread)"""
        while rclpy.ok():
            try:
                if self.serial_conn and self.serial_conn.is_open:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    # Only process lines that look like JSON (start with '{')
                    if line and line.startswith('{'):
                        self.process_command(line)
            except Exception as e:
                # Only log errors occasionally to avoid spam
                pass

    def process_command(self, cmd_str):
        """Process incoming command from ESP32"""
        try:
            cmd = json.loads(cmd_str)
            self.get_logger().info(f'Received command: {cmd}')

            if cmd.get('cmd') == 'start_sensors':
                success = self.start_sensors()
                self.send_response({'status': 'ok' if success else 'error'})

            elif cmd.get('cmd') == 'stop_sensors':
                success = self.stop_sensors()
                self.send_response({'status': 'ok' if success else 'error'})

            elif cmd.get('cmd') == 'start_recording':
                bag_name = cmd.get('name', None)
                mode = cmd.get('mode', 'static')
                ok, _msg, _name = self.start_recording(bag_name=bag_name, mode=mode)
                self.send_response({'status': 'ok' if ok else 'error'})

            elif cmd.get('cmd') == 'stop_recording':
                ok, _msg, _name = self.stop_recording()
                self.send_response({'status': 'ok' if ok else 'error'})

            elif cmd.get('cmd') == 'ping':
                self.send_response({'status': 'pong'})

            else:
                self.get_logger().warn(f'Unknown command: {cmd}')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON command: {e}')

    def send_response(self, response):
        """Send response back to ESP32"""
        if self.serial_conn and self.serial_conn.is_open:
            try:
                json_str = json.dumps(response) + '\n'
                self.serial_conn.write(json_str.encode())
            except Exception as e:
                self.get_logger().error(f'Failed to send response: {e}')

    def destroy_node(self):
        """Clean up on shutdown"""
        if self.is_recording:
            self.stop_recording()

        if self.sensors_running:
            self.stop_sensors()

        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HMIBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
