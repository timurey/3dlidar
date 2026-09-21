"""
CYD HMI Manager Node.

Bridges ESP32 CYD touchscreen panel with ROS2 bag recording.

Serial protocol (115200 baud, /dev/ttyUSB0):
  RX from CYD: {"cmd": "start_recording"} / {"cmd": "stop_recording"}
  TX to CYD:   {"sensors_running": bool, "lidar_hz": float, "imu_hz": float,
                 "lidar_ok": bool, "imu_ok": bool, "recording": bool,
                 "disk_gb": float, "rec_duration": int, "bag_name": str}

Usage:
  ros2 run hmi_manager hmi_manager_node \
    --ros-args -p port:=/dev/ttyUSB0 -p bag_dir:=/home/openclaw/bags
"""

import json
import os
import shutil
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String

import serial

# How long without a message before topic is considered dead
TOPIC_TIMEOUT = 3.0
# Status send interval to CYD (seconds)
STATUS_INTERVAL = 0.5


class HmiManagerNode(Node):

    def __init__(self):
        super().__init__('hmi_manager')

        self.declare_parameter('port',        '/dev/ttyUSB0')
        self.declare_parameter('baud',        115200)
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        self._lock = threading.Lock()

        # Bridge status cache — populated from /hmi_bridge/status
        self._bridge: dict = {}
        self._bridge_time: float = 0.0

        # Publisher → hmi_bridge command bus
        self._cmd_pub = self.create_publisher(String, '/hmi_bridge/cmd', 10)

        # Subscribe to unified bridge status
        self.create_subscription(String, '/hmi_bridge/status', self._cb_bridge_status, 10)

        # Status send timer
        self.create_timer(STATUS_INTERVAL, self._send_status)

        # Open serial port to CYD
        self._serial = None
        try:
            self._serial = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'CYD connected on {port} at {baud} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open CYD port {port}: {e}')

        # Serial reader thread
        self._running = True
        if self._serial:
            self._reader = threading.Thread(target=self._read_loop, daemon=True)
            self._reader.start()


    # ── Bridge status ─────────────────────────────────────────────────────────

    def _cb_bridge_status(self, msg: String):
        try:
            s = json.loads(msg.data)
        except Exception:
            return
        with self._lock:
            self._bridge = s
            self._bridge_time = time.monotonic()

    # ── CYD serial reader ─────────────────────────────────────────────────────

    def _read_loop(self):
        buf = ''
        while self._running and self._serial:
            try:
                raw = self._serial.read(256)
                if not raw:
                    continue
                buf += raw.decode('utf-8', errors='ignore')
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    line = line.strip()
                    if line.startswith('{'):
                        self._handle_cmd(line)
            except serial.SerialException as e:
                self.get_logger().error(f'CYD read error: {e}')
                time.sleep(1.0)

    def _handle_cmd(self, line: str):
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            return
        cmd = msg.get('cmd', '')
        if cmd == 'start_recording':
            self._start_recording()
        elif cmd == 'stop_recording':
            self._stop_recording()
        elif cmd == 'shutdown':
            self._shutdown()

    # ── Recording — delegated to hmi_bridge ──────────────────────────────────

    def _start_recording(self):
        self.get_logger().info('CYD: requesting start_record via hmi_bridge')
        msg = String()
        msg.data = json.dumps({'action': 'start_record', 'mode': 'static'})
        self._cmd_pub.publish(msg)

    def _stop_recording(self):
        self.get_logger().info('CYD: requesting stop_record via hmi_bridge')
        msg = String()
        msg.data = json.dumps({'action': 'stop_record'})
        self._cmd_pub.publish(msg)

    def _shutdown(self):
        self.get_logger().info('Shutdown requested from HMI')
        self._stop_recording()
        import subprocess
        subprocess.Popen('echo openclaw | sudo -S shutdown -h now', shell=True)

    # ── Status sender ─────────────────────────────────────────────────────────

    def _send_status(self):
        with self._lock:
            b = dict(self._bridge)
            fresh = (time.monotonic() - self._bridge_time) < 3.0 and bool(b)

        if fresh:
            status = {
                'sensors_running': b.get('sensors_running', False),
                'lidar_hz':        b.get('lidar_raw_hz', 0.0),
                'imu_hz':          b.get('imu_hz', 0.0),
                'lidar_ok':        b.get('lidar_raw_ok', False),
                'imu_ok':          b.get('imu_ok', False),
                'recording':       b.get('recording', False),
                'disk_gb':         b.get('disk_gb', 0.0),
                'rec_duration':    b.get('rec_duration', 0),
                'bag_name':        b.get('bag_name', ''),
            }
        else:
            status = {
                'sensors_running': False,
                'lidar_hz': 0.0, 'imu_hz': 0.0,
                'lidar_ok': False, 'imu_ok': False,
                'recording': False, 'disk_gb': 0.0,
                'rec_duration': 0, 'bag_name': '',
            }

        if self._serial and self._serial.is_open:
            try:
                line = json.dumps(status) + '\n'
                self._serial.write(line.encode())
            except serial.SerialException:
                pass

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._running = False
        if self._serial and self._serial.is_open:
            self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HmiManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
