"""
ROS2 driver for VLP-16 platform spin controller (RP2040 + SimpleFOC).

Subscribes to UART stream from RP2040, publishes platform angle, RPM,
and a stamped JointState for per-point deskew interpolation.
Provides start/stop services and target RPM control.

UART protocol: /dev/ttyS3, 230400 baud, 8N1
  RX: $T,<millis>,<angle_deg>,<rpm>*CS  — telemetry at 100 Hz
  TX: $START*CS / $STOP*CS / $SETRPM,<rpm>*CS / $STATUS*CS / $DIAG*CS
"""

import math
import os
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray, KeyValue
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import serial


def _checksum(payload: str) -> str:
    cs = 0
    for b in payload.encode():
        cs ^= b
    return f'{cs:02X}'


def _make_frame(payload: str) -> bytes:
    return f'${payload}*{_checksum(payload)}\n'.encode()


def _parse_frame(line: str):
    """Return list of payload fields or None if invalid."""
    line = line.strip()
    if not line.startswith('$') or '*' not in line:
        return None
    payload, _, cs_hex = line[1:].rpartition('*')
    if _checksum(payload) != cs_hex.upper():
        return None
    return payload.split(',')


class SpinControllerNode(Node):

    def __init__(self):
        super().__init__('spin_controller')

        # Parameters
        self.declare_parameter('port', '/dev/ttyS3')
        self.declare_parameter('baud', 230400)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('target_rpm', 40.0)
        self.declare_parameter('diag_period', 5.0)
        self.declare_parameter('data_timeout', 10.0)   # sec без данных → watchdog kill

        port         = self.get_parameter('port').value
        baud         = self.get_parameter('baud').value
        self._auto_start   = self.get_parameter('auto_start').value
        self._target_rpm   = self.get_parameter('target_rpm').value
        diag_period        = self.get_parameter('diag_period').value
        self._data_timeout = self.get_parameter('data_timeout').value

        # State
        self._state = 'UNKNOWN'
        self._fault = 0
        self._lock  = threading.Lock()
        self._ack_event  = threading.Event()
        self._ack_result = None
        self._serial: serial.Serial | None = None

        # RP2040 time synchronisation (millis → ROS ns)
        # Обновляется каждые ~10 сек: медиана последних 5 оценок offset,
        # чтобы компенсировать дрейф кварца RP2040 относительно host clock.
        self._rp2040_time_offset_ns: int | None = None
        self._offset_samples: list[int] = []     # последние оценки (до 5)
        self._offset_last_update: float = 0.0    # time.monotonic() последнего обновления
        self._OFFSET_UPDATE_INTERVAL = 10.0      # сек между пересчётами
        self._OFFSET_WINDOW = 5                  # количество оценок для медианы

        # Continuous (unwrapped) angle tracking
        self._angle_unwrapped_deg: float | None = None   # None до первого сообщения
        self._angle_prev_raw_deg:  float | None = None

        # Watchdog: monotonic timestamp последнего валидного T-сообщения
        self._last_data_time = time.monotonic()
        self._shutdown = False

        # TF broadcaster
        self._tf_broadcaster = TransformBroadcaster(self)
        self.declare_parameter('base_frame',     'platform_base')
        self.declare_parameter('rotating_frame', 'platform_rotating')

        # Publishers — topic names match PandarMapper convention for HMI/deskew compatibility
        self._pub_angle = self.create_publisher(Float64,    '/rotating_platform/angle',      10)
        self._pub_rpm   = self.create_publisher(Float64,    '/rotating_platform/velocity',   10)
        self._pub_joint = self.create_publisher(JointState, '/rotating_platform/joint_state', 10)
        self._pub_diag  = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # Services
        self.create_service(Trigger, '~/start', self._srv_start)
        self.create_service(Trigger, '~/stop',  self._srv_stop)

        # Target RPM via topic
        self.create_subscription(Float64, '~/target_rpm', self._on_target_rpm, 10)

        # Open serial port
        try:
            self._serial = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Opened {port} at {baud} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Cannot open {port}: {e}')
            return

        # UART reader thread
        self._running = True
        self._reader_thread = threading.Thread(
            target=self._reader_loop, daemon=True, name='spin_serial')
        self._reader_thread.start()

        # Daemon watchdog thread — независим от ROS executor.
        # Если reader thread завис (UART hang) → os._exit(1) для respawn сервиса.
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, daemon=True, name='spin_watchdog')
        self._watchdog_thread.start()

        # Periodic diagnostics timer
        self.create_timer(diag_period, self._poll_diag)

        # Auto-start after brief delay (let ROS graph settle)
        self._auto_start_timer = None
        if self._auto_start:
            self._auto_start_timer = self.create_timer(2.0, self._do_auto_start)

    # ── UART reader ───────────────────────────────────────────────────────────

    def _reader_loop(self):
        # readline() вместо read(256): каждое сообщение обрабатывается сразу
        # при приходе '\n' → индивидуальный log_time без batch-jitter.
        while self._running and self._serial:
            try:
                raw = self._serial.readline()
                if not raw:
                    continue
                line = raw.decode('ascii', errors='ignore').strip()
                if line:
                    self._dispatch(line)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                time.sleep(1.0)

    def _dispatch(self, line: str):
        fields = _parse_frame(line)
        if fields is None:
            return

        kind = fields[0]

        if kind == 'T' and len(fields) == 4:
            try:
                millis    = int(fields[1])
                angle_raw = float(fields[2])
                rpm       = float(fields[3])
            except ValueError:
                return

            # ── ROS timestamp из RP2040 millis (с периодическим ресинком) ──
            rp2040_ns = millis * 1_000_000
            now_ns    = self.get_clock().now().nanoseconds
            sample    = now_ns - rp2040_ns          # текущая оценка offset

            mono_now  = time.monotonic()
            if self._rp2040_time_offset_ns is None:
                # первый раз — применяем сразу
                self._rp2040_time_offset_ns = sample
                self._offset_samples = [sample]
                self._offset_last_update = mono_now
            elif mono_now - self._offset_last_update >= self._OFFSET_UPDATE_INTERVAL:
                # раз в 10 сек пересчитываем: медиана 5 последних оценок
                self._offset_samples.append(sample)
                if len(self._offset_samples) > self._OFFSET_WINDOW:
                    self._offset_samples.pop(0)
                sorted_s = sorted(self._offset_samples)
                self._rp2040_time_offset_ns = sorted_s[len(sorted_s) // 2]
                self._offset_last_update = mono_now

            ros_time_ns = rp2040_ns + self._rp2040_time_offset_ns

            # ── Continuous angle (unwrapped) ──────────────────────────────
            if self._angle_prev_raw_deg is None:
                self._angle_unwrapped_deg = angle_raw
            else:
                # Smallest signed delta, обрабатывает переход через 0/360°
                delta = (angle_raw - self._angle_prev_raw_deg + 180.0) % 360.0 - 180.0
                self._angle_unwrapped_deg += delta
            self._angle_prev_raw_deg = angle_raw

            # ── Публикуем Float64 (обратная совместимость) ────────────────
            self._pub_angle.publish(Float64(data=angle_raw))
            self._pub_rpm.publish(Float64(data=rpm))
            self._publish_tf(angle_raw)

            # ── Публикуем JointState со stamped header ────────────────────
            # header.stamp = ROS-время момента измерения (из millis RP2040).
            # position[0] = continuous angle (rad) — без 0/2π разрыва.
            # Это позволяет офлайн-deskew точно интерполировать угол платформы
            # в момент каждой точки лидара по per-point timestamp.
            js = JointState()
            js.header.stamp.sec     = ros_time_ns // 1_000_000_000
            js.header.stamp.nanosec = ros_time_ns %  1_000_000_000
            js.name     = ['platform']
            js.position = [math.radians(self._angle_unwrapped_deg)]
            js.velocity = [rpm * 2.0 * math.pi / 60.0]
            self._pub_joint.publish(js)

            # Обновляем watchdog timestamp
            self._last_data_time = time.monotonic()

        elif kind == 'S' and len(fields) >= 7:
            with self._lock:
                self._state = fields[1]
                self._fault = int(fields[5])
            self._publish_diag()

        elif kind == 'D' and len(fields) == 4:
            sensor_ok  = fields[1] == '1'
            wire_error = int(fields[2])
            bus_ok     = fields[3] == '1'
            self._publish_diag(sensor_ok=sensor_ok, wire_error=wire_error, bus_ok=bus_ok)

        elif kind == 'A' and len(fields) >= 3:
            with self._lock:
                self._ack_result = (fields[1], fields[2])
            self._ack_event.set()

    # ── Watchdog ──────────────────────────────────────────────────────────────

    def _watchdog_loop(self):
        """Daemon watchdog — не зависит от ROS executor.
        Если данных нет дольше data_timeout секунд → os._exit(1) для respawn."""
        while not self._shutdown:
            time.sleep(1.0)
            silence = time.monotonic() - self._last_data_time
            if silence > self._data_timeout:
                print(
                    f'[SPIN WATCHDOG] No encoder data for {silence:.1f}s '
                    f'(timeout={self._data_timeout}s). Killing for respawn.',
                    flush=True)
                os._exit(1)

    # ── TF ───────────────────────────────────────────────────────────────────

    def _publish_tf(self, angle_deg: float):
        rad = math.radians(angle_deg % 360)
        half = rad / 2.0
        t = TransformStamped()
        t.header.stamp    = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter('base_frame').value
        t.child_frame_id  = self.get_parameter('rotating_frame').value
        t.transform.rotation.x = math.sin(half)
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = math.cos(half)
        self._tf_broadcaster.sendTransform(t)

    # ── Send helpers ──────────────────────────────────────────────────────────

    def _send(self, payload: str):
        if self._serial and self._serial.is_open:
            self._serial.write(_make_frame(payload))

    def _send_and_wait(self, payload: str, timeout: float = 6.0):
        self._ack_event.clear()
        self._ack_result = None
        self._send(payload)
        if self._ack_event.wait(timeout):
            with self._lock:
                return self._ack_result
        return None

    # ── Services ──────────────────────────────────────────────────────────────

    def _srv_start(self, _req, resp):
        self.get_logger().info('START requested')
        ack = self._send_and_wait('START', timeout=6.0)
        if ack and ack[1] == 'OK':
            resp.success = True
            resp.message = 'Motor started'
        else:
            detail = ack[2] if ack and len(ack) > 2 else 'timeout'
            resp.success = False
            resp.message = f'START failed: {detail}'
            self.get_logger().warn(resp.message)
        return resp

    def _srv_stop(self, _req, resp):
        self.get_logger().info('STOP requested')
        ack = self._send_and_wait('STOP', timeout=3.0)
        if ack and ack[1] == 'OK':
            resp.success = True
            resp.message = 'Motor stopped'
        else:
            resp.success = False
            resp.message = 'STOP timeout — sent anyway'
            self.get_logger().warn(resp.message)
        return resp

    def _on_target_rpm(self, msg: Float64):
        self.get_logger().info(f'Setting target RPM: {msg.data}')
        self._send(f'SETRPM,{msg.data:.1f}')

    # ── Auto-start ────────────────────────────────────────────────────────────

    def _do_auto_start(self):
        if self._auto_start_timer:
            self._auto_start_timer.cancel()
            self._auto_start_timer = None
        self.get_logger().info(f'Auto-start: RPM={self._target_rpm}')
        self._send(f'SETRPM,{self._target_rpm:.1f}')
        time.sleep(0.2)
        ack = self._send_and_wait('START', timeout=6.0)
        if ack and ack[1] == 'OK':
            self.get_logger().info('Auto-start: motor running')
        else:
            detail = ack[2] if ack and len(ack) > 2 else 'timeout'
            self.get_logger().warn(f'Auto-start failed: {detail}')

    # ── Diagnostics ───────────────────────────────────────────────────────────

    def _poll_diag(self):
        self._send('STATUS')
        time.sleep(0.1)
        self._send('DIAG')

    def _publish_diag(self, sensor_ok=None, wire_error=None, bus_ok=None):
        with self._lock:
            state = self._state
            fault = self._fault

        status = DiagnosticStatus()
        status.name        = 'spin_controller'
        status.hardware_id = 'rp2040_spin'
        status.level   = (DiagnosticStatus.OK    if state == 'RUNNING'
                          else DiagnosticStatus.ERROR if state == 'FAULT'
                          else DiagnosticStatus.WARN)
        status.message = state
        status.values  = [KeyValue(key='state', value=state),
                          KeyValue(key='fault', value=str(fault))]

        if sensor_ok is not None:
            status.values += [
                KeyValue(key='sensor_ok',  value=str(sensor_ok)),
                KeyValue(key='wire_error', value=str(wire_error)),
                KeyValue(key='bus_ok',     value=str(bus_ok)),
            ]
            if not sensor_ok:
                status.level   = DiagnosticStatus.ERROR
                status.message = f'Encoder error (wire={wire_error})'

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [status]
        self._pub_diag.publish(arr)

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._shutdown = True
        self._running  = False
        if self._serial and self._serial.is_open:
            self._send('STOP')
            self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SpinControllerNode()
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
