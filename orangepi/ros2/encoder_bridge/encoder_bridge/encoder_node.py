#!/usr/bin/env python3
"""
ROS2 node that reads rotating platform encoder data from Teensy via UART
and publishes to /rotating_platform/encoder topic.

Teensy sends CSV format: <timestamp_ms>,<angle_deg>,<velocity_rpm>
Example: 1142165,-64687.33,0.02
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from sensor_msgs.msg import JointState
import serial
import math
import time
import signal
import sys
import os
import errno
import traceback
import threading


class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyAMA4')
        self.declare_parameter('baud_rate', 230400)
        self.declare_parameter('publish_rate', 200.0)  # Expected 200 Hz from Teensy
        self.declare_parameter('max_rpm', 80.0)  # Reject encoder jumps beyond this
        self.declare_parameter('reconnect_delay', 5.0)  # Seconds before reconnect attempt
        self.declare_parameter('max_reconnect_attempts', 0)  # 0 = unlimited

        # Get parameters
        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        self.max_rpm = float(self.get_parameter('max_rpm').value)
        self.reconnect_delay = self.get_parameter('reconnect_delay').value
        self.max_reconnect_attempts = self.get_parameter('max_reconnect_attempts').value

        # Create publisher for angle (simple Float64 for now)
        self.angle_pub = self.create_publisher(Float64, '/rotating_platform/angle', 10)
        self.velocity_pub = self.create_publisher(Float64, '/rotating_platform/velocity', 10)

        # Motor command subscriber — writes single-char commands to Teensy via serial
        self._write_lock = threading.Lock()
        self.motor_cmd_sub = self.create_subscription(
            String, '/motor_cmd', self._motor_cmd_callback, 10)
        # Stamped state for offline deskew: carries the Pi-synced per-sample timestamp
        # (header.stamp = ros_time_ns) that the bare Float64 /angle topic lacks. Record
        # this and the offline deskewer can interpolate platform angle at each lidar
        # point's exact time instead of relying on the bag's jittery record-time.
        self.state_pub = self.create_publisher(JointState, '/rotating_platform/joint_state', 10)

        # Teensy time synchronization (ms -> ROS time)
        self.teensy_time_offset_ns = None
        self.last_teensy_ts_ms = None
        self.last_angle_deg = None
        self.last_angle_raw_deg = None

        # Statistics for diagnostics
        self.rx_buffer_max = 0
        self.checksum_errors = 0
        self.parse_errors = 0
        self.valid_messages = 0
        self.consecutive_read_errors = 0
        self.last_stats_time = time.time()

        # Data watchdog: detect silent serial stalls where port is "open"
        # but UART has stopped delivering data (common on Pi 5 PL011 under load)
        self.declare_parameter('data_timeout', 12.0)  # Seconds with no valid data before reconnect
        # 12s: Teensy USB CDC init + SimpleFOC initFOC() takes 3-5s after a USB reset.
        # 3s was too tight — data watchdog fired before the Teensy could reconnect, causing
        # an infinite close→reconnect→timeout loop that pkill could not escape.
        self.data_timeout = self.get_parameter('data_timeout').value

        # Serial connection state
        self.serial_port = None
        self.serial_connected = False
        self.reconnect_attempts = 0

        # Reconnect scheduling via monotonic timestamp — avoids create_timer() from
        # background threads. The executor sets _reconnect_at (a simple float assignment,
        # non-blocking). The serial reader thread checks and acts on it.
        # Initial value: connect immediately at startup.
        self._reconnect_at = time.monotonic()  # serial thread connects on first loop tick

        # Thread-safe state: serial thread updates these, ROS callbacks read them
        self._lock = threading.Lock()
        self._latest_msg = None  # (angle_deg, angle_raw, velocity_rpm, ros_time_ns)
        self._serial_thread_alive = time.monotonic()  # Heartbeat from serial thread (updated every loop)
        self._last_data_time = time.monotonic()       # Only updated on valid parsed message
        self._serial_thread = None
        self._shutdown = False

        # Start serial reader thread (handles all serial open/close/read — never blocks executor)
        # NOTE: _connect_serial() is called exclusively from this thread, never from the
        # ROS executor. This prevents serial.Serial() hangs from freezing the executor.
        self._start_serial_thread()

        # Start raw Python daemon watchdog — fires even when ROS executor is stalled
        # (Pi 5 PL011 AXI kernel spinlock can stall the executor too, killing ROS timers)
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_thread_fn, daemon=True, name='watchdog'
        )
        self._watchdog_thread.start()

        # Publish timer — runs in ROS executor, publishes latest data from serial thread
        self.publish_timer = self.create_timer(0.005, self._publish_latest)  # 200 Hz check

        # Secondary watchdog timer in ROS executor (belt-and-suspenders fallback)
        self.watchdog_timer = self.create_timer(1.0, self._watchdog_check)

        # Diagnostic timer (log stats every 10 seconds)
        self.stats_timer = self.create_timer(10.0, self.log_statistics)

        # Signal handler to log WHY we're being killed
        for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP, signal.SIGABRT):
            signal.signal(sig, self._signal_handler)

        self.get_logger().info('Encoder node started (threaded serial reader)')

    def _signal_handler(self, signum, frame):
        """Log signal before dying so we know what killed us."""
        sig_name = signal.Signals(signum).name
        self.get_logger().fatal(
            f'RECEIVED SIGNAL {sig_name} ({signum}) — encoder node being killed!'
        )
        print(f'[ENCODER FATAL] Signal {sig_name} ({signum})', file=sys.stderr, flush=True)
        if frame:
            print(f'[ENCODER FATAL] Stack: {"".join(traceback.format_stack(frame))}',
                  file=sys.stderr, flush=True)
        sys.exit(128 + signum)

    def _start_serial_thread(self):
        """Start (or restart) the serial reader thread."""
        if self._serial_thread is not None and self._serial_thread.is_alive():
            return  # Already running
        self._serial_thread = threading.Thread(
            target=self._serial_reader_loop, daemon=True, name='serial_reader'
        )
        self._serial_thread.start()
        self.get_logger().info('Serial reader thread started')

    def _connect_serial(self):
        """
        Open the serial port. MUST only be called from the serial reader thread.

        NEVER call this from the ROS executor thread. The Pi 5 PL011 AXI UART
        driver can hang indefinitely inside serial.Serial() (blocking ioctls during
        open) when the driver is in a bad state under I/O load. If called from the
        executor, this freezes the entire ROS executor — all timers, publishers, and
        watchdogs stop firing, and the node becomes permanently stuck.

        By running here (serial thread), a hang causes the thread heartbeat to stop,
        which the daemon watchdog detects after data_timeout seconds and kills the
        process for a clean respawn.
        """
        try:
            if self.serial_port is not None:
                try:
                    self.serial_port.close()
                except Exception:
                    pass

            self.serial_port = serial.Serial(self.port, self.baud, timeout=0, exclusive=False)
            # Flush any stale data in kernel/hardware buffers
            self.serial_port.reset_input_buffer()
            self.serial_port.reset_output_buffer()
            self.serial_connected = True
            self.reconnect_attempts = 0
            self.consecutive_read_errors = 0
            # Reset time sync on reconnect
            self.teensy_time_offset_ns = None
            self.last_teensy_ts_ms = None
            with self._lock:
                self._serial_thread_alive = time.monotonic()
                self._last_data_time = time.monotonic()
            self.get_logger().info(
                f'Opened serial port {self.port} at {self.baud} baud'
            )
        except Exception as e:
            self.serial_connected = False
            self.reconnect_attempts += 1
            self.get_logger().error(
                f'Failed to open serial port {self.port}: {e} '
                f'(attempt {self.reconnect_attempts})'
            )
            # Schedule next attempt via timestamp — serial thread will retry
            self._reconnect_at = time.monotonic() + self.reconnect_delay

    def _serial_reader_loop(self):
        """
        Runs in a separate thread. Handles serial connect/reconnect and data reading.

        ALL serial.Serial() open/close calls happen here — never in the ROS executor.
        This is critical: if the Pi 5 PL011 AXI UART driver hangs during open(),
        only this thread stalls. The ROS executor stays alive (timers, publishers, and
        watchdogs keep firing). The daemon watchdog detects the thread stall after
        data_timeout seconds and calls os._exit(1) for a clean respawn.

        Reconnect scheduling uses a monotonic timestamp (_reconnect_at) instead of
        ROS timers. The executor sets _reconnect_at (a non-blocking Python assignment).
        This thread checks and acts on it — no create_timer() from background threads.
        """
        rx_buffer = bytearray()

        while not self._shutdown:
            # ── Disconnected: wait for reconnect time, then connect ──────────
            if not self.serial_connected:
                # Update heartbeat so daemon watchdog doesn't kill us while waiting
                with self._lock:
                    self._serial_thread_alive = time.monotonic()

                ra = self._reconnect_at
                if ra is not None and time.monotonic() >= ra:
                    # Time to attempt connect. Heartbeat stops updating inside
                    # _connect_serial() — if it hangs >data_timeout, daemon
                    # watchdog fires os._exit(1) for respawn. This is intentional.
                    self._reconnect_at = None
                    self.get_logger().info('Attempting serial connect/reconnect...')
                    self._connect_serial()
                else:
                    time.sleep(0.1)
                continue

            # ── Connected: read serial data ──────────────────────────────────
            try:
                # Use os.read() directly with O_NONBLOCK — this is the only syscall
                # path that avoids the Pi 5 PL011 AXI poll/FIONREAD bug entirely.
                #
                # Both select() and pyserial's read() internally call the driver's
                # poll() function, which hangs under load on ttyAMA4. os.read() on an
                # O_NONBLOCK fd goes through the VFS tty buffer directly, raising
                # BlockingIOError (EAGAIN) immediately when the buffer is empty —
                # no poll syscall, no ioctl(FIONREAD).
                #
                # pyserial sets O_NONBLOCK on the fd when timeout=0 (see serialposix.py
                # _reconfigure_port). We force-set it here as a belt-and-suspenders
                # measure in case pyserial's internal state ever changes the flag.
                try:
                    data = os.read(self.serial_port.fileno(), 4096)
                except BlockingIOError:
                    # EAGAIN — no data available, update heartbeat and loop
                    time.sleep(0.002)
                    with self._lock:
                        self._serial_thread_alive = time.monotonic()
                    continue
                except OSError as e:
                    if e.errno == errno.EINTR:
                        with self._lock:
                            self._serial_thread_alive = time.monotonic()
                        continue
                    raise serial.SerialException(f'os.read() failed: {e}')

                if not data:
                    with self._lock:
                        self._serial_thread_alive = time.monotonic()
                    continue

                rx_buffer.extend(data)

                # Limit buffer size
                if len(rx_buffer) > 4096:
                    sync_idx = rx_buffer.find(b'$')
                    if sync_idx > 0:
                        rx_buffer = rx_buffer[sync_idx:]
                    elif sync_idx == -1:
                        rx_buffer.clear()

                # Process complete lines
                while b'\n' in rx_buffer:
                    raw_line, _, remainder = rx_buffer.partition(b'\n')
                    rx_buffer = bytearray(remainder)

                    line = raw_line.decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue

                    parts, error_reason = self.parse_framed_line(line)
                    if parts is None:
                        self.checksum_errors += 1
                        continue

                    try:
                        teensy_timestamp_ms = int(parts[0])
                        angle_raw = float(parts[1])
                        velocity_rpm = float(parts[2])

                        angle_deg = self.normalize_angle(angle_raw)

                        # Reject implausible jumps
                        if self.last_teensy_ts_ms is not None and self.last_angle_raw_deg is not None:
                            dt = (teensy_timestamp_ms - self.last_teensy_ts_ms) / 1000.0
                            if dt <= 0.0 or dt > 0.2:
                                self.teensy_time_offset_ns = None
                            else:
                                max_deg_per_sec = self.max_rpm * 6.0
                                max_delta = max_deg_per_sec * dt * 1.5
                                delta = abs(angle_raw - self.last_angle_raw_deg)
                                if delta > max_delta:
                                    continue

                        # Compute ROS timestamp
                        now_ns = self.get_clock().now().nanoseconds
                        teensy_time_ns = int(teensy_timestamp_ms * 1e6)
                        if self.teensy_time_offset_ns is None:
                            self.teensy_time_offset_ns = now_ns - teensy_time_ns
                        ros_time_ns = teensy_time_ns + self.teensy_time_offset_ns

                        # Store for publishing by the ROS executor thread
                        with self._lock:
                            self._latest_msg = (angle_deg, angle_raw, velocity_rpm, ros_time_ns)
                            self._serial_thread_alive = time.monotonic()
                            self._last_data_time = time.monotonic()

                        self.last_teensy_ts_ms = teensy_timestamp_ms
                        self.last_angle_deg = angle_deg
                        self.last_angle_raw_deg = angle_raw
                        self.valid_messages += 1
                        # NOTE: Do NOT call get_logger() here. The serial reader thread
                        # must never block on the ROS log pipe — if the async log writer
                        # backs up (slow disk), pipe_write blocks the serial thread,
                        # halting all reads and causing encoder_hz to drop to zero.

                    except (ValueError, IndexError):
                        self.parse_errors += 1

                # Track max buffer size
                self.rx_buffer_max = max(self.rx_buffer_max, len(rx_buffer))

            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'Serial connection lost: {e}')
                self.serial_connected = False
                # _connect_serial() will close and reopen on next reconnect attempt.
                # Do NOT close the port here — serial_port.close() can also hang on PL011
                # under load; let _connect_serial() handle it with the daemon watchdog
                # able to detect and kill if it hangs.
                self._reconnect_at = time.monotonic() + self.reconnect_delay
                with self._lock:
                    self._serial_thread_alive = time.monotonic()
            except Exception as e:
                self.consecutive_read_errors += 1
                self.get_logger().error(
                    f'Serial read error ({self.consecutive_read_errors}): {e}',
                    throttle_duration_sec=1.0
                )
                if self.consecutive_read_errors >= 10:
                    self.serial_connected = False
                    self.consecutive_read_errors = 0
                    self._reconnect_at = time.monotonic() + self.reconnect_delay
                with self._lock:
                    self._serial_thread_alive = time.monotonic()

    def _publish_latest(self):
        """Timer callback (ROS executor): publish latest message from serial thread."""
        with self._lock:
            msg = self._latest_msg
            self._latest_msg = None

        if msg is None:
            # No new data — ensure a reconnect is scheduled if we're disconnected
            if not self.serial_connected and self._reconnect_at is None:
                self.get_logger().warn('No reconnect scheduled — scheduling now')
                self._reconnect_at = time.monotonic() + self.reconnect_delay
            return

        angle_deg, angle_raw, velocity_rpm, ros_time_ns = msg

        # Publish angle
        angle_msg = Float64()
        angle_msg.data = angle_deg
        self.angle_pub.publish(angle_msg)

        # Publish velocity
        velocity_msg = Float64()
        velocity_msg.data = velocity_rpm
        self.velocity_pub.publish(velocity_msg)

        # Stamped state for offline deskew. header.stamp = ros_time_ns is the Pi's
        # estimate (from the Teensy ms counter) of when this sample was measured, on the
        # SAME wall clock the Hesai driver stamps points with (use_timestamp_type:1).
        # That shared clock is what lets the offline deskewer interpolate platform angle
        # at each point's exact time WITHOUT using the bag's record-time (log_time),
        # which carries tens of ms of scheduling jitter. position = CONTINUOUS (unwrapped)
        # platform angle in radians, so no 0/2pi unwrap is needed downstream.
        state_msg = JointState()
        state_msg.header.stamp.sec = int(ros_time_ns // 1_000_000_000)
        state_msg.header.stamp.nanosec = int(ros_time_ns % 1_000_000_000)
        state_msg.name = ['platform']
        state_msg.position = [math.radians(angle_raw)]
        state_msg.velocity = [float(velocity_rpm) * 2.0 * math.pi / 60.0]
        self.state_pub.publish(state_msg)

    def _watchdog_thread_fn(self):
        """
        Watchdog running in a raw Python daemon thread — completely independent of
        the ROS executor. If the Pi 5 PL011 AXI kernel spinlock fires, it can stall
        the entire ROS executor (and therefore the ROS-timer watchdog). This thread
        uses only time.sleep() and os._exit(), both of which are immune to executor
        stalls. It will fire even if the ROS executor is permanently frozen.

        IMPORTANT: This thread ONLY handles hard thread-stall detection (os._exit).
        It does NOT manage reconnect scheduling. The serial reader thread handles all
        serial operations and reconnect timing via _reconnect_at timestamp.
        """
        while not self._shutdown:
            time.sleep(1.0)
            with self._lock:
                last_heartbeat = self._serial_thread_alive

            # Thread stall check (PL011 hang / executor freeze) — kill for respawn
            thread_stall = time.monotonic() - last_heartbeat
            if thread_stall > self.data_timeout:
                print(
                    f'[ENCODER WATCHDOG THREAD] Serial thread stalled {thread_stall:.1f}s '
                    f'(timeout={self.data_timeout}s). Killing process for respawn.',
                    file=sys.stderr, flush=True
                )
                os._exit(1)

    def _watchdog_check(self):
        """
        Secondary watchdog running in the ROS executor (belt-and-suspenders fallback).

        IMPORTANT: This method MUST NOT call serial.Serial(), serial_port.close(), or
        any blocking serial operation. The Pi 5 PL011 AXI UART driver can hang
        indefinitely inside these calls under I/O load, which would freeze the entire
        ROS executor. Instead, we only set _reconnect_at (a non-blocking Python
        assignment). The serial reader thread detects this and handles the reconnect.
        """
        with self._lock:
            last_heartbeat = self._serial_thread_alive
            last_data = self._last_data_time
            connected = self.serial_connected

        # Thread stall — kill for respawn
        thread_stall = time.monotonic() - last_heartbeat
        if thread_stall > self.data_timeout:
            self.get_logger().fatal(
                f'WATCHDOG: Serial thread stalled for {thread_stall:.1f}s! '
                f'Killing process for respawn.'
            )
            os._exit(1)

        # Data silence while connected — trigger reconnect via serial thread
        if connected:
            data_silence = time.monotonic() - last_data
            if data_silence > self.data_timeout:
                self.get_logger().warn(
                    f'WATCHDOG: No valid data for {data_silence:.1f}s — scheduling reconnect.'
                )
                # Mark disconnected and set reconnect timestamp.
                # Do NOT call serial_port.close() or serial.Serial() here — those can
                # block on PL011 and freeze the executor. The serial reader thread will
                # close and reopen the port in _connect_serial().
                self.serial_connected = False
                self._reconnect_at = time.monotonic() + self.reconnect_delay

        # Also check if serial thread died unexpectedly
        if self._serial_thread is not None and not self._serial_thread.is_alive():
            self.get_logger().error('Serial reader thread died, restarting it')
            self._start_serial_thread()

    def _motor_cmd_callback(self, msg: String):
        """Write a single-char motor command to the Teensy via the open serial port."""
        cmd = msg.data.strip()
        valid = {'x', 'r', 'm', 's', '+', '-'}
        if cmd not in valid:
            self.get_logger().warn(f'Unknown motor command: {cmd!r}')
            return
        if not self.serial_connected or self.serial_port is None:
            self.get_logger().warn('Motor command ignored: serial not connected')
            return
        try:
            with self._write_lock:
                self.serial_port.write(f'{cmd}\n'.encode())
            self.get_logger().info(f'Motor command sent: {cmd!r}')
        except Exception as e:
            self.get_logger().error(f'Failed to write motor command: {e}')

    def normalize_angle(self, angle_deg):
        """Normalize angle to 0-360 range"""
        normalized = angle_deg % 360.0
        if normalized < 0:
            normalized += 360.0
        return normalized

    def angle_diff_deg(self, a_deg, b_deg):
        """Smallest signed difference a - b (degrees)"""
        return (a_deg - b_deg + 180.0) % 360.0 - 180.0

    def parse_framed_line(self, line):
        """
        Parse checksum-framed line: $ts,angle,rpm*CS
        CS is XOR of payload bytes between $ and *
        Returns (parts, error_reason) tuple
        """
        if not line.startswith('$') or '*' not in line:
            return None, 'invalid_format'

        try:
            payload, checksum_str = line[1:].split('*', 1)
        except ValueError:
            return None, 'no_checksum_delim'

        checksum_str = checksum_str.strip()
        if len(checksum_str) < 2:
            return None, 'checksum_too_short'

        checksum_calc = 0
        for b in payload.encode('ascii', errors='ignore'):
            checksum_calc ^= b

        try:
            checksum_rx = int(checksum_str[:2], 16)
        except ValueError:
            return None, 'checksum_invalid_hex'

        if checksum_calc != checksum_rx:
            return None, f'checksum_mismatch(calc={checksum_calc:02x},rx={checksum_rx:02x})'

        parts = payload.split(',')
        if len(parts) != 3:
            return None, f'wrong_field_count({len(parts)})'

        return parts, None

    def log_statistics(self):
        """Log diagnostic statistics every 10 seconds"""
        now = time.time()
        mono_now = time.monotonic()
        elapsed = now - self.last_stats_time
        msg_rate = self.valid_messages / elapsed if elapsed > 0 else 0

        with self._lock:
            thread_age = mono_now - self._serial_thread_alive
        thread_ok = self._serial_thread is not None and self._serial_thread.is_alive()

        ra = self._reconnect_at
        reconnect_in = max(0.0, ra - time.monotonic()) if ra is not None else None
        reconnect_str = f'{reconnect_in:.1f}s' if reconnect_in is not None else 'none'

        self.get_logger().info(
            f'Encoder stats: {self.valid_messages} msgs in {elapsed:.1f}s '
            f'({msg_rate:.1f} Hz), checksum_errors={self.checksum_errors}, '
            f'parse_errors={self.parse_errors}, max_buffer={self.rx_buffer_max}B, '
            f'thread={"OK" if thread_ok else "DEAD"}, heartbeat={thread_age:.1f}s ago, '
            f'connected={self.serial_connected}, reconnect_in={reconnect_str}'
        )

        # Reset counters
        self.valid_messages = 0
        self.checksum_errors = 0
        self.parse_errors = 0
        self.rx_buffer_max = 0
        self.last_stats_time = now

    def destroy_node(self):
        """Clean up serial port on shutdown"""
        self._shutdown = True
        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info('Closed serial port')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = EncoderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'[ENCODER] Fatal exception: {e}', file=sys.stderr, flush=True)
        traceback.print_exc(file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
