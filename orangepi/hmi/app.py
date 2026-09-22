#!/usr/bin/env python3
"""
ROS2 HMI Backend — Flask server for Pi-5 kiosk display.
Monitors topic Hz via background ros2 topic hz processes,
tracks disk usage, and controls bag recording.
"""

import subprocess
import threading
import re
import shutil
import time
import json
import psutil
import urllib.request
import urllib.error
import signal
import os
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String as StringMsg
from flask import Flask, render_template, jsonify, request
from datetime import datetime, timedelta

app = Flask(__name__)

# ── Configuration ─────────────────────────────────────────────────────────────

TOPICS = {
    # Lidar Hz is reported by hmi_bridge in-process, not tracked here.
    'IMU':       '/imu',
    'Encoder':   '/rotating_platform/angle',
}

# Topics recorded per mode
RECORD_TOPICS = {
    'static': [
        '/velodyne_points',
        '/rotating_platform/angle',
        '/rotating_platform/velocity',
        '/rotating_platform/joint_state',   # stamped angle for offline per-point deskew
        '/imu',
        '/tf',
        '/tf_static',
    ],
    'slam': [
        '/velodyne_points',
        '/rotating_platform/angle',
        '/rotating_platform/velocity',
        '/rotating_platform/joint_state',   # stamped angle for offline per-point deskew
        '/imu',
        '/tf',
        '/tf_static',
    ],
}

# Which topic cards are active (full opacity) per mode
TOPIC_MODES = {
    'Velodyne':  ['static', 'slam'],
    'IMU':       ['slam'],
    'Encoder':   ['static', 'slam'],
}

# (ok_min_hz, warn_min_hz) — below warn = red, between = yellow, above ok = green
THRESHOLDS = {
    'Velodyne':  (8,   1),
    'IMU':       (100, 20),
    'Encoder':   (150, 50),
}

BAG_DIR       = '/home/openclaw/bags'
BAG_TRASH_DIR = '/home/openclaw/bags_trash'  # discarded bags moved here, never rm -rf'd
ROS_SETUP    = '/opt/ros/jazzy/setup.bash'
WS_SETUP     = os.path.expanduser('~/ros2_ws/install/setup.bash')
VELODYNE_IP  = '192.168.100.201'   # VLP-16 built-in HTTP interface
VELODYNE_SPINDLE_RPM = 600         # RPM used when turning spindle on
HZ_WINDOW    = 5   # samples for ros2 topic hz rolling window
DEAD_AFTER   = 4.0 # seconds with no update → mark as dead

# spin_controller ROS services / topics
SPIN_START_SRV   = '/spin_controller/start'
SPIN_STOP_SRV    = '/spin_controller/stop'
SPIN_RPM_TOPIC   = '/spin_controller/target_rpm'


# ── Subprocess registry — kill on exit ──────────────────────────────────────
# All long-lived Popen children register here so SIGTERM kills them promptly,
# preventing systemd from waiting the full TimeoutStopSec for the cgroup to drain.

_child_procs: list[subprocess.Popen] = []
_child_lock = threading.Lock()


def _register_proc(proc: subprocess.Popen) -> subprocess.Popen:
    with _child_lock:
        _child_procs.append(proc)
    return proc


def _kill_all_children():
    with _child_lock:
        procs = list(_child_procs)
    for proc in procs:
        try:
            # start_new_session=True makes proc.pid the process-group leader,
            # so killpg kills bash + the ros2 child it exec'd.
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass


def _sigterm_handler(signum, frame):
    _kill_all_children()
    raise SystemExit(0)


signal.signal(signal.SIGTERM, _sigterm_handler)


# ── VLP-16 Spindle Control ───────────────────────────────────────────────────

_spindle_rpm_cache: int | None = None   # None = unknown (never queried yet)
_spindle_cache_time: float = 0.0
_SPINDLE_CACHE_TTL = 5.0               # re-query lidar at most every 5 s


def get_lidar_spindle_rpm() -> int | None:
    """Query the VLP-16 web interface for current motor RPM. Returns None on error."""
    global _spindle_rpm_cache, _spindle_cache_time
    now = time.monotonic()
    if _spindle_rpm_cache is not None and (now - _spindle_cache_time) < _SPINDLE_CACHE_TTL:
        return _spindle_rpm_cache
    try:
        url = f'http://{VELODYNE_IP}/cgi/settings.json'
        with urllib.request.urlopen(url, timeout=2) as resp:
            data = json.loads(resp.read())
        rpm = int(data.get('rpm', data.get('motor', {}).get('rpm', -1)))
        _spindle_rpm_cache = rpm
        _spindle_cache_time = now
        return rpm
    except Exception:
        return _spindle_rpm_cache  # return stale value rather than None if available


VELODYNE_SPINDLE_VALID_RPMS = {300, 600}

def set_lidar_spindle(on: bool, rpm: int | None = None) -> tuple[bool, str]:
    """Set VLP-16 spindle RPM. rpm=None uses VELODYNE_SPINDLE_RPM default when on=True.
    VLP-16 HTTP API: POST /cgi/setting with form body rpm=<value>."""
    global _spindle_rpm_cache, _spindle_cache_time
    if not on:
        target_rpm = 0
    else:
        if rpm is not None and rpm in VELODYNE_SPINDLE_VALID_RPMS:
            target_rpm = rpm
        else:
            target_rpm = VELODYNE_SPINDLE_RPM
    try:
        url = f'http://{VELODYNE_IP}/cgi/setting'
        data = f'rpm={target_rpm}'.encode()
        req = urllib.request.Request(url, data=data,
                                     headers={'Content-Type': 'application/x-www-form-urlencoded'},
                                     method='POST')
        with urllib.request.urlopen(req, timeout=3) as resp:
            resp.read()
        _spindle_rpm_cache = target_rpm
        _spindle_cache_time = time.monotonic()
        return True, f'Spindle {"started" if on else "stopped"} ({target_rpm} RPM)'
    except urllib.error.URLError as e:
        return False, f'VLP-16 unreachable: {e.reason}'
    except Exception as e:
        return False, str(e)


# ── Hz Monitor ───────────────────────────────────────────────────────────────

class HzMonitor:
    """
    Spawns one `ros2 topic hz` subprocess per topic in a background thread.
    Continuously parses output and stores the latest value.
    If a process dies (topic gone), restarts after a short delay.
    """

    def __init__(self, topics: dict):
        self._lock = threading.Lock()
        self._hz: dict[str, float | None] = {name: None for name in topics}
        self._last_update: dict[str, float] = {name: 0.0 for name in topics}
        for name, topic in topics.items():
            t = threading.Thread(target=self._monitor, args=(name, topic), daemon=True)
            t.start()

    def _monitor(self, name: str, topic: str):
        pattern = re.compile(r'average rate:\s*([\d.]+)')
        while True:
            try:
                proc = _register_proc(subprocess.Popen(
                    f'source {ROS_SETUP} && source {WS_SETUP} && '
                    f'ros2 topic hz --window {HZ_WINDOW} {topic}',
                    shell=True, executable='/bin/bash',
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                    text=True, start_new_session=True,
                ))
                for line in proc.stdout:
                    m = pattern.search(line)
                    if m:
                        with self._lock:
                            self._hz[name] = round(float(m.group(1)), 1)
                            self._last_update[name] = time.monotonic()
                proc.wait()
            except Exception:
                pass
            # Process ended — topic likely stopped; brief pause then retry
            with self._lock:
                self._hz[name] = 0.0
            time.sleep(3)

    def get(self) -> dict:
        now = time.monotonic()
        with self._lock:
            result = {}
            for name, hz in self._hz.items():
                # If we haven't received an update in a while, treat as dead
                if hz is not None and hz > 0 and (now - self._last_update[name]) > DEAD_AFTER:
                    result[name] = 0.0
                else:
                    result[name] = hz
            return result


# ── Angle Monitor ────────────────────────────────────────────────────────────

class AngleMonitor:
    """
    Streams /rotating_platform/angle via `ros2 topic echo` and keeps
    the latest Float64 value. Falls back to None when topic is dead.
    """
    def __init__(self, topic: str = '/rotating_platform/angle'):
        self._topic = topic
        self._angle: float | None = None
        self._last  = 0.0
        self._lock  = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        pattern = re.compile(r'^data:\s*([\d.eE+\-]+)')
        while True:
            try:
                proc = _register_proc(subprocess.Popen(
                    f'source {ROS_SETUP} && source {WS_SETUP} && '
                    f'ros2 topic echo --no-daemon {self._topic}',
                    shell=True, executable='/bin/bash',
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                    text=True, start_new_session=True,
                ))
                for line in proc.stdout:
                    m = pattern.match(line.strip())
                    if m:
                        with self._lock:
                            self._angle = round(float(m.group(1)), 1)
                            self._last  = time.monotonic()
                proc.wait()
            except Exception:
                pass
            with self._lock:
                self._angle = None
            time.sleep(3)

    def get(self) -> float | None:
        with self._lock:
            if self._angle is None:
                return None
            if time.monotonic() - self._last > 4.0:
                return None
            return self._angle


# ── Velocity Monitor ─────────────────────────────────────────────────────────

class VelocityMonitor:
    """
    Streams /rotating_platform/velocity via `ros2 topic echo` and keeps
    the latest Float64 value. Falls back to None when topic is dead.
    """
    def __init__(self, topic: str = '/rotating_platform/velocity'):
        self._topic = topic
        self._rpm: float | None = None
        self._last  = 0.0
        self._lock  = threading.Lock()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        pattern = re.compile(r'^data:\s*([\d.eE+\-]+)')
        while True:
            try:
                proc = _register_proc(subprocess.Popen(
                    f'source {ROS_SETUP} && source {WS_SETUP} && '
                    f'ros2 topic echo --no-daemon {self._topic}',
                    shell=True, executable='/bin/bash',
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                    text=True, start_new_session=True,
                ))
                for line in proc.stdout:
                    m = pattern.match(line.strip())
                    if m:
                        with self._lock:
                            self._rpm = round(float(m.group(1)), 1)
                            self._last = time.monotonic()
                proc.wait()
            except Exception:
                pass
            with self._lock:
                self._rpm = None
            time.sleep(3)

    def get(self) -> float | None:
        with self._lock:
            if self._rpm is None:
                return None
            if time.monotonic() - self._last > 4.0:
                return None
            return self._rpm


# ── ROS coordinator (rclpy node living in this Flask process) ────────────────
#
# Replaces the previous pattern of spawning `ros2 bag record` as a subprocess.
# Instead we publish a JSON command on /hmi_bridge/cmd; the hmi_bridge node owns
# the in-process rosbag2 SequentialWriter and writes from its own subscription
# callbacks. One persistent subscriber per topic on the DDS bus, no rosbag2
# subscribe/unsubscribe cycles -- which is what was causing the empty-cloud
# burst regression.

class ROSCoordinator:
    """Tiny in-process rclpy node + executor running in a daemon thread.
    Provides synchronous request/response over the /hmi_bridge/cmd <-> /response
    String topics so Flask handlers can call hmi_bridge cleanly."""

    def __init__(self):
        self._lock = threading.Lock()
        # Pending-action -> {'event': Event, 'result': dict|None}
        self._pending: dict[str, dict] = {}
        self._node = None
        self._executor = None
        # Latest full status JSON from hmi_bridge (lidar/imu/encoder Hz etc.).
        # hmi_bridge owns the one efficient PointCloud2 subscriber and publishes
        # the computed status on /hmi_bridge/status ~2 Hz; we just cache it.
        self._status: dict = {}
        self._status_time: float = 0.0

        try:
            rclpy.init()
        except Exception:
            # Already initialized — fine, just continue.
            pass
        self._node = rclpy.create_node('hmi_app_client')
        self._cmd_pub = self._node.create_publisher(
            StringMsg, '/hmi_bridge/cmd', 10
        )
        self._resp_sub = self._node.create_subscription(
            StringMsg, '/hmi_bridge/response', self._on_response, 10
        )
        self._status_sub = self._node.create_subscription(
            StringMsg, '/hmi_bridge/status', self._on_status, 10
        )
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        threading.Thread(target=self._executor.spin, daemon=True).start()

    def _on_status(self, msg):
        try:
            s = json.loads(msg.data)
        except Exception:
            return
        with self._lock:
            self._status = s
            self._status_time = time.monotonic()

    def get_status(self) -> dict:
        """Latest hmi_bridge status, or {} if stale (>3s old) / never received."""
        with self._lock:
            if self._status and (time.monotonic() - self._status_time) < 3.0:
                return dict(self._status)
            return {}

    def _on_response(self, msg):
        try:
            r = json.loads(msg.data)
        except Exception:
            return
        action = r.get('action')
        with self._lock:
            slot = self._pending.get(action)
            if slot is not None:
                slot['result'] = r
                slot['event'].set()

    def call(self, action: str, timeout: float = 5.0, **kwargs) -> dict | None:
        """Publish a command and block until the matching response arrives or
        timeout elapses. Returns the response dict, or None on timeout."""
        event = threading.Event()
        with self._lock:
            self._pending[action] = {'event': event, 'result': None}
        m = StringMsg()
        m.data = json.dumps({'action': action, **kwargs})
        self._cmd_pub.publish(m)
        event.wait(timeout)
        with self._lock:
            slot = self._pending.pop(action, None)
        return (slot or {}).get('result')


ros_coord: ROSCoordinator | None = None  # initialized at module bottom


# ── Bag Recorder ─────────────────────────────────────────────────────────────

class BagRecorder:
    """Thin wrapper that delegates start/stop to hmi_bridge over /hmi_bridge/cmd.
    Keeps the Flask-level state (recording flag, pending bag for the verify
    modal, rec_start for the elapsed timer) but never spawns a subprocess."""

    def __init__(self):
        self._lock        = threading.Lock()
        self.recording    = False
        self.bag_name     = None
        self.mode         = 'static'
        self._rec_start   = None
        # Pending finalization: bag has stopped but user hasn't saved/discarded yet.
        # Drives the post-stop modal in the HMI; cleared on finalize/discard.
        self.pending_bag  = None
        self.pending_mode = None
        os.makedirs(BAG_DIR, exist_ok=True)
        os.makedirs(BAG_TRASH_DIR, exist_ok=True)

    def start(self, mode: str = 'static'):
        with self._lock:
            if self.recording:
                return False, 'Already recording'
            if mode not in RECORD_TOPICS:
                mode = 'static'
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            proposed_name = f'{mode}_{ts}'

            if ros_coord is None:
                return False, 'ROS coordinator not initialized'
            resp = ros_coord.call(
                'start_record', timeout=15.0,  # motor start + 2s wait + bag open
                mode=mode, name=proposed_name
            )
            if resp is None:
                return False, 'No response from hmi_bridge (timeout)'
            if not resp.get('ok'):
                return False, resp.get('msg', 'start_record failed')

            self.bag_name   = resp.get('name', proposed_name)
            self.mode       = mode
            self.recording  = True
            self._rec_start = time.monotonic()
            return True, self.bag_name

    def stop(self):
        with self._lock:
            bridge_rec = ros_coord.get_status().get('recording', False) if ros_coord else False
            if not self.recording and not bridge_rec:
                return False, 'Not recording'
            name = self.bag_name or (ros_coord.get_status().get('bag_name') if ros_coord else None) or 'unknown'
            mode = self.mode
            if ros_coord is not None:
                resp = ros_coord.call('stop_record', timeout=5.0)
                if resp and resp.get('name'):
                    name = resp['name']
            # Even if response was bad, move out of recording state — hmi_bridge
            # will have closed the writer or crashed; either way Flask is no
            # longer recording from its perspective.
            self.recording    = False
            self._rec_start   = None
            self.pending_bag  = name
            self.pending_mode = mode
            self.bag_name     = None
            return True, name

    def clear_pending(self):
        with self._lock:
            self.pending_bag  = None
            self.pending_mode = None


# ── Disk ─────────────────────────────────────────────────────────────────────

def get_disk() -> dict:
    usage = shutil.disk_usage('/')
    return {
        'total_gb': round(usage.total / 1e9, 1),
        'used_gb':  round(usage.used  / 1e9, 1),
        'free_gb':  round(usage.free  / 1e9, 1),
        'pct':      round(usage.used  / usage.total * 100, 1),
    }


# ── System Diagnostics ────────────────────────────────────────────────────────

def get_cpu_temp() -> float | None:
    """Read CPU temperature from sysfs (works on Pi without vcgencmd)."""
    try:
        with open('/sys/class/thermal/thermal_zone0/temp') as f:
            return round(int(f.read().strip()) / 1000.0, 1)
    except Exception:
        return None


def get_throttle() -> dict:
    """vcgencmd is Raspberry Pi specific — not available on Orange Pi 5 Plus."""
    return {'ok': None, 'current': {}, 'history': {}, 'raw': None}


def get_system() -> dict:
    cpu_pct   = psutil.cpu_percent(percpu=True)
    cpu_freq  = psutil.cpu_freq()
    mem       = psutil.virtual_memory()
    boot_time = psutil.boot_time()
    uptime_s  = int(time.time() - boot_time)
    uptime    = str(timedelta(seconds=uptime_s))

    return {
        'cpu_pct':    [round(p, 1) for p in cpu_pct],
        'cpu_avg':    round(sum(cpu_pct) / len(cpu_pct), 1),
        'cpu_freq_mhz': round(cpu_freq.current) if cpu_freq else None,
        'cpu_freq_max': round(cpu_freq.max)     if cpu_freq else None,
        'cpu_temp':   get_cpu_temp(),
        'ram_used_gb':  round(mem.used  / 1e9, 2),
        'ram_total_gb': round(mem.total / 1e9, 2),
        'ram_pct':    round(mem.percent, 1),
        'uptime':     uptime,
        'throttle':   get_throttle(),
    }


# ── Init ─────────────────────────────────────────────────────────────────────

monitor   = HzMonitor(TOPICS)
angle_mon = AngleMonitor()
vel_mon   = VelocityMonitor()
ros_coord = ROSCoordinator()
recorder  = BagRecorder()


# ── Bag Manager ──────────────────────────────────────────────────────────────

def dir_size_bytes(path: str) -> int:
    total = 0
    for dirpath, _, filenames in os.walk(path):
        for f in filenames:
            try:
                total += os.path.getsize(os.path.join(dirpath, f))
            except OSError:
                pass
    return total


def fmt_size(n: int) -> str:
    for unit in ('B', 'KB', 'MB', 'GB'):
        if n < 1024:
            return f'{n:.1f} {unit}'
        n /= 1024
    return f'{n:.1f} TB'


def get_bags() -> list[dict]:
    bags = []
    if not os.path.isdir(BAG_DIR):
        return bags
    for name in sorted(os.listdir(BAG_DIR), reverse=True):
        full = os.path.join(BAG_DIR, name)
        if not os.path.isdir(full):
            continue
        # Skip non-bag items (scripts, docs etc.)
        if not any(f.endswith(('.db3', '.mcap', 'metadata.yaml'))
                   for _, _, files in os.walk(full) for f in files):
            continue
        stat    = os.stat(full)
        size_b  = dir_size_bytes(full)
        mtime   = datetime.fromtimestamp(stat.st_mtime).strftime('%Y-%m-%d %H:%M:%S')
        bags.append({
            'name':    name,
            'size':    fmt_size(size_b),
            'size_b':  size_b,
            'mtime':   mtime,
        })
    return bags


# ── Sensor Control ───────────────────────────────────────────────────────────

def send_motor_cmd(cmd: str) -> tuple[bool, str]:
    """Send motor command to spin_controller via ROS service / topic."""
    if cmd == 'r':
        ros_cmd = (
            f'source {ROS_SETUP} && source {WS_SETUP} && '
            f'ros2 service call {SPIN_START_SRV} std_srvs/srv/Trigger {{}}'
        )
    elif cmd == 'x':
        ros_cmd = (
            f'source {ROS_SETUP} && source {WS_SETUP} && '
            f'ros2 service call {SPIN_STOP_SRV} std_srvs/srv/Trigger {{}}'
        )
    else:
        return False, f'Unsupported command: {cmd!r}'
    try:
        result = subprocess.run(
            ros_cmd, shell=True, executable='/bin/bash',
            capture_output=True, text=True, timeout=10
        )
        ok = result.returncode == 0
        out = (result.stdout or result.stderr or '').strip()
        return ok, out or cmd
    except subprocess.TimeoutExpired:
        return False, 'Service call timed out'
    except Exception as e:
        return False, str(e)


def restart_all_sensors() -> tuple[bool, str]:
    """Restart the hmi_bridge service — kills and relaunches full sensor stack."""
    try:
        subprocess.Popen(
            ['sudo', 'systemctl', 'restart', 'hmi_bridge'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        return True, 'hmi_bridge restarting — sensors back in ~15s'
    except Exception as e:
        return False, str(e)


# ── Routes ───────────────────────────────────────────────────────────────────

@app.route('/')
def index():
    return render_template('index.html', thresholds=THRESHOLDS)


@app.route('/api/status')
def status():
    # Base Hz from app.py's own monitors (IMU, Encoder).
    hz = monitor.get()
    # Lidar Hz comes from hmi_bridge (the single efficient /velodyne_points
    # subscriber) via /hmi_bridge/status, so the dashboard shows it without
    # app.py opening its own multi-MB PointCloud2 subscriber. None = unknown
    # → frontend renders the card as dead, matching real "no data" behavior.
    bridge = ros_coord.get_status() if ros_coord is not None else {}
    hz['Velodyne'] = bridge.get('lidar_raw_hz') if bridge else None
    # Use hmi_bridge as source of truth for recording state — recorder.recording
    # can be False if the start_record response timed out while hmi_bridge still
    # started the bag (motor spin-up adds ~10s before the response arrives).
    is_recording = bridge.get('recording', recorder.recording) if bridge else recorder.recording
    rec_duration = bridge.get('rec_duration', 0) if is_recording else 0
    bag_name = bridge.get('bag_name') or recorder.bag_name
    return jsonify({
        'hz':               hz,
        'encoder_angle':    angle_mon.get(),
        'platform_rpm':     vel_mon.get(),
        'disk':             get_disk(),
        'system':           get_system(),
        'recording':        is_recording,
        'rec_duration':     rec_duration,
        'bag_name':         bag_name,
        'mode':             recorder.mode,
        'pending_bag':      recorder.pending_bag,
        'pending_mode':     recorder.pending_mode,
        'topic_modes':      TOPIC_MODES,
        'lidar_spindle_rpm': get_lidar_spindle_rpm(),
    })


@app.route('/api/record/start', methods=['POST'])
def record_start():
    body = request.get_json(silent=True) or {}
    mode = body.get('mode', 'slam')
    ok, msg = recorder.start(mode)
    return jsonify({'ok': ok, 'msg': msg})


@app.route('/api/record/stop', methods=['POST'])
def record_stop():
    ok, msg = recorder.stop()
    return jsonify({'ok': ok, 'msg': msg})


# ── Post-stop bag finalization (verify / rename / discard) ───────────────────

_BAG_NAME_RE = re.compile(r'^[A-Za-z0-9_\-]{1,64}$')


def sanitize_bag_name(name: str) -> str | None:
    """Return a safe new bag-name or None if invalid. Restricts to [A-Za-z0-9_-],
    1–64 chars, must not start with '-' (would look like a CLI flag)."""
    if not isinstance(name, str):
        return None
    name = name.strip()
    if not name or name.startswith('-') or name.startswith('.'):
        return None
    if not _BAG_NAME_RE.match(name):
        return None
    return name


def _expected_topics_for_mode(mode: str) -> list[str]:
    return list(RECORD_TOPICS.get(mode, RECORD_TOPICS.get('slam', [])))


def verify_bag(bag_name: str, mode: str) -> dict:
    """Read the just-finalized mcap and run quick sanity checks.
    Returns {'ok': bool, 'checks': [{label, status: ok|warn|fail, detail}],
             'summary': str}."""
    import glob as _glob
    bag_path = os.path.join(BAG_DIR, bag_name)
    if not os.path.isdir(bag_path):
        return {'ok': False, 'summary': 'Bag directory not found',
                'checks': [{'label': 'bag', 'status': 'fail', 'detail': bag_path}]}
    mcap_files = sorted(_glob.glob(os.path.join(bag_path, '*.mcap')))
    if not mcap_files:
        return {'ok': False, 'summary': 'No .mcap file in bag',
                'checks': [{'label': 'mcap', 'status': 'fail', 'detail': 'missing'}]}

    try:
        from mcap.reader import make_reader
    except Exception as e:
        return {'ok': False, 'summary': f'mcap lib missing: {e}',
                'checks': [{'label': 'reader', 'status': 'fail', 'detail': str(e)}]}

    from collections import Counter, defaultdict
    counts        = Counter()
    empty_counts  = Counter()
    log_min       = {}
    log_max       = {}
    overall_min   = None
    overall_max   = None
    try:
        with open(mcap_files[0], 'rb') as f:
            for schema, channel, message in make_reader(f).iter_messages():
                t = channel.topic
                counts[t] += 1
                if len(message.data) == 0:
                    empty_counts[t] += 1
                lt = message.log_time
                if log_min.get(t) is None or lt < log_min[t]: log_min[t] = lt
                if log_max.get(t) is None or lt > log_max[t]: log_max[t] = lt
                if overall_min is None or lt < overall_min: overall_min = lt
                if overall_max is None or lt > overall_max: overall_max = lt
    except Exception as e:
        return {'ok': False, 'summary': f'Read failed: {e}',
                'checks': [{'label': 'mcap read', 'status': 'fail', 'detail': str(e)}]}

    duration_s = ((overall_max - overall_min) / 1e9) if (overall_min and overall_max) else 0.0
    checks: list[dict] = []

    # 1. Duration sanity (catches the LiDAR-PTP epoch-0 corruption pattern)
    if duration_s <= 0:
        checks.append({'label': 'Duration', 'status': 'fail', 'detail': '0 s — empty bag'})
    elif duration_s > 60 * 60 * 24 * 365:
        checks.append({'label': 'Duration', 'status': 'fail',
                       'detail': f'{duration_s:.0f}s — corrupted (epoch-0 messages present)'})
    else:
        checks.append({'label': 'Duration', 'status': 'ok', 'detail': f'{duration_s:.1f} s'})

    # 2. Per-topic checks. Rate windows are intentionally loose.
    rate_windows = {
        '/velodyne_points':            (10.0, 20.0, 14.0),
        '/rotating_platform/angle':    (150.0, 210.0, 180.0),
        '/rotating_platform/velocity': (150.0, 210.0, 180.0),
    }
    for t in _expected_topics_for_mode(mode):
        n = counts.get(t, 0)
        e = empty_counts.get(t, 0)
        if n == 0:
            checks.append({'label': t, 'status': 'fail', 'detail': 'no messages'})
            continue
        if e > 0:
            checks.append({'label': t, 'status': 'fail',
                           'detail': f'{e} of {n} are 0-byte (likely LiDAR-PTP regression)'})
            continue
        rate = (n / duration_s) if duration_s > 0 else 0.0
        rmin, rmax, rexp = rate_windows.get(t, (0.0, float('inf'), None))
        if rmin <= rate <= rmax:
            checks.append({'label': t, 'status': 'ok',
                           'detail': f'{n} msgs @ {rate:.1f} Hz'})
        else:
            exp = f' (expect ~{rexp:.0f} Hz)' if rexp is not None else ''
            checks.append({'label': t, 'status': 'warn',
                           'detail': f'{n} msgs @ {rate:.1f} Hz{exp}'})

    # 3. Unexpected-topic note (informational)
    extra = sorted(set(counts) - set(_expected_topics_for_mode(mode)))
    if extra:
        checks.append({'label': 'Extra topics', 'status': 'warn',
                       'detail': ', '.join(extra)})

    fails = sum(1 for c in checks if c['status'] == 'fail')
    warns = sum(1 for c in checks if c['status'] == 'warn')
    ok = fails == 0
    if fails:
        summary = f'{fails} check(s) failed — recording likely bad'
    elif warns:
        summary = f'{warns} warning(s) — usable, but inspect'
    else:
        summary = 'All checks passed'
    return {'ok': ok, 'checks': checks, 'summary': summary}


@app.route('/api/record/verify', methods=['POST'])
def record_verify():
    body = request.get_json(silent=True) or {}
    name = body.get('name') or recorder.pending_bag
    mode = body.get('mode') or recorder.pending_mode or 'slam'
    if not name:
        return jsonify({'ok': False, 'msg': 'No pending bag'}), 400
    if not sanitize_bag_name(name):
        return jsonify({'ok': False, 'msg': 'Invalid name'}), 400
    return jsonify(verify_bag(name, mode))


@app.route('/api/record/finalize', methods=['POST'])
def record_finalize():
    """Save the pending bag. Optionally rename. Clears pending state."""
    body     = request.get_json(silent=True) or {}
    cur_name = body.get('name') or recorder.pending_bag
    new_name = body.get('new_name')
    if not cur_name:
        return jsonify({'ok': False, 'msg': 'No pending bag'}), 400
    if not sanitize_bag_name(cur_name):
        return jsonify({'ok': False, 'msg': 'Invalid current name'}), 400
    src = os.path.join(BAG_DIR, cur_name)
    if not os.path.isdir(src):
        return jsonify({'ok': False, 'msg': 'Bag directory missing on disk'}), 404

    final_name = cur_name
    if new_name and new_name != cur_name:
        clean = sanitize_bag_name(new_name)
        if not clean:
            return jsonify({'ok': False, 'msg': 'Invalid new name (use A-Z a-z 0-9 _ -)'}), 400
        dst = os.path.join(BAG_DIR, clean)
        if os.path.exists(dst):
            return jsonify({'ok': False, 'msg': f'Name "{clean}" already exists'}), 409
        try:
            os.rename(src, dst)
            final_name = clean
        except Exception as e:
            return jsonify({'ok': False, 'msg': f'Rename failed: {e}'}), 500

    recorder.clear_pending()
    return jsonify({'ok': True, 'msg': f'Saved as {final_name}', 'name': final_name})


@app.route('/api/record/discard', methods=['POST'])
def record_discard():
    """Move the pending bag into BAG_TRASH_DIR (no rm -rf). Clears pending state."""
    body = request.get_json(silent=True) or {}
    name = body.get('name') or recorder.pending_bag
    if not name:
        return jsonify({'ok': False, 'msg': 'No pending bag'}), 400
    if not sanitize_bag_name(name):
        return jsonify({'ok': False, 'msg': 'Invalid name'}), 400
    src = os.path.join(BAG_DIR, name)
    if not os.path.isdir(src):
        # Nothing to move — still clear pending so UI returns to normal
        recorder.clear_pending()
        return jsonify({'ok': True, 'msg': 'Bag dir missing — pending cleared'})
    dst_name = name
    dst = os.path.join(BAG_TRASH_DIR, dst_name)
    if os.path.exists(dst):
        dst_name = f"{name}_trashed_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        dst = os.path.join(BAG_TRASH_DIR, dst_name)
    try:
        shutil.move(src, dst)
    except Exception as e:
        return jsonify({'ok': False, 'msg': f'Move-to-trash failed: {e}'}), 500
    recorder.clear_pending()
    return jsonify({'ok': True, 'msg': f'Moved to trash as {dst_name}'})


@app.route('/bags')
def bags_page():
    return render_template('bags.html')


@app.route('/api/bags')
def api_bags():
    bags = get_bags()
    total_b = sum(b['size_b'] for b in bags)
    return jsonify({'bags': bags, 'total': fmt_size(total_b), 'count': len(bags)})


@app.route('/api/bags/<path:name>', methods=['DELETE'])
def api_bag_delete(name):
    # Sanitise — no path traversal
    if '/' in name or '..' in name:
        return jsonify({'ok': False, 'msg': 'Invalid name'}), 400
    full = os.path.join(BAG_DIR, name)
    if not os.path.isdir(full):
        return jsonify({'ok': False, 'msg': 'Not found'}), 404
    import shutil as _shutil
    try:
        _shutil.rmtree(full)
        return jsonify({'ok': True, 'msg': f'Deleted {name}'})
    except Exception as e:
        return jsonify({'ok': False, 'msg': str(e)}), 500


# ── Point-cloud preview (downsampled deskew, computed on-device) ─────────────
# Reuses the calibration/deskew math from ~/Reference/DeskewStudio; see
# bag_preview.py for why this reads a bounded, evenly-spaced subset instead
# of the full bag.

@app.route('/preview/<name>')
def preview_page(name):
    if not sanitize_bag_name(name):
        return 'Invalid bag name', 400
    if not os.path.isdir(os.path.join(BAG_DIR, name)):
        return 'Bag not found', 404
    return render_template('preview.html', bag_name=name)


@app.route('/api/bags/<name>/preview')
def api_bag_preview(name):
    if not sanitize_bag_name(name):
        return jsonify({'ok': False, 'msg': 'Invalid name'}), 400
    bag_path = os.path.join(BAG_DIR, name)
    if not os.path.isdir(bag_path):
        return jsonify({'ok': False, 'msg': 'Bag not found'}), 404

    max_clouds = max(1, min(request.args.get('max_clouds', 25, type=int), 300))
    max_points = max(1000, min(request.args.get('max_points', 150000, type=int), 1000000))
    target_rotations = max(0.5, min(request.args.get('target_rotations', 2.5, type=float), 30.0))
    mount_pitch = max(-30.0, min(request.args.get('mount_pitch', 0.0, type=float), 30.0))
    mount_yaw   = max(-30.0, min(request.args.get('mount_yaw',   0.0, type=float), 30.0))
    angle_offset = max(-180.0, min(request.args.get('angle_offset', 0.0, type=float), 180.0))
    center_z_raw = request.args.get('center_z', None, type=float)
    center_z = max(-0.5, min(center_z_raw, 0.5)) if center_z_raw is not None else None

    try:
        from bag_preview import build_bag_preview
        result = build_bag_preview(bag_path, max_clouds=max_clouds, max_points=max_points,
                                    target_rotations=target_rotations,
                                    mount_pitch_deg=mount_pitch, mount_yaw_deg=mount_yaw,
                                    angle_offset_deg=angle_offset, center_z=center_z)
    except (FileNotFoundError, ValueError) as e:
        return jsonify({'ok': False, 'msg': str(e)}), 400
    except Exception as e:
        return jsonify({'ok': False, 'msg': f'Preview failed: {e}'}), 500

    resp = app.response_class(result['buf'], mimetype='application/octet-stream')
    resp.headers['X-Point-Count']          = str(result['points'])
    resp.headers['X-Cloud-Count']          = str(result['clouds'])
    resp.headers['X-Cloud-Total']          = str(result['clouds_total'])
    resp.headers['X-Low-Coverage-Clouds']  = str(result['low_coverage_clouds'])
    resp.headers['X-Encoder-Source']       = result['encoder_source']
    resp.headers['X-Rotations-Captured']   = str(result['rotations_captured'])
    resp.headers['X-Compute-S']            = str(result['compute_s'])
    resp.headers['X-Gravity-Source']       = result.get('gravity_source', 'none')
    return resp


@app.route('/api/bags/<name>/export')
def api_bag_export(name):
    """Export full-quality point cloud as E57 (or XYZ if pye57 missing).

    Query params:
      fmt=e57|xyz  (default: e57)
      max_points   (default: 1000000)
      target_rotations (default: 10)
    """
    if not sanitize_bag_name(name):
        return jsonify({'ok': False, 'msg': 'Invalid name'}), 400
    bag_path = os.path.join(BAG_DIR, name)
    if not os.path.isdir(bag_path):
        return jsonify({'ok': False, 'msg': 'Bag not found'}), 404

    fmt = request.args.get('fmt', 'e57').lower()
    if fmt not in ('e57', 'xyz', 'las'):
        fmt = 'e57'
    max_points       = max(1000, min(request.args.get('max_points', 1_000_000, type=int), 5_000_000))
    target_rotations = max(1.0,  min(request.args.get('target_rotations', 10.0, type=float), 60.0))

    try:
        from bag_preview import build_bag_preview
        result = build_bag_preview(bag_path,
                                   max_clouds=200,
                                   max_points=max_points,
                                   target_rotations=target_rotations)
    except (FileNotFoundError, ValueError) as e:
        return jsonify({'ok': False, 'msg': str(e)}), 400
    except Exception as e:
        return jsonify({'ok': False, 'msg': f'Export failed: {e}'}), 500

    import numpy as np
    pts = np.frombuffer(result['buf'], dtype=np.float32).reshape(-1, 3)

    # build_bag_preview returns points in WebGL Y-up frame (Y=height).
    # CAD / E57 / LAS expect Z-up (ROS/standard). Apply inverse of _R_zup_to_yup:
    #   _R_zup_to_yup  = [[1,0,0],[0,0,1],[0,-1,0]]
    #   _R_yup_to_zup  = [[1,0,0],[0,0,-1],[0,1,0]]  (transpose, it's orthogonal)
    _R_yup_to_zup = np.array([[1,0,0],[0,0,-1],[0,1,0]], dtype=np.float32)
    pts = pts @ _R_yup_to_zup.T   # now X=ROS X, Y=ROS Y, Z=up

    import io, tempfile
    if fmt == 'e57':
        try:
            import pye57
            tmp = tempfile.NamedTemporaryFile(suffix='.e57', delete=False)
            tmp.close()
            e57 = pye57.E57(tmp.name, mode='w')
            e57.write_scan_raw({
                'cartesianX': pts[:, 0].astype(np.float64),
                'cartesianY': pts[:, 1].astype(np.float64),
                'cartesianZ': pts[:, 2].astype(np.float64),
            })
            e57.close()
            with open(tmp.name, 'rb') as f:
                data = f.read()
            os.unlink(tmp.name)
            return app.response_class(
                data, mimetype='application/octet-stream',
                headers={'Content-Disposition': f'attachment; filename="{name}.e57"',
                         'X-Point-Count': str(len(pts))})
        except ImportError:
            fmt = 'xyz'  # fall through to XYZ

    if fmt == 'xyz':
        buf = io.BytesIO()
        np.savetxt(buf, pts, fmt='%.4f', delimiter=' ')
        return app.response_class(
            buf.getvalue(), mimetype='text/plain',
            headers={'Content-Disposition': f'attachment; filename="{name}.xyz"',
                     'X-Point-Count': str(len(pts))})

    if fmt == 'las':
        try:
            import laspy
            header = laspy.LasHeader(point_format=0, version='1.4')
            offset = pts.mean(axis=0).astype(np.float64)
            header.offsets = offset
            header.scales  = np.array([0.001, 0.001, 0.001])
            las = laspy.LasData(header=header)
            las.x = pts[:, 0].astype(np.float64)
            las.y = pts[:, 1].astype(np.float64)
            las.z = pts[:, 2].astype(np.float64)
            buf = io.BytesIO()
            las.write(buf)
            return app.response_class(
                buf.getvalue(), mimetype='application/octet-stream',
                headers={'Content-Disposition': f'attachment; filename="{name}.las"',
                         'X-Point-Count': str(len(pts))})
        except ImportError:
            return jsonify({'ok': False, 'msg': 'laspy not installed on Pi'}), 500

    return jsonify({'ok': False, 'msg': 'Unknown format'}), 400


@app.route('/api/lidar/spindle', methods=['GET', 'POST'])
def lidar_spindle():
    if request.method == 'GET':
        rpm = get_lidar_spindle_rpm()
        return jsonify({'ok': rpm is not None, 'rpm': rpm, 'on': (rpm or 0) > 0})
    body = request.get_json(silent=True) or {}
    on = bool(body.get('on', True))
    rpm = body.get('rpm', None)
    if rpm is not None:
        try:
            rpm = int(rpm)
        except (ValueError, TypeError):
            rpm = None
    ok, msg = set_lidar_spindle(on, rpm)
    return jsonify({'ok': ok, 'msg': msg, 'rpm': _spindle_rpm_cache})


@app.route('/api/motor/cmd', methods=['POST'])
def motor_cmd():
    body = request.get_json(silent=True) or {}
    cmd  = body.get('cmd', '').strip()
    ok, msg = send_motor_cmd(cmd)
    return jsonify({'ok': ok, 'msg': msg})


@app.route('/api/sensors/restart_all', methods=['POST'])
def sensors_restart_all():
    ok, msg = restart_all_sensors()
    return jsonify({'ok': ok, 'msg': msg})


@app.route('/api/shutdown', methods=['POST'])
def shutdown():
    try:
        subprocess.Popen(['sudo', 'shutdown', '-h', 'now'])
        return jsonify({'ok': True})
    except Exception as e:
        return jsonify({'ok': False, 'msg': str(e)})


UPDATE_SCRIPT = os.path.expanduser('~/scripts/update.sh')
UPDATE_LOG    = os.path.expanduser('~/update.log')

@app.route('/api/system/update', methods=['POST'])
def system_update():
    if not os.path.exists(UPDATE_SCRIPT):
        return jsonify({'ok': False, 'msg': 'update.sh not found — run bootstrap first'}), 404
    try:
        with open(UPDATE_LOG, 'a') as log_f:
            subprocess.Popen(['/bin/bash', UPDATE_SCRIPT],
                             stdout=log_f, stderr=subprocess.STDOUT,
                             start_new_session=True)
        return jsonify({'ok': True, 'msg': 'Update started — see log below'})
    except Exception as e:
        return jsonify({'ok': False, 'msg': str(e)})


@app.route('/api/system/update/log')
def system_update_log():
    """SSE stream of ~/update.log tail."""
    def generate():
        try:
            with open(UPDATE_LOG, 'r') as f:
                f.seek(0, 2)  # jump to end
                while True:
                    line = f.readline()
                    if line:
                        yield f'data: {line.rstrip()}\n\n'
                    else:
                        time.sleep(0.3)
                        yield ': ping\n\n'
        except GeneratorExit:
            pass
        except FileNotFoundError:
            yield 'data: (log not found)\n\n'
    return app.response_class(generate(), mimetype='text/event-stream',
                              headers={'Cache-Control': 'no-cache',
                                       'X-Accel-Buffering': 'no'})


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=3000, debug=False, threaded=True)
