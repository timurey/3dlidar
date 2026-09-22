# 3D LIDAR SLAM Scanner

Самодельный 3D SLAM-сканер: Velodyne VLP-16 на вращающейся платформе с RP2040-энкодером.  
Orange Pi 5 Plus — ROS2 Jazzy, Flask HMI, запись bag-файлов.  
Mac — постобработка: deskew, ICP, экспорт в CAD.

---

## Железо

```
┌─────────────────────────────────────────────────────────────┐
│                    Orange Pi 5 Plus                          │
│            Ubuntu 24.04 (Noble) ARM64                        │
│               ROS 2 Jazzy (ros-base)                         │
│                                                              │
│  Ethernet ←──── Velodyne VLP-16 (UDP :2368)                  │
│  UART0    ←──── RP2040 (230400 baud, 100 Hz)                 │
│  UART1    ←──── CYD ESP32 (115200 baud)                      │
│  USB      ←──── IMU (Yahboom 9-axis)                         │
└─────────────────────────────────────────────────────────────┘
         │                    │
         ▼                    ▼
  [spin_controller]    [cyd_hmi]
  RP2040 Zero          CYD ESP32-2432S028R
  + SimpleFOC Mini     240×320 TFT touchscreen
  + 2840 BLDC
  + AS5600 encoder
  + 5:1 редуктор
  12V питание
```

| Компонент           | Модель                   | Роль                                    |
|---------------------|--------------------------|-----------------------------------------|
| Основной ПК         | Orange Pi 5 Plus         | ROS2, запись bag, Flask HMI             |
| Лидар               | Velodyne VLP-16          | Основной сенсор, UDP :2368              |
| Контроллер вращения | RP2040 Zero (Waveshare)  | FOC closed-loop PID, encoder → UART 100 Hz |
| Драйвер мотора      | SimpleFOC Mini           | 3-фазный PWM                            |
| Мотор               | 2840 BLDC, 7 пар полюсов | Вращение платформы                      |
| Редуктор            | 5:1                      | Мотор 200 RPM → платформа 40 RPM        |
| Датчик угла         | AS5600 (I2C)             | Магнитный энкодер, абсолютный угол      |
| HMI                 | CYD ESP32-2432S028R      | Touchscreen панель управления           |
| IMU                 | Yahboom 9-axis IMU       | 9DOF (используется в 6-axis режиме — без магнетометра, помехи от мотора); пакет `imu_ros2_device` |
| Аккумулятор         | 1P10S (42 В)             | От гироскутера                          |

---

## Структура монорепо

```
3dlidar/
├── orangepi/               # Всё, что живёт на Orange Pi
│   ├── hmi/                # Flask HMI (порт 3000)
│   │   ├── app.py
│   │   ├── bag_preview.py
│   │   └── templates/
│   ├── ros2/               # ROS2 пакеты
│   │   ├── slam_bringup/
│   │   ├── spin_controller/
│   │   ├── hmi_bridge/
│   │   ├── hmi_manager/
│   │   └── encoder_bridge/
│   ├── scripts/
│   │   └── update.sh       # OTA: git pull + rebuild + restart
│   └── system/             # systemd, netplan, udev, sysctl
│
├── desktop/                # Mac-side инструменты
│   ├── offline_deskew.py   # symlink → ../shared/
│   ├── world_map.py
│   ├── flythrough.py       # OpenGL вьювер облаков
│   ├── icp_merge.py
│   ├── export_cad.py       # → E57 для CAD
│   └── requirements.txt
│
├── firmware/
│   ├── spin_controller/    # RP2040 + SimpleFOC (PlatformIO)
│   └── cyd_hmi/            # CYD ESP32 TFT HMI (PlatformIO)
│
├── shared/
│   └── offline_deskew.py   # Единый источник: деплоится на Pi и symlink на Mac
│
├── scripts/
│   ├── deploy.sh           # Mac → Pi: rsync + colcon + restart
│   ├── bootstrap.sh        # Чистая Pi с нуля
│   └── sync_bags.sh        # Pi bags/ → local bags/
│
├── bags/                   # Локальные записи (gitignored)
├── Makefile
└── README.md
```

---

## Быстрый старт

```bash
# Деплой на Pi
make deploy           # hmi + ros2
make deploy-hmi       # только HMI
make deploy-ros2      # только ROS2 + colcon build

# OTA с Pi (Pi сама тянет git pull)
make ota              # через SSH
# или кнопка ↑ Update в веб-HMI

# Синхронизация записей
make sync-bags

# Чистая установка Pi с нуля
make bootstrap

# Прошивка железа
make flash-rp2040
make flash-cyd
```

---

## Деплой вручную

```bash
PI=openclaw@192.168.1.108

# HMI
rsync -av orangepi/hmi/ $PI:~/hmi/
rsync -av shared/offline_deskew.py $PI:~/hmi/
ssh $PI "sudo systemctl restart hmi"

# ROS2
rsync -av orangepi/ros2/ $PI:~/ros2_ws/src/
ssh $PI "source /opt/ros/jazzy/setup.bash && cd ~/ros2_ws && colcon build"
ssh $PI "sudo systemctl restart hmi_bridge slam-scanner"
```

---

## Протоколы

### RP2040 → Orange Pi (UART, 230400, 100 Hz)

Телеметрия (100 Hz):
```
$T,<millis>,<angle_deg_unwrapped>,<rpm>*<XOR_checksum>\n
```
Пример: `$T,12345,184.23,39.87*3F`

Команды Orange Pi → RP2040:
```
$START*CS        — запустить мотор (initFOC + PID)
$STOP*CS         — остановить
$STATUS*CS       — ответ: $S,<state>,<rpm>,<target_rpm>,<enabled>,<fault>,<uptime>*CS
$DIAG*CS         — ответ: $D,<sensor_ok>,<wire_err>,<bus_ok>*CS
$SETRPM,<rpm>*CS — установить целевые RPM платформы
```

Состояния: `INIT` → `SELFTEST` → `IDLE` → `RUNNING` / `FAULT`  
Коды fault: `1=SENSOR_DEAD`, `2=ROTATE_FAILED`

> Мотор молчит до явной команды `$START` — `initFOC()` не вызывается автоматически.

### CYD ESP32 ↔ Orange Pi (UART, 115200)

Пины UART: **GPIO 22 (RX) / GPIO 27 (TX)** через CN1-коннектор.  
GPIO 1/3 (P5) нельзя — CH340C держит GPIO3 HIGH даже без USB.

Статус Orange Pi → CYD:
```json
{
  "sensors_running": true,
  "lidar_hz": 10.0,
  "imu_hz": 100.0,
  "lidar_ok": true,
  "imu_ok": true,
  "recording": false,
  "disk_gb": 123.4,
  "rec_duration": 60,
  "bag_name": "rosbag2_2026_..."
}
```
Команды CYD → Orange Pi:
```json
{"cmd": "start_recording"}
{"cmd": "stop_recording"}
{"cmd": "shutdown"}          ← длинное нажатие ≥2 с
```

> XPT2046 координаты не работают (SPI-конфликт с TFT_eSPI) — используется только IRQ на GPIO 36.

---

## ROS2 топики

| Топик                          | Тип                       | Источник           |
|--------------------------------|---------------------------|--------------------|
| `/velodyne_points`             | `sensor_msgs/PointCloud2` | VLP-16 driver      |
| `/rotating_platform/angle`     | `std_msgs/Float64`        | spin_controller    |
| `/rotating_platform/joint_state` | `sensor_msgs/JointState` | spin_controller    |
| `/imu`                         | `sensor_msgs/Imu`         | Yahboom 9-axis IMU (`imu_ros2_device`) |
| `/imu/mag`                     | `sensor_msgs/MagneticField` | Yahboom 9-axis IMU (магнетометр) |

### TF дерево

```
world → imu_link → platform_base → platform_rotating → velodyne
```

Монтаж лидара (`slam_scanner.launch.py`):
```
--roll 0 --pitch 0 --yaw -0.02793  (≈ -1.6°, калибровка 2026-09-21)
```

---

## Пайплайн постобработки (Mac)

```
MCAP bag
   │
   ▼
shared/offline_deskew.py   ← /velodyne_points + /rotating_platform/angle
   │   MOUNT_RPY_DEG = [0.0, 0.0, -1.6]   (yaw-калибровка)
   ▼
desktop/icp_merge.py       ← угловые срезы + ICP регистрация
   ▼
desktop/flythrough.py      ← OpenGL вьювер (сравнение, выравнивание)
   ▼
desktop/export_cad.py      ← E57 для CAD
```

### Параметры калибровки (`shared/offline_deskew.py`)

| Параметр             | Значение    | Описание                            |
|----------------------|-------------|-------------------------------------|
| `ROTATION_AXIS`      | `'x'`       | Ось вращения платформы              |
| `ANGLE_OFFSET_DEG`   | `184.0`     | Калибровочное смещение нуля         |
| `MOUNT_RPY_DEG`      | `[0,0,-1.6]`| Крен/тангаж/рыск монтажа лидара     |
| `ROTATION_CENTER`    | `[0,0,0]`   | Смещение оптического центра лидара  |

---

## HMI (Flask, порт 3000)

Адрес: `http://192.168.1.108:3000`

Основные API:

| Endpoint                    | Метод | Назначение                     |
|-----------------------------|-------|--------------------------------|
| `/api/status`               | GET   | Статус сенсоров                |
| `/api/record/start`         | POST  | Начать запись bag              |
| `/api/record/stop`          | POST  | Остановить запись              |
| `/api/lidar/spindle`        | POST  | Включить/выключить, RPM 300/600|
| `/api/preview`              | GET   | XYZ облако для WebGL           |
| `/api/system/update`        | POST  | Запустить OTA update.sh        |
| `/api/system/update/log`    | GET   | SSE лог обновления             |
| `/api/shutdown`             | POST  | Выключить Pi                   |

---

## Окружение Orange Pi

- OS: Ubuntu 24.04 (Noble) ARM64
- ROS: ROS 2 Jazzy
- HMI: `~/hmi/` → `systemctl status hmi`
- Bags: `~/bags/`
- ROS2 ws: `~/ros2_ws/`
- OTA скрипт: `~/scripts/update.sh`
- IP: `192.168.1.108`, пользователь `openclaw`

---

## Статус (2026-09-22)

- ✅ VLP-16 на изолированной подсети `192.168.100.x`
- ✅ RP2040 spin_controller: FOC, энкодер, UART 100 Hz
- ✅ ROS2 нода парсинга UART → `/rotating_platform/angle`
- ✅ Flask HMI: запись, мониторинг, WebGL-просмотр bag
- ✅ Deskew pipeline: компенсация motion blur по углу энкодера
- ✅ Yaw-калибровка: −1.6° (вращение не строго вертикальное)
- ✅ Монорепо: orangepi/ desktop/ firmware/ shared/
- ✅ deploy.sh, bootstrap.sh, OTA update.sh
- ⏸ Биение оси ~1° — требует механической диагностики
- ⏸ CYD touchscreen координаты — XPT2046 SPI конфликт, workaround: только IRQ
