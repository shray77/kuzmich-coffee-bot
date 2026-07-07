# 2 часа на роботе — чек-лист

> У нас 120 минут. Это **критически мало**. Каждый шаг тайм-боксирован.
> Если что-то не работает >5 мин — **пропускай, иди дальше**.

## Pre-flight (до прихода на робота, у себя на ноуте)

```bash
# 1. Клонировать SDK Unitree
git clone https://github.com/unitreerobotics/unitree_sdk2_python
cd unitree_sdk2_python
pip install -e .
pip install cyclonedds pymodbus pyrealsense2 ultralytics openai-whisper httpx

# 2. Скачать YOLOv8m COCO-pretrained
python -c "from ultralytics import YOLO; YOLO('yolov8m.pt')"

# 3. Скачать Whisper-medium
python -c "import whisper; whisper.load_model('medium')"

# 4. Установить Ollama + Gemma3:4B
ollama pull gemma3:4b
```

Время: **0:00** (счётчик запущен).

---

## T+0:00 — T+0:15: Подключение к G1

```bash
# 1. Подключить Ethernet-кабель от ПК к свитчу G1
# 2. Настроить IP ПК: 192.168.123.222/24 (любой в подсети, не конфликтующий)
#    Linux: sudo ip addr add 192.168.123.222/24 dev eth0
#    Windows: Network settings → IPv4 → 192.168.123.222 / 255.255.255.0

# 3. Ping G1
ping 192.168.123.164

# 4. Установить CYCLONEDDS_URI
export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
```

`cyclonedds.xml`:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain Id="any">
    <General>
      <NetworkInterfaceAddress>192.168.123.222</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
```

**Тест подключения:**
```python
python -c "
from unitree_sdk2py.core.channel import ChannelFactory
from unitree_sdk2py.go2.sport.sport_client import SportClient
ChannelFactory.Initialize('eth0')
ChannelFactory.SetLogLevel(2)
sc = SportClient()
sc.SetTimeout(5.0)
sc.Init()
print('Connected! SportClient ready.')
"
```

✅ Успех: `Connected! SportClient ready.`
❌ Провал: проверь ping, firewall, CycloneDDS config.

---

## T+0:15 — T+0:30: RealSense D435 + Intrinsics

```python
# tests/test_realsense.py
import pyrealsense2 as rs
import numpy as np

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
pipeline.start(config)

align = rs.align(rs.stream.color)

# Чтение intrinsics — СОХРАНЯЕМ В ФАЙЛ для дальнейшего использования
profile = pipeline.get_active_profile()
intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
intrinsics = {'fx': intr.fx, 'fy': intr.fy, 'ppx': intr.ppx, 'ppy': intr.ppy, 'coeffs': list(intr.coeffs)}
print('Intrinsics:', intrinsics)

import json
with open('configs/realsense_intrinsics.json', 'w') as f:
    json.dump(intrinsics, f, indent=2)

# Тест depth: навести на стену 1-2 м, проверить
for _ in range(30):
    frames = pipeline.wait_for_frames()
    aligned = align.process(frames)
    depth = aligned.get_depth_frame()
    color = aligned.get_color_frame()
    # центр кадра
    cx, cy = 640, 360
    d = depth.get_distance(cx, cy)
    print(f'Center depth: {d:.3f} m')

pipeline.stop()
```

✅ Успех: intrinsics сохранены, depth читается
❌ Провал: попробовать USB 3.0 порт, обновить firmware D435

---

## T+0:30 — T+0:50: YOLO на железе

```bash
# Тест детекции чашек
python perception/vision/detector.py 0   # 0 = веб-камера, или путь к видео
```

Поднеси **реальную чашку** к камере. Проверь:
- [ ] bbox вокруг чашки
- [ ] confidence > 0.5
- [ ] depth в кадре (если RealSense подключён)

**Если точность плохая (<0.5 conf):**
- Записать 20-30 фото чашки с разных ракурсов
- Дообучить YOLOv8m на 50 эпох (10-15 мин на RTX 4060)
```bash
yolo train model=yolov8m.pt data=cup_dataset.yaml epochs=50 imgsz=640 batch=16
```

✅ Успех: чашка детектится с conf ≥ 0.7
❌ Провал: дообучить или использовать YOLO-World с промптом "coffee cup"

---

## T+0:50 — T+1:10: RH56DFTP тактильные сенсоры

### Калибровка (5 минут на сенсор × 6 = 30 минут, но можно 3 сенсора)

```bash
python perception/tactile/rh56dftp.py --calibrate
```

Сценарий:
1. Робот стоит, рука вытянута ладонью вверх
2. Кладёшь груз 0 г (воздух) → Enter → снимаем 20 сэмплов
3. Груз 50 г → Enter → сэмплы
4. Груз 100 г → Enter → сэмплы
5. Груз 500 г → Enter → сэмплы
6. Груз 1000 г → Enter → сэмплы

⚠️ **Не кладите 2000 г на палец G1** — может повредить сенсор. Максимум 1000 г.

Если калибровка не идёт (Modbus error):
```bash
# Проверь RS-485 подключение
# Убедись, что порт правильный: ls /dev/ttyUSB*
# Попробуй другой baudrate: 9600, 19200, 38400, 57600
```

✅ Успех: `configs/tactile_calibration.json` создан, slope/offset ≠ 0
❌ Провал: использовать **только моторный ток** как proxy для силы (менее точно, но работает)

---

## T+1:10 — T+1:30: Walk + Grasp dry-run

```python
# tests/test_walk_grasp.py — упрощённый сценарий
import time
from interfaces.unitree_sdk import UnitreeG1Interface
from perception.tactile.rh56dftp import RH56DFTPDriver, GripController

robot = UnitreeG1Interface()
robot.connect()
robot.stand_up()
print("G1 стоит")

# Положить тестовую чашку на стол перед роботом
input("Поставь чашку на стол перед G1. Нажми Enter...")

# Идти к чашке 1 метр
robot.move_to(0.8, 0.0, 0.0)
print("G1 подошёл")

# (тут должна быть логика рук — low-level arm control)
# ПОКА БАЙПАСИМ — просто тестируем кисть

# Тест grip
driver = RH56DFTPDriver(hand="right", port="/dev/ttyUSB0",
                        calibration_path="configs/tactile_calibration.json")
driver.connect()
grip = GripController(driver, target_force_n=2.0)

input("Подставь чашку под пальцы G1. Нажми Enter...")
status = "gripping"
while status == "gripping":
    pos, status = grip.grip_step()
    driver.close_hand(grip_strength=pos)
    time.sleep(0.05)
print(f"Grip status: {status}")

# Поднимаем руку (упрощённо — high-level поза)
# TODO: low-level arm positions

# Идём обратно
robot.move_to(-0.8, 0.0, 0.0)
print("G1 вернулся")

# Отпускаем
grip.release()
print("Чашка передана")
```

✅ Успех: G1 подошёл, схватил, вернулся, отпустил
❌ Провал: записать **где** упало, перейти к следующему шагу

---

## T+1:30 — T+1:50: End-to-end demo

```bash
python main.py
>>> принеси кофе
```

Должно произойти:
1. LLM парсит "принеси кофе" → `{"action":"fetch","object":"coffee"}`
2. Behavior tree запускается
3. Walk → Find cup → Approach → Grasp → Walk back → Handover → Confirm

**Записать видео** — даже если упало на середине. Это материал для дебага.

---

## T+1:50 — T+2:00: Дамп и логи

```bash
# Сохранить всё важное
cp configs/tactile_calibration.json backup_calib_$(date +%s).json
cp configs/realsense_intrinsics.json backup_intr_$(date +%s).json

# Сохранить логи
dmesg | tail -50 > logs_dmesg.txt
journalctl -u cyclonedds --since "2 hours ago" > logs_dds.txt

# Закоммитить всё на GitHub
git add -A
git commit -m "Hardware session: calibration + test results"
git push
```

---

## Если что-то сломалось

| Симптом | Решение |
|---|---|
| `ping 192.168.123.164` не идёт | Проверь IP ПК (192.168.123.222/24), кабель, свитч |
| DDS timeout | `export CYCLONEDDS_URI=...`, отключи firewall |
| G1 не двигается | Проверь что **не в fall-protection** (пульт L1+RIGHT) |
| RealSense не находится | USB 3.0 порт, `rs-enumerate-devices` |
| YOLO медленная | Уменьши разрешение до 640×480, или yolov8s вместо m |
| Modbus RH56DFTP таймаут | Проверь /dev/ttyUSB*, baudrate, slave_address |
| Рука не двигается | Low-level команды нужно разрешить в настройках G1 |
| G1 падает | E-STOP на пульте, проверь Kp/Kd |

## Что НЕ делать

- ❌ Не меняй firmware G1 — нет времени на откат
- ❌ Не калибруй все 6 сенсоров — хватит 3 (thumb, index, middle)
- ❌ Не дообучай YOLO больше 50 эпох — рискуешь не успеть
- ❌ Не запускай RL LOCO policy — нет весов, рискуем уронить робота
- ❌ Не используй low-level arm control без Kp=0,Kd=5 (damping) первым шагом

## Минимум для успеха

Даже если ничего не получилось с рукой, главное:
1. ✅ Робот стоит и не падает (T+0:15)
2. ✅ Видит чашку через YOLO (T+0:50)
3. ✅ Идёт к чашке и обратно (T+1:30)
4. ✅ Сняли видео демки (T+1:50)

Этого достаточно, чтобы показать "робот двигается к чашке по голосовой команде".
