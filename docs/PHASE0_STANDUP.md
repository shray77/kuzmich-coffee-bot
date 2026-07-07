# Phase 0 — Стендовые тесты (1 неделя)

Цель: подтвердить, что всё железо отвечает, снять baseline-метрики.

## День 1: Unitree SDK + telemetry

```bash
pip install unitree_sdk2py
```

Проверить:
- [ ] Подключение по WiFi к G1 (IP 192.168.123.161 по дефолту)
- [ ] Чтение IMU (частота 500 Гц)
- [ ] Чтение joint states (43 DoF, частота 500 Гц)
- [ ] Команда "стоять" (stand_cmd) — робот держит баланс
- [ ] Команда "сесть" — корректное выполнение

См. `tests/test_sdk_connection.py`

## День 2: RealSense D435

```bash
pip install pyrealsense2
```

Проверить:
- [ ] RGB stream 1280×720 @ 30 fps
- [ ] Depth stream 1280×720 @ 30 fps, range 0.3-3 м
- [ ] Aligned depth-to-color
- [ ] Point cloud generation

См. `tests/test_realsense.py`

## День 3: RH56DFTP тактильные сенсоры

Подключение через RS-485 (протокол Robot Heart, см. docs к кисти).

Калибровка:
- [ ] Снять 0 г (воздух) — baseline
- [ ] 50 г — палец 1
- [ ] 100 г — палец 1
- [ ] 500 г — палец 1
- [ ] 1000 г — палец 1
- [ ] 2000 г — палец 1
- [ ] Повторить для пальцев 2-5
- [ ] Повторить для второй кисти
- [ ] Сохранить калибровочную таблицу в `configs/tactile_calibration.json`

См. `tests/test_tactile.py`

## День 4: Whisper STT

```bash
pip install openai-whisper
```

Тесты:
- [ ] Whisper-tiny на CPU — распознаёт "принеси кофе"?
- [ ] Whisper-medium на CPU — точность, задержка
- [ ] Whisper-medium на GPU (если есть) — latency < 1 сек?

См. `tests/test_whisper.py`

## День 5: LLM parser

```bash
ollama pull gemma3:4b
```

Промпт-инжиниринг:
- [ ] "Кузьмич, принеси кофе" → `{"action": "fetch", "object": "coffee", "target": "self"}`
- [ ] "Принеси Олеже кофе" → `{"action": "fetch", "object": "coffee", "target": "Oleg"}`
- [ ] "Хочу чай" → `{"action": "fetch", "object": "tea", "target": "self"}`
- [ ] "Стоп" → `{"action": "abort"}`
- [ ] "Вернись на базу" → `{"action": "return_home"}`

См. `perception/voice/parser.py`

## День 6-7: Интеграция

- [ ] Голос → STT → LLM → JSON goal
- [ ] Vision: YOLOv8m детектит чашку в кадре
- [ ] RealSense depth → 3D координаты чашки относительно базы
- [ ] Tactile: считать показания при нажатии на палец

Метрики Phase 0:
- STT accuracy на 20 фразах: target ≥ 95%
- YOLO mAP на 50 фото чашек: target ≥ 0.7
- Tactile SNR: target ≥ 20 дБ
- End-to-end latency (голос → goal JSON): target ≤ 2 сек
