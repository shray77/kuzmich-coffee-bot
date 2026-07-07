# Кузьмич — Coffee Bot

Unitree G1 EDU Ultimate humanoid robot, приносящий кофе по голосовой команде.

## Железо

| Компонент | Характеристика |
|---|---|
| Платформа | Unitree G1 EDU Ultimate |
| DoF | 43 (ноги 6×2, руки 5×2, кисти 7×2, торс 1+2) |
| Бортовой компьютер | 8-ядерный CPU, опц. NVIDIA Jetson Orin |
| 3D камера | Intel RealSense D435 |
| LiDAR | Livox MID-360 |
| Кисти | RH56DFTP × 2 (6 DoF, тактильные сенсоры 10-2500 г) |
| Нагрузка на руку | до 3 кг |
| Связь | WiFi 6, Bluetooth 5.2, USB 3.0/3.2, 2× RJ45 |
| Аккумулятор | 9000 мА·ч, ~2 ч работы |
| Рост / вес | 1270 мм / 37 кг |

## Архитектура (модульная)

```
                ┌─────────────────────────────────────────┐
                │              Голос Олежа                │
                │   "Кузьмич, принеси кофе, будь другом"  │
                └────────────────┬────────────────────────┘
                                 │
                                 ▼
            ┌────────────────────────────────────────────┐
            │           perception/voice/                │
            │   Whisper (STT)  →  LLM parser             │
            │   "GOAL: fetch coffee, target=Oleg"        │
            └────────────────┬───────────────────────────┘
                             │
                             ▼
            ┌────────────────────────────────────────────┐
            │              planning/                     │
            │   Behavior tree / FSM                      │
            │   1. localize kitchen area                 │
            │   2. walk there (LOCO/MPC)                 │
            │   3. find cup (YOLO + RealSense depth)     │
            │   4. approach + grasp (tactile feedback)   │
            │   5. walk back to Oleg                     │
            │   6. hand over (force-torque check)        │
            └────────────────┬───────────────────────────┘
                             │
              ┌──────────────┼──────────────┐
              ▼              ▼              ▼
        ┌──────────┐   ┌──────────┐   ┌──────────┐
        │ vision/  │   │ tactile/ │   │ action/  │
        │ YOLOv8   │   │ RH56DFTP │   │ unitree  │
        │ RealSense│   │  force-  │   │  SDK     │
        │   depth  │   │  feedback│   │          │
        └──────────┘   └──────────┘   └──────────┘
                             │
                             ▼
            ┌────────────────────────────────────────────┐
            │              control/                      │
            │   Whole-body control (RL policy + safety)  │
            │   Joint impedance / MPC / RL LOCO          │
            └────────────────────────────────────────────┘
```

## Стек (план)

| Модуль | Технология | Приоритет |
|---|---|---|
| **STT** | OpenAI Whisper (medium) на Jetson/CPU | P0 |
| **LLM parser** | Llama 3.2 3B или Gemma 3 4B через Ollama | P0 |
| **Vision** | Ultralytics YOLOv8m + COCO 'cup'/'bottle' | P0 |
| **Depth** | Intel RealSense D435 + pyrealsense2 | P0 |
| **Grasp detection** | AnyGrasp или GraspNet-1B (если есть GPU) | P1 |
| **Tactile** | RH56DFTP SDK + custom force-feedback loop | P0 |
| **Locomotion** | Unitree RL LOCO policy (предобученная) | P0 |
| **Manipulation** | Unitree G1 manipulation SDK | P0 |
| **Behavior** | py_trees (behavior trees) | P0 |
| **Safety** | Force-torque monitor, e-stop, geofencing | P0 |

## Фазы разработки

### Phase 0 — Стенд (1 неделя)
- [ ] Настроить SDK Unitree G1, получить telemetry
- [ ] Подключить RealSense D435, проверить depth stream
- [ ] Снять показания RH56DFTP при хвате разных объектов
- [ ] Whisper на Jetson — распознаёт "принеси кофе"

### Phase 1 — Vision (1 неделя)
- [ ] Тест YOLOv8m COCO-pretrained на 'cup' класс
- [ ] Если точность <80% — собрать датасет 500-1000 фото кружек, дообучить
- [ ] Связать YOLO bbox с RealSense depth → 3D координаты чашки

### Phase 2 — Tactile (1 неделя)
- [ ] Снять калибровку сенсоров RH56DFTP (5 грузов 50-2000 г)
- [ ] Реализовать closed-loop grip: сжать пока force > threshold
- [ ] Прогнать 50 попыток хватать чашку — записать метрики

### Phase 3 — Walk + Reach (2 недели)
- [ ] Навигация через ROS2 + Nav2, карта через Livox MID-360
- [ ] Подойти к столу, не уронить чашку при ходьбе
- [ ] Вернуться к Олежу, не расплескать

### Phase 4 — Hand-over (1 неделя)
- [ ] Обнаружить руку Олежи (детекция человека)
- [ ] Передача: ослабить grip когда внешняя сила > X
- [ ] Голосовая фраза "держи, Олежа"

### Phase 5 — Demo ready (1 неделя)
- [ ] End-to-end: "Кузьмич, кофе" → через 60 сек Олежа пьёт
- [ ] Видеозапись, тик-ток, хайп

## Структура репозитория

```
kuzmich-coffee-bot/
├── README.md
├── perception/
│   ├── vision/          # YOLOv8, RealSense depth
│   ├── tactile/         # RH56DFTP force feedback
│   └── voice/           # Whisper + LLM parser
├── planning/            # behavior tree, FSM
├── action/              # high-level: Walk, Grasp, HandOver
├── control/             # low-level: joint control, safety
├── interfaces/          # Unitree SDK wrapper, ROS2 topics
├── configs/             # thresholds, models paths
├── tests/               # unit + integration
└── docs/                # setup, calibration, demo videos
```

## Что нужно решить в первую очередь

1. **Какой SDK у G1?** Unitree SDK2 Python (`unitree_sdk2py`) — проверяем доступность на Jetson
2. **Тактильные сенсоры RH56DFTP** — какой протокол? RS-485? Modbus? Нужно documentation
3. **Jetson Orin или внешний GPU?** Whisper-medium + YOLOv8m одновременно — потянет Orin Nano/NX?
4. **LLM локально или облако?** Если локально — 4B модель ужимается до 3 ГБ VRAM (Q4_K_M)
5. **Безопасность при ходьбе с чашкой** — MPC gait + tilt-compensation для руки

## Лицензия

MIT (когда будет код). Пока что это наброски.
