# G1 Cup Approach Solution — восстановительный пакет

> Отдельная архитектура от остального репо: тут ROS 2 Foxy (nodes/topics/services),
> а не прямой Python-SDK как в `interfaces/unitree_sdk.py` и `coffee_delivery.py`
> (в `burunov-joke-bot`). Не смешивать — деплоится и живёт отдельно, через
> `deploy_to_home.sh` этой папки. `g1_manip_node.py` и `cup_fetch_orchestrator.py`
> в `scripts/` — добавлены поверх присланного пакета (захват чашки + связка с
> approach-контуром), остальное — как прислали.

Архитектура:

```text
RealSense + YOLO → /perception/cup_pose_map
→ cup_center_then_adaptive_node.py
→ /cmd_vel_raw
→ ros_cmd_vel_udp_exporter.py
→ UDP 127.0.0.1:15000
→ g1_sdk_udp_receiver_fsm801.py
→ Unitree SDK2 / FSM801
→ Unitree G1
```

## Развертывание на роботе

```bash
cd ~
unzip g1_cup_solution_package.zip -d ~/
cd ~/g1_cup_solution_package
./deploy_to_home.sh
~/check_g1_solution_deps.sh
```

Если есть `MISSING` по Python/ROS зависимостям:

```bash
~/install_g1_solution_deps.sh
~/check_g1_solution_deps.sh
```

Модель YOLO должна быть здесь:

```bash
~/g1_robot_delivery_solution_foxy/models/yolo26n.pt
```

## Холодный старт текущего этапа center + adaptive

```bash
~/cold_cleanup_g1_cup.sh
```

Открыть терминалы:

1. `~/start_realsense_g1.sh`
2. `~/start_tf_g1.sh`
3. `~/start_yolo_cup_standalone.sh`
4. `~/g1_raw_udp_exporter_stable.sh`
5. `~/g1_receiver_fsm801_stable.sh`
6. `~/cup_center_then_adaptive_stable.sh`
7. Проверка: `~/preflight_g1_cup.sh`
8. Старт: `~/cup_approach_start.sh`
9. Стоп: `~/cup_approach_stop.sh`

## Критерий успеха

- `/perception/cup_pose_map` публикуется.
- `/cmd_vel_raw`: publisher `cup_center_then_adaptive_node`, subscriber `ros_cmd_vel_udp_exporter`.
- `ss -lunp | grep 15000` показывает python receiver.
- В receiver есть `GetFsmId after: (0, 801)`.
- В движении видны `UDP recv vx=0.300` или `UDP recv wz=...` и `SDK Move sent ...`.

## Захват чашки (закрывает пункт "заменить fake /g1_manip")

Новое, не было в исходном пакете: `g1_manip_node.py` — реальный захват с
контролем силы через `HandClient` (Inspire RH56DFTP). Если рука на самом
деле другая (Dex3/BrainCo) — увидишь явную ошибку в логе, а не тишину.

Терминал 8 (после того как approach-контур из шагов 1-7 уже поднят):

```bash
~/g1_manip_node.py --ros-args -p iface:=eth0 -p hand_used:=right \
  -p target_force_g:=80.0 -p max_force_g:=1200.0
```

Проверка вручную:

```bash
ros2 service call /g1_manip/grasp_cup std_srvs/srv/Trigger {}
ros2 service call /g1_manip/release_cup std_srvs/srv/Trigger {}
```

Полный цикл "доехать + взять" одной командой (дёргает существующий
`/cup_approach_x/start`, ждёт пока `/perception/cup_pose_map` покажет
`x <= stop_x_m`, потом `/g1_manip/grasp_cup`):

```bash
~/cup_fetch_orchestrator.py --ros-args -p stop_x_m:=1.25 -p timeout_s:=40.0
```
