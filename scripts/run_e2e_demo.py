"""
scripts/run_e2e_demo.py
───────────────────────
End-to-end демо "принеси кофе" без голоса — goal подставляется автоматически.

Используется для быстрого теста на роботе: положи чашку перед G1,
запусти скрипт — он попытается пройти все шаги behavior tree.

Запуск:
    python scripts/run_e2e_demo.py --mock
    python scripts/run_e2e_demo.py --robot
    python scripts/run_e2e_demo.py --robot --lowlevel   # с управлением руками
    python scripts/run_e2e_demo.py --robot --goal 'fetch coffee for Oleg'
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--mock", action="store_true")
    p.add_argument("--robot", action="store_true")
    p.add_argument("--lowlevel", action="store_true")
    p.add_argument("--hand", choices=["left", "right"], default="right")
    p.add_argument("--goal", default="fetch coffee for Oleg",
                   help="Goal string (передаётся в behavior tree как raw)")
    args = p.parse_args()

    from interfaces.unitree_sdk import UnitreeG1Interface, MockG1Interface
    from perception.tactile.rh56dftp import (
        RH56DFTPDriver, MockRH56DFTPDriver, GripController,
    )
    from control.arm_controller import ArmController
    from control.safety import SafetyMonitor
    from action.handover import HandoverController
    from planning.behavior_tree import build_coffee_tree, Blackboard

    # 1. Robot
    if args.robot and not args.mock:
        robot = UnitreeG1Interface(enable_low_level=args.lowlevel)
        print("[e2e] Подключение к G1...")
    else:
        robot = MockG1Interface(enable_low_level=args.lowlevel)
        print("[e2e] Mock-режим")
    robot.connect()
    robot.stand_up()

    # 2. Tactile
    if args.mock or not args.robot:
        tactile = MockRH56DFTPDriver(hand=args.hand)
    else:
        tactile = RH56DFTPDriver(hand=args.hand, port="/dev/ttyUSB0")
    tactile.connect()

    # 3. Arm
    arm = ArmController(robot, side=args.hand)
    arm.enable()

    # 4. Handover
    handover = HandoverController(
        robot=robot,
        tactile_driver=tactile,
        arm_ctrl=arm,
        timeout_s=10.0,
    )

    # 5. Detector (опционально)
    detector = None
    try:
        from perception.vision.detector import CupDetector
        import torch
        device = "cuda:0" if torch.cuda.is_available() else "cpu"
        detector = CupDetector(model_path="yolov8m.pt", device=device)
        print(f"[e2e] CupDetector loaded on {device}")
    except Exception as e:
        print(f"[e2e] CupDetector недоступен: {e}")

    # 6. Safety
    safety = SafetyMonitor(robot, tactile)
    safety.start()

    # 7. Tree
    tree = build_coffee_tree(
        robot=robot,
        detector=detector,
        realsense=None,
        tactile_driver=tactile,
        arm_ctrl=arm,
        handover_controller=handover,
        grip_controller_cls=GripController,
    )

    print("\n" + "=" * 60)
    print("🚀 E2E DEMO: Кузьмич принеси кофе")
    print("=" * 60)
    print(f"Goal: {args.goal}")
    print(f"Hand: {args.hand}, LowLevel: {args.lowlevel}")
    print("=" * 60 + "\n")

    bb = Blackboard(goal={
        "action": "fetch",
        "object": "coffee",
        "target": "Oleg",
        "raw": args.goal,
    })

    t0 = time.time()
    try:
        status = tree.tick(bb)
    except KeyboardInterrupt:
        print("\n[e2e] Прервано, E-STOP")
        robot.emergency_stop()
        safety.emergency_stop()
        sys.exit(1)
    finally:
        safety.stop()
        try: tactile.close()
        except: pass
        try: arm.relax()
        except: pass

    dt = time.time() - t0
    print(f"\n{'='*60}")
    print(f"РЕЗУЛЬТАТ: {status.value} за {dt:.1f}с")
    print(f"Ошибок: {bb.error_count}")
    if bb.last_error:
        print(f"Последняя ошибка: {bb.last_error}")
    print(f"Чашка найдена: {bb.cup_detected}")
    print(f"Чашка схвачена: {bb.cup_grasped}")
    print(f"Передача: {bb.handover_complete}")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
