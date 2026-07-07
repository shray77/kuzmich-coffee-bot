"""
scripts/run_test_arm.py
───────────────────────
Тест рук G1 — проходит все позы из control/arm_poses.py.

Запуск:
    python scripts/run_test_arm.py --mock          # без железа
    python scripts/run_test_arm.py --robot --lowlevel   # реальный G1
    python scripts/run_test_arm.py --mock --hand left
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from control.arm_poses import list_poses, get_pose, get_pose_duration
from control.arm_controller import ArmController, DualArmController, TorsoController


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--mock", action="store_true")
    p.add_argument("--robot", action="store_true")
    p.add_argument("--lowlevel", action="store_true")
    p.add_argument("--hand", choices=["left", "right", "both"], default="right")
    p.add_argument("--duration", type=float, default=None,
                   help="Override длительности каждой позы (сек)")
    p.add_argument("--poses", nargs="*",
                   default=["home", "idle", "pregrasp", "grasp", "lift",
                            "carry", "handover", "release", "home"],
                   help="Список поз для прохода")
    args = p.parse_args()

    from interfaces.unitree_sdk import UnitreeG1Interface, MockG1Interface

    if args.robot and not args.mock:
        robot = UnitreeG1Interface(enable_low_level=args.lowlevel)
    else:
        robot = MockG1Interface(enable_low_level=args.lowlevel)
    robot.connect()
    robot.stand_up()

    print(f"\n[arm] Тест поз: {args.poses}")
    print(f"[arm] Hand: {args.hand}, LowLevel: {args.lowlevel}\n")

    if args.hand == "both":
        ctrl = DualArmController(robot)
        ctrl.enable()
        for pose in args.poses:
            print(f"\n→ {pose} (both)")
            ctrl.move_both_to_pose(pose, duration=args.duration)
        ctrl.relax()
    else:
        arm = ArmController(robot, side=args.hand)
        arm.enable()
        torso = TorsoController(robot)
        for pose in args.poses:
            print(f"\n→ {pose} ({args.hand})")
            arm.move_to_pose(pose, duration=args.duration)
            try:
                torso.move_to_pose(pose, duration=args.duration or 1.0)
            except Exception as e:
                print(f"  torso skipped: {e}")
        arm.relax()

    print("\n✅ Arm test OK")


if __name__ == "__main__":
    main()
