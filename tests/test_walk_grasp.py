"""
tests/test_walk_grasp.py
────────────────────────
End-to-end тест на железе: G1 идёт к чашке, хватает её тактильно, возвращается.

Сценарий:
1. Положить тестовую чашку на стол перед G1 (~80 см впереди)
2. Запустить: python tests/test_walk_grasp.py --robot --lowlevel
3. G1 должен:
   - stand_up
   - move_to(0.8, 0, 0)
   - подвести правую руку к чашке (pregrasp → grasp)
   - closed-loop grip через тактильные сенсоры
   - поднять чашку
   - move_to(-0.8, 0, 0)
   - отпустить (open_hand)

⚠️ ПЕРЕД ЗАПУСКОМ:
- Убедиться что вокруг робота нет людей/хрупких предметов
- Пульт L1+RIGHT для fall-protection при необходимости
- E-STOP под рукой
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--mock", action="store_true", help="Симуляция без железа")
    p.add_argument("--robot", action="store_true", help="Реальный G1")
    p.add_argument("--lowlevel", action="store_true",
                   help="Включить low-level arm control")
    p.add_argument("--distance", type=float, default=0.8,
                   help="Дистанция до чашки (м)")
    p.add_argument("--hand", choices=["left", "right"], default="right")
    args = p.parse_args()

    from interfaces.unitree_sdk import UnitreeG1Interface, MockG1Interface
    from perception.tactile.rh56dftp import (
        RH56DFTPDriver, MockRH56DFTPDriver, GripController,
    )
    from control.arm_controller import ArmController
    from control.safety import SafetyMonitor

    if args.robot and not args.mock:
        robot = UnitreeG1Interface(enable_low_level=args.lowlevel)
        tactile = RH56DFTPDriver(hand=args.hand, port="/dev/ttyUSB0")
    else:
        robot = MockG1Interface(enable_low_level=args.lowlevel)
        tactile = MockRH56DFTPDriver(hand=args.hand)

    robot.connect()
    tactile.connect()

    arm = ArmController(robot, side=args.hand)
    arm.enable()

    safety = SafetyMonitor(robot, tactile)
    safety.start()

    print("\n" + "=" * 60)
    print("🧪 WALK + GRASP TEST")
    print("=" * 60)
    input("\nПоставь чашку на стол перед G1. Нажми Enter...")

    try:
        # 1. Stand up
        print("\n[1/7] stand_up...")
        robot.stand_up()
        time.sleep(1.0)

        # 2. Open hand
        print("[2/7] open_hand...")
        tactile.open_hand()
        time.sleep(0.5)

        # 3. Move to pregrasp pose (arm forward)
        print("[3/7] arm → pregrasp...")
        arm.move_to_pose("pregrasp", blocking=True)

        # 4. Walk forward
        print(f"[4/7] walk forward {args.distance}m...")
        robot.move(vx=0.3, vy=0.0, vyaw=0.0,
                  duration_s=args.distance / 0.3)
        time.sleep(0.5)

        # 5. Closed-loop grip
        print("[5/7] grip (closed-loop, max 200 steps)...")
        grip = GripController(
            driver=tactile,
            target_force_n=2.0,
            max_force_n=8.0,
        )
        grip.reset()
        for step in range(200):
            pos, status = grip.grip_step()
            tactile.close_hand(pos)
            if status == "stable":
                print(f"  ✅ Grip stable at step {step}, pos={pos:.2f}")
                break
            if status == "overforce":
                print(f"  ❌ Overforce at step {step} — abort")
                grip.release()
                raise RuntimeError("Overforce during grip")
            time.sleep(0.05)
        else:
            print("  ❌ Timeout — не удалось stabilise grip")
            grip.release()
            raise RuntimeError("Grip timeout")

        # 6. Lift arm
        print("[6/7] arm → lift...")
        arm.move_to_pose("lift", blocking=True)

        # 7. Walk back
        print(f"[7/7] walk backward {args.distance}m...")
        robot.move(vx=-0.3, vy=0.0, vyaw=0.0,
                  duration_s=args.distance / 0.3)

        # Release
        print("\n[release] open_hand...")
        tactile.open_hand()
        arm.move_to_pose("home", blocking=True)

        print("\n✅ WALK + GRASP TEST PASSED")
        print("   (если чашка не упала по дороге — отлично)")

    except KeyboardInterrupt:
        print("\n\n❌ Прервано пользователем")
        robot.emergency_stop()
    except Exception as e:
        print(f"\n❌ FAILED: {e}")
        try:
            robot.emergency_stop()
        except Exception:
            pass
        sys.exit(1)
    finally:
        safety.stop()
        try: tactile.close()
        except: pass
        try: arm.relax()
        except: pass
        try: robot.stand_down()
        except: pass


if __name__ == "__main__":
    main()
