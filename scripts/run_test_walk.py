"""
scripts/run_test_walk.py
────────────────────────
Тест ходьбы G1: stand_up → move_to(1m,0,0) → move_to(-1m,0,0) → stand_down.

Запуск:
    python scripts/run_test_walk.py --mock        # симуляция
    python scripts/run_test_walk.py --robot       # реальный G1
    python scripts/run_test_walk.py --robot --distance 2.0
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from interfaces.unitree_sdk import UnitreeG1Interface, MockG1Interface


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--mock", action="store_true")
    p.add_argument("--robot", action="store_true")
    p.add_argument("--distance", type=float, default=1.0,
                   help="Дистанция вперёд/назад (м)")
    p.add_argument("--side", type=float, default=0.0,
                   help="Боковое смещение (м)")
    p.add_argument("--speed", type=float, default=0.5,
                   help="Скорость (м/с), задаёт duration = distance/speed")
    args = p.parse_args()

    if args.robot and not args.mock:
        robot = UnitreeG1Interface()
        print("[walk] Подключение к реальному G1...")
    else:
        robot = MockG1Interface()
        print("[walk] Mock-режим")

    robot.connect()
    duration = max(2.0, args.distance / max(0.1, args.speed))

    try:
        print(f"\n[walk] 1. stand_up")
        robot.stand_up()
        time.sleep(1.0)

        print(f"[walk] 2. move forward {args.distance}m ({duration:.1f}s)")
        robot.move(vx=args.speed, vy=0.0, vyaw=0.0, duration_s=duration)

        time.sleep(0.5)

        print(f"[walk] 3. move backward {args.distance}m")
        robot.move(vx=-args.speed, vy=0.0, vyaw=0.0, duration_s=duration)

        if args.side != 0.0:
            side_dur = max(2.0, abs(args.side) / args.speed)
            print(f"[walk] 4. move side {args.side}m")
            robot.move(vx=0.0, vy=args.side / side_dur, vyaw=0.0, duration_s=side_dur)

        print(f"[walk] 5. stop_move")
        robot.stop_move()

        print(f"[walk] 6. stand_down")
        robot.stand_down()
        print("\n✅ Walk test OK")

    except KeyboardInterrupt:
        print("\n[walk] Прервано, e-stop")
        robot.emergency_stop()
    except Exception as e:
        print(f"\n❌ Walk test failed: {e}")
        try:
            robot.emergency_stop()
        except Exception:
            pass
        sys.exit(1)


if __name__ == "__main__":
    main()
