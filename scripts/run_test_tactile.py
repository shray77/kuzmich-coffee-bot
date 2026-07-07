"""
scripts/run_test_tactile.py
───────────────────────────
Быстрый тест тактильных сенсоров RH56DFTP — чтение показаний 10 сек.

Запуск:
    python scripts/run_test_tactile.py --port /dev/ttyUSB0
    python scripts/run_test_tactile.py --mock
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from perception.tactile.rh56dftp import RH56DFTPDriver, MockRH56DFTPDriver


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--hand", choices=["left", "right"], default="right")
    p.add_argument("--port", default="/dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--mock", action="store_true")
    p.add_argument("--duration", type=float, default=10.0)
    args = p.parse_args()

    if args.mock:
        driver = MockRH56DFTPDriver(hand=args.hand)
    else:
        driver = RH56DFTPDriver(hand=args.hand, port=args.port, baudrate=args.baud)

    try:
        driver.connect()
    except Exception as e:
        print(f"❌ Подключение не удалось: {e}")
        sys.exit(1)

    print(f"\n=== Тест RH56DFTP {args.hand} ({args.duration} сек) ===\n")
    print("Потыкай пальцами в сенсоры — должна расти сила.\n")

    t0 = time.time()
    samples = 0
    max_force = 0.0
    slip_count = 0
    try:
        while time.time() - t0 < args.duration:
            r = driver.read()
            samples += 1
            max_force = max(max_force, r.total_force_n)
            if r.slip_detected:
                slip_count += 1
            # Каждые 0.5 сек — вывод
            if samples % 5 == 0:
                forces_str = " ".join(f"{f:5.2f}" for f in r.forces_n)
                print(f"  t={time.time()-t0:5.1f}s | {forces_str} | total={r.total_force_n:5.2f}N "
                      f"contact={'Y' if r.contact_detected else 'N'} slip={'Y' if r.slip_detected else 'N'}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        driver.close()

    print(f"\n=== Сводка ===")
    print(f"  Сэмплов: {samples}")
    print(f"  Max force: {max_force:.2f} N")
    print(f"  Slip events: {slip_count}")
    if max_force > 0.5:
        print("  ✅ Сенсоры отвечают")
    else:
        print("  ⚠️  Сенсоры молчат — проверь подключение")


if __name__ == "__main__":
    main()
