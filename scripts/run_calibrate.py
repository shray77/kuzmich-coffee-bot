"""
scripts/run_calibrate.py
────────────────────────
Калибровка тактильных сенсоров RH56DFTP.

Запуск:
    python scripts/run_calibrate.py --hand right --port /dev/ttyUSB0
    python scripts/run_calibrate.py --mock   # симуляция

Нужны грузы: 0, 50, 100, 500, 1000, 2000 г.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Добавить корень проекта в sys.path
sys.path.insert(0, str(Path(__file__).parent.parent))

from perception.tactile.rh56dftp import (
    RH56DFTPDriver, MockRH56DFTPDriver, run_full_calibration,
)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--hand", choices=["left", "right"], default="right")
    p.add_argument("--port", default="/dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--slave", type=lambda x: int(x, 0), default=0x01)
    p.add_argument("--mock", action="store_true", help="Симуляция без железа")
    p.add_argument("--out", default="configs/tactile_calibration.json")
    args = p.parse_args()

    if args.mock:
        print("=== MOCK CALIBRATION (для теста пайплайна) ===")
        driver = MockRH56DFTPDriver(hand=args.hand)
        driver.connect()
        # Мок не калибруется реально — просто пишем placeholder
        import json
        Path(args.out).parent.mkdir(exist_ok=True, parents=True)
        Path(args.out).write_text(json.dumps({
            "_comment": "MOCK calibration — не для реального использования",
            "hand": args.hand,
            "force_sensors": [{"slope": 0.1, "offset": 0.0} for _ in range(6)],
        }, indent=2), encoding="utf-8")
        print(f"✅ Mock calibration written to {args.out}")
        return

    driver = RH56DFTPDriver(
        hand=args.hand,
        port=args.port,
        baudrate=args.baud,
        slave_address=args.slave,
        calibration_path=None,
    )
    try:
        driver.connect()
    except Exception as e:
        print(f"❌ Не удалось подключиться: {e}")
        print("   Проверь: --port, --baud, --slave, USB-RS485 адаптер")
        sys.exit(1)

    try:
        run_full_calibration(driver, output_path=args.out)
    except KeyboardInterrupt:
        print("\nПрервано пользователем")
    finally:
        driver.close()


if __name__ == "__main__":
    main()
