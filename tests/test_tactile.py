"""
tests/test_tactile.py
─────────────────────
Подключение RH56DFTP через Modbus RTU / RS-485.

Протокол:
- 6 силовых сенсоров (0.1 Н/LSB)
- 5-17 тактильных сенсоров
- Modbus RTU, 115200 baud
"""
import sys
import time
from pathlib import Path


def test_tactile():
    print("[1/4] Импорт pymodbus...")
    try:
        from pymodbus.client import ModbusSerialClient
        print("  OK")
    except ImportError as e:
        print(f"  FAIL: {e}")
        print("  pip install pymodbus")
        return False

    # Поиск RS-485 портов
    print("[2/4] Поиск RS-485 портов...")
    import glob
    candidates = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*") + glob.glob("/dev/tty.SLAB_USBtoUART*")
    if not candidates:
        print("  FAIL: не найдено /dev/ttyUSB* или /dev/ttyACM*")
        print("  Подключи RS-485 → USB адаптер")
        return False
    print(f"  Найдены: {candidates}")

    # Попытка подключения на каждом порту
    for port in candidates:
        for baud in [115200, 9600, 19200, 57600]:
            print(f"[3/4] Пытаемся {port} @ {baud}...")
            try:
                client = ModbusSerialClient(
                    port=port,
                    baudrate=baud,
                    bytesize=8,
                    parity="N",
                    stopbits=1,
                    timeout=0.2,
                )
                if not client.connect():
                    continue
                # Опрос регистра статуса
                for slave_addr in [0x01, 0x02, 0x10, 0x11]:
                    rr = client.read_holding_registers(address=0x0001, count=1, slave=slave_addr)
                    if not rr.isError():
                        print(f"  ✓ Ответ на slave_addr=0x{slave_addr:02X} @ {port} @ {baud}")
                        client.close()
                        print(f"\n✅ RH56DFTP НАЙДЕН: {port} @ {baud}, slave=0x{slave_addr:02X}")
                        # Сохранить конфиг
                        import json
                        Path("configs").mkdir(exist_ok=True)
                        Path("configs/tactile_port.json").write_text(json.dumps({
                            "port": port,
                            "baudrate": baud,
                            "slave_address": slave_addr,
                        }, indent=2), encoding="utf-8")
                        return True
                client.close()
            except Exception as e:
                print(f"  Ошибка: {e}")
                continue

    print("\n❌ RH56DFTP не отвечает ни на одном порту")
    print("  Проверь:")
    print("  - RS-485 подключение (A/B/GND)")
    print("  - Терминатор (120 Ом) если длинная линия")
    print("  - Питание кисти (RH56DFTP нужно отдельное 24V)")
    return False


def read_samples():
    """Чтение 5 секунд показаний сенсоров."""
    from pymodbus.client import ModbusSerialClient
    import json
    from pathlib import Path

    cfg = json.loads(Path("configs/tactile_port.json").read_text())
    client = ModbusSerialClient(
        port=cfg["port"],
        baudrate=cfg["baudrate"],
        bytesize=8, parity="N", stopbits=1, timeout=0.1,
    )
    client.connect()

    print("\nЧтение сенсоров 5 секунд...")
    t0 = time.time()
    while time.time() - t0 < 5:
        try:
            forces = client.read_holding_registers(
                address=0x0100, count=6, slave=cfg["slave_address"]
            )
            if not forces.isError():
                raw = list(forces.registers)
                # Конвертация: 0.1 Н/LSB
                forces_n = [r * 0.1 for r in raw]
                print(f"  forces (N): {[f'{f:.2f}' for f in forces_n]}  total={sum(forces_n):.2f}N")
            time.sleep(0.1)
        except Exception as e:
            print(f"  Ошибка чтения: {e}")

    client.close()


if __name__ == "__main__":
    if test_tactile():
        read_samples()
