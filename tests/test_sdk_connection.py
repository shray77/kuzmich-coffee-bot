"""
tests/test_sdk_connection.py
────────────────────────────
Первый скрипт на железе — проверка подключения к G1 через CycloneDDS.

Запуск:
  export CYCLONEDDS_URI=file://$(pwd)/cyclonedds.xml
  python tests/test_sdk_connection.py
"""
import sys
import time

def test_connection(iface: str = "eth0"):
    print("[1/4] Импорт unitree_sdk2py...")
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        # G1 использует LocoClient (Sport Services), НЕ Go2 SportClient —
        # это разные роботы с разным API (тот же фикс что в interfaces/unitree_sdk.py).
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        print("  OK")
    except ImportError as e:
        print(f"  FAIL: {e}")
        print("  Установи: git clone https://github.com/unitreerobotics/unitree_sdk2_python && pip install -e .")
        return False

    print("[2/4] Инициализация ChannelFactory...")
    try:
        # ChannelFactoryInitialize — отдельная функция, не метод класса
        # (ChannelFactory.Initialize реально не существует — проверено на роботе).
        # domainId=0 явно, без auto-detect — так везде в проверенном коде.
        ChannelFactoryInitialize(0, iface)
        print("  OK")
    except Exception as e:
        print(f"  FAIL: {e}")
        return False

    print("[3/4] Создание LocoClient...")
    try:
        sc = LocoClient()
        sc.SetTimeout(5.0)
        sc.Init()
        print("  OK")
    except Exception as e:
        print(f"  FAIL: {e}")
        return False

    print("[4/4] Ping G1 (чтение state)...")
    try:
        time.sleep(0.5)
        print("  LocoClient готов к командам")
        print("\nПОДКЛЮЧЕНО К G1!")
        return True
    except Exception as e:
        print(f"  FAIL: {e}")
        return False


if __name__ == "__main__":
    iface = sys.argv[1] if len(sys.argv) > 1 else "eth0"
    success = test_connection(iface)
    sys.exit(0 if success else 1)
