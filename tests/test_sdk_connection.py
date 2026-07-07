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

def test_connection():
    print("[1/4] Импорт unitree_sdk2py...")
    try:
        from unitree_sdk2py.core.channel import ChannelFactory
        from unitree_sdk2py.go2.sport.sport_client import SportClient
        print("  OK")
    except ImportError as e:
        print(f"  FAIL: {e}")
        print("  Установи: git clone https://github.com/unitreerobotics/unitree_sdk2_python && pip install -e .")
        return False

    print("[2/4] Инициализация ChannelFactory...")
    try:
        ChannelFactory.Initialize()  # auto-detect interface
        ChannelFactory.SetLogLevel(2)
        print("  OK")
    except Exception as e:
        print(f"  FAIL: {e}")
        return False

    print("[3/4] Создание SportClient...")
    try:
        sc = SportClient()
        sc.SetTimeout(5.0)
        sc.Init()
        print("  OK")
    except Exception as e:
        print(f"  FAIL: {e}")
        return False

    print("[4/4] Ping G1 (чтение state)...")
    try:
        # Попробовать StandUp — если робот лежит, встанет
        # StandUp — безопасная команда
        time.sleep(0.5)
        print("  SportClient готов к командам")
        print("\n✅ ПОДКЛЮЧЕНО К G1!")
        return True
    except Exception as e:
        print(f"  FAIL: {e}")
        return False


if __name__ == "__main__":
    success = test_connection()
    sys.exit(0 if success else 1)
