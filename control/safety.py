"""
control/safety.py
─────────────────
Мониторинг безопасности для G1 при выполнении coffee task.

Потенциально опасные ситуации:
1. **Overforce на пальцах** — чашка раздавлена, либо препятствие
2. **Резкий наклон корпуса** — риск падения при ходьбе с чашкой
3. **Падение (IMU z-acceleration spike)** — необходимо отпустить чашку и защититься
4. **Slip detection** — чашка выскальзывает, нужно усилить хват
5. **Battery low** — успеть вернуться на базу
6. **Watchdog timeout** — loss of DDS communication

API:
    monitor = SafetyMonitor(robot, tactile_driver)
    monitor.start()           # фоновый поток
    monitor.check()           # синхронная проверка, возвращает SafetyStatus
    monitor.emergency_stop()  # немедленная остановка
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Callable


class SafetyLevel(Enum):
    OK = "ok"
    WARNING = "warning"
    CRITICAL = "critical"
    ABORT = "abort"


@dataclass
class SafetyStatus:
    level: SafetyLevel = SafetyLevel.OK
    reasons: list[str] = field(default_factory=list)
    timestamp: float = field(default_factory=time.time)

    def add(self, level: SafetyLevel, reason: str) -> None:
        if level.value == "abort" or (level.value == "critical" and self.level != SafetyLevel.ABORT):
            self.level = level
        elif level.value == "warning" and self.level == SafetyLevel.OK:
            self.level = level
        self.reasons.append(reason)


class SafetyMonitor:
    """Фоновый мониторинг safety на G1."""

    def __init__(
        self,
        robot,
        tactile_driver: Optional[object] = None,
        callbacks: Optional[list[Callable[[SafetyStatus], None]]] = None,
        # Пороги
        max_force_n: float = 8.0,           # >8 Н на пальцах = overforce
        max_tilt_deg: float = 25.0,         # >25° наклон = риск падения
        min_battery_pct: float = 15.0,
        watchdog_timeout_s: float = 5.0,    # нет телеметрии 5с = ABORT
        poll_hz: float = 20.0,
    ):
        self.robot = robot
        self.tactile = tactile_driver
        self.callbacks = callbacks or []

        self.max_force_n = max_force_n
        self.max_tilt_deg = max_tilt_deg
        self.min_battery_pct = min_battery_pct
        self.watchdog_timeout_s = watchdog_timeout_s
        self.poll_interval = 1.0 / poll_hz

        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._last_status: SafetyStatus = SafetyStatus()
        self._last_telemetry_t: float = time.time()
        self._aborted: bool = False

    # ─── Запуск/остановка ───────────────────────────────────────────────

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        print("[Safety] monitor started")

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        print("[Safety] monitor stopped")

    def _loop(self) -> None:
        while not self._stop.is_set():
            status = self.check()
            if status.level != SafetyLevel.OK:
                print(f"[Safety] {status.level.value.upper()}: {status.reasons}")
                for cb in self.callbacks:
                    try:
                        cb(status)
                    except Exception as e:
                        print(f"[Safety] callback error: {e}")
                if status.level == SafetyLevel.ABORT and not self._aborted:
                    self.emergency_stop()
                    self._aborted = True
            time.sleep(self.poll_interval)

    # ─── Синхронная проверка ────────────────────────────────────────────

    def check(self) -> SafetyStatus:
        status = SafetyStatus()

        # 1. Tactile — overforce / slip
        if self.tactile is not None:
            try:
                reading = self.tactile.read()
                self._last_telemetry_t = time.time()

                if reading.total_force_n > self.max_force_n:
                    status.add(SafetyLevel.ABORT,
                               f"OVERFORCE: {reading.total_force_n:.1f}N > {self.max_force_n}N")

                if reading.slip_detected:
                    status.add(SafetyLevel.WARNING, "SLIP detected on fingers")
            except Exception as e:
                status.add(SafetyLevel.WARNING, f"tactile read failed: {e}")

        # 2. IMU — tilt / fall
        try:
            imu = self.robot.get_imu()
            self._last_telemetry_t = time.time()

            # Кватернион → угол наклона от вертикали
            tilt = self._compute_tilt_deg(imu.get("orientation", [0, 0, 0, 1]))
            if tilt > self.max_tilt_deg:
                status.add(SafetyLevel.ABORT, f"TILT {tilt:.1f}° > {self.max_tilt_deg}°")
        except Exception as e:
            status.add(SafetyLevel.WARNING, f"imu read failed: {e}")

        # 3. Watchdog — нет телеметрии
        if time.time() - self._last_telemetry_t > self.watchdog_timeout_s:
            status.add(SafetyLevel.ABORT, "Watchdog: no telemetry > 5s")

        # 4. Battery (если есть)
        try:
            battery = getattr(self.robot, "get_battery_pct", lambda: None)()
            if battery is not None and battery < self.min_battery_pct:
                status.add(SafetyLevel.WARNING, f"Low battery: {battery}%")
        except Exception:
            pass

        self._last_status = status
        return status

    def _compute_tilt_deg(self, quaternion: list[float]) -> float:
        """Угол наклона от вертикали по кватерниону (qx,qy,qz,qw)."""
        try:
            qx, qy, qz, qw = quaternion
            # Проекция вертикали (z) на тело робота
            z_proj = 1.0 - 2.0 * (qx * qx + qy * qy)
            import math
            return math.degrees(math.acos(max(-1.0, min(1.0, z_proj))))
        except Exception:
            return 0.0

    # ─── Аварийная остановка ─────────────────────────────────────────────

    def emergency_stop(self) -> None:
        """Немедленная остановка всех движений."""
        print("[Safety] !!! EMERGENCY STOP !!!")
        try:
            self.robot.emergency_stop()
        except Exception as e:
            print(f"[Safety] robot.estop failed: {e}")
        # Отпустить кисти (чтобы не раздавить чашку / пальцы)
        if self.tactile is not None:
            try:
                self.tactile.open_hand()
            except Exception:
                pass

    @property
    def status(self) -> SafetyStatus:
        return self._last_status

    @property
    def aborted(self) -> bool:
        return self._aborted


if __name__ == "__main__":
    # Демо с моками
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).parent.parent))
    from interfaces.unitree_sdk import MockG1Interface

    class MockTactile:
        def read(self):
            from perception.tactile.rh56dftp import TactileReading
            return TactileReading(
                timestamp=time.time(),
                forces_n=[0.5, 0.3, 0.2, 0.1, 0.1, 0.1],
                tactile_raw=[100] * 17,
                total_force_n=1.3,
                slip_detected=False,
                contact_detected=True,
            )
        def open_hand(self): pass

    robot = MockG1Interface()
    robot.connect()

    mon = SafetyMonitor(robot, MockTactile())
    print("Initial check:", mon.check())
    mon.start()
    time.sleep(2)
    mon.stop()
    print("Final:", mon.status)
