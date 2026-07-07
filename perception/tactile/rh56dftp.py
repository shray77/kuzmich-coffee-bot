"""
perception/tactile/rh56dftp.py
──────────────────────────────
Драйвер тактильных сенсоров RH56DFTP (Robot Heart 56D FTP).

Кисти Unitree G1 EDU Ultimate:
- 6 DoF на каждую кисть
- 12 суставов
- Интегрированные сенсоры усилия: диапазон 10-2500 г
- Протокол: RS-485 (нужно уточнить в доке Unitree/Robot Heart)

Этот модуль — НАБРОСОК. Реальные адреса регистров и протокол нужно
взять из официальной доки RH56DFTP (Robot Heart).

Стратегия:
1. Опрос сенсоров на 50-100 Гц
2. Калибровка по таблице (configs/tactile_calibration.json)
3. Closed-loop grip: сжать пока force > threshold ИЛИ моторный ток > limit
4. Реакция на скольжение (slip detection) через derivative force
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np


@dataclass
class TactileReading:
    timestamp: float
    # 5 пальцев на кисти × N сенсоров (уточнить по доке)
    forces_g: list[float]   # в граммах
    total_force_g: float
    slip_detected: bool = False


class RH56DFTPDriver:
    """Драйвер тактильных сенсоров RH56DFTP.

    ВНИМАНИЕ: адреса регистров и протокол — placeholder. Заменить на реальные
    из документации Robot Heart / Unitree G1 SDK.
    """

    def __init__(
        self,
        hand: str = "right",          # "left" | "right"
        port: str = "/dev/ttyUSB0",   # RS-485
        baudrate: int = 115200,
        calibration_path: Optional[str] = None,
    ):
        self.hand = hand
        self.port = port
        self.baudrate = baudrate
        self.calibration = self._load_calibration(calibration_path)
        self._ser = None  # pyserial Serial — открыть в connect()

    def _load_calibration(self, path: Optional[str]) -> dict:
        """Загружает калибровочную таблицу: raw_value → граммы."""
        if path and Path(path).exists():
            return json.loads(Path(path).read_text(encoding="utf-8"))
        # Дефолтная (placeholder) — 1:1 маппинг
        return {
            "fingers": [
                {"slope": 1.0, "offset": 0.0} for _ in range(5)
            ]
        }

    def connect(self) -> None:
        """Открыть RS-485 порт. TODO: реализовать через pyserial."""
        # import serial
        # self._ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        raise NotImplementedError(
            "Подставить реальный протокол из доки RH56DFTP. "
            "Возможно, он уже входит в unitree_sdk2py как part of Hand SDK."
        )

    def read_raw(self) -> list[int]:
        """Читает raw-значения сенсоров (5 пальцев)."""
        # TODO: отправить Modbus/RS-485 запрос, распарсить ответ
        return [0, 0, 0, 0, 0]

    def read_forces(self) -> TactileReading:
        """Читает силы (граммы) после калибровки."""
        raw = self.read_raw()
        forces = []
        for i, r in enumerate(raw):
            cal = self.calibration["fingers"][i]
            forces.append(r * cal["slope"] + cal["offset"])
        total = sum(forces)
        # Slip detection: если сила резко падает при сохранении позиции — скольжение
        slip = False  # TODO: реализовать через history + derivative
        return TactileReading(
            timestamp=time.time(),
            forces_g=forces,
            total_force_g=total,
            slip_detected=slip,
        )


# ─── Closed-loop grip ───────────────────────────────────────────────────

class GripController:
    """Closed-loop хват: сжимаем пальцы пока сила не достигнет target."""

    def __init__(
        self,
        driver: RH56DFTPDriver,
        target_force_g: float = 200.0,
        max_force_g: float = 800.0,           # защита от раздавливания
        slip_threshold_g_per_s: float = 50.0, # если сила падает быстрее — slip
        poll_hz: float = 50.0,
    ):
        self.driver = driver
        self.target_force_g = target_force_g
        self.max_force_g = max_force_g
        self.slip_threshold_g_per_s = slip_threshold_g_per_s
        self.poll_interval = 1.0 / poll_hz
        self._history: list[TactileReading] = []

    def grip_step(self, current_position: float) -> tuple[float, str]:
        """
        Один шаг управления. Возвращает (новую_позицию_пальцев, статус).

        current_position: 0.0 = открыто, 1.0 = полностью закрыто
        status: 'gripping' | 'stable' | 'slip' | 'overforce' | 'aborted'
        """
        reading = self.driver.read_forces()
        self._history.append(reading)
        if len(self._history) > 10:
            self._history.pop(0)

        # Защита от overforce
        if reading.total_force_g > self.max_force_g:
            return current_position, "overforce"

        # Slip detection: сравниваем с предыдущим чтением
        if len(self._history) >= 2:
            prev = self._history[-2]
            dt = reading.timestamp - prev.timestamp
            if dt > 0:
                dforce = reading.total_force_g - prev.total_force_g
                # Сила резко падает → предмет выскальзывает
                if dforce / dt < -self.slip_threshold_g_per_s:
                    return current_position + 0.05, "slip"

        # Достигли целевой силы → стабилизация
        if reading.total_force_g >= self.target_force_g:
            return current_position, "stable"

        # Иначе сжимаем дальше
        return current_position + 0.02, "gripping"

    def release(self) -> float:
        """Разжимаем пальцы."""
        return 0.0


# ─── Тест ────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    # Без реального железа — симуляция
    print("RH56DFTP driver — SIMULATION MODE")
    print("Подключи реальный сенсор и реализуй connect()/read_raw()")
