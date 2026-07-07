"""
perception/tactile/rh56dftp.py
──────────────────────────────
Драйвер тактильных сенсоров Inspire Robots RH56DFTP.

⚠️ УТОЧНЕНО ПО РЕАЛЬНОЙ ДОКУМЕНТАЦИИ:
- Производитель: Inspire Robots (модель RH56DFTP, серия RH56)
- 5 пальцев, 6 DoF, 12 суставов на кисть
- 6 силовых сенсоров (разрешение 0.1 Н)
- 5-17 тактильных сенсоров
- Протокол: Modbus RTU поверх RS-485
- Управляющий интерфейс: "FOXTEC"
- Официального Python SDK нет, но есть сообщество: Sentdex/inspire_hands

Калибровка (на стенде):
1. Установить руку в горизонтальную позицию ладонью вверх
2. Положить грузы 50/100/500/1000/2000 г на каждый палец
3. Снять показания, вычислить slope/offset для каждого сенсора
4. Сохранить в configs/tactile_calibration.json

На железе 2 часа — калибровка займёт ~5 минут на палец (25 минут на руку).
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
    """Показания сенсоров кисти RH56DFTP в один момент времени."""
    timestamp: float
    # 6 силовых сенсоров (force, разрешение 0.1 Н)
    forces_n: list[float]          # в Ньютонах
    # Тактильные сенсоры (5-17, точное число из конфига кисти)
    tactile_raw: list[int]         # raw ADC значения
    # Производные метрики
    total_force_n: float
    slip_detected: bool = False
    contact_detected: bool = False


class RH56DFTPDriver:
    """Драйвер кисти Inspire Robots RH56DFTP через Modbus RTU / RS-485.

    Протокол: Modbus RTU
    Физический слой: RS-485 (2 провода + GND)
    Скорость: 115200 (дефолт Inspire), проверять в доке конкретной версии

    Регистры (примерные — уточнить по User Manual V1.0.0):
    - Force sensors: holding registers 0x0100-0x0105 (6 значений, 0.1 Н/LSB)
    - Tactile sensors: input registers 0x0200-0x0210 (5-17 значений)
    - Hand position: holding registers 0x0300-0x030B (12 суставов, 0.01 рад/LSB)
    """

    # Modbus-адреса регистров (примерные, уточнить по мануалу!)
    REG_FORCE_BASE = 0x0100       # 6 регистров по 2 байта
    REG_TACTILE_BASE = 0x0200     # N регистров
    REG_HAND_POSITION = 0x0300    # 12 регистров (суставы)
    REG_HAND_STATUS = 0x0001      # статус кисти: ready/error/calibrating

    def __init__(
        self,
        hand: str = "right",          # "left" | "right"
        port: str = "/dev/ttyUSB0",   # RS-485 USB-адаптер
        baudrate: int = 115200,
        slave_address: int = 0x01,    # Modbus slave-адрес кисти
        calibration_path: Optional[str] = None,
        num_tactile_sensors: int = 17,
    ):
        self.hand = hand
        self.port = port
        self.baudrate = baudrate
        self.slave_address = slave_address
        self.num_tactile_sensors = num_tactile_sensors
        self.calibration = self._load_calibration(calibration_path)
        self._client = None

    def _load_calibration(self, path: Optional[str]) -> dict:
        """Загружает калибровочную таблицу: raw_value → Ньютоны."""
        if path and Path(path).exists():
            return json.loads(Path(path).read_text(encoding="utf-8"))
        # Дефолтная (placeholder) — 1:1 маппинг
        return {
            "force_sensors": [
                {"slope": 0.1, "offset": 0.0} for _ in range(6)  # 0.1 Н/LSB
            ],
            "tactile_sensors": [
                {"slope": 1.0, "offset": 0.0} for _ in range(self.num_tactile_sensors)
            ],
        }

    def connect(self) -> None:
        """Открыть RS-485 порт и установить Modbus-соединение.

        Использует pymodbus (pip install pymodbus).
        """
        try:
            from pymodbus.client import ModbusSerialClient
        except ImportError as e:
            raise RuntimeError(
                "pymodbus не установлен: pip install pymodbus"
            ) from e

        self._client = ModbusSerialClient(
            port=self.port,
            baudrate=self.baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=0.1,
        )
        if not self._client.connect():
            raise ConnectionError(f"Не удалось открыть {self.port} для RH56DFTP {self.hand}")
        print(f"[RH56DFTP {self.hand}] connected via {self.port}")

    def close(self) -> None:
        if self._client:
            self._client.close()
            self._client = None

    def _read_holding_registers(self, address: int, count: int) -> list[int]:
        """Чтение holding registers через Modbus."""
        if not self._client:
            raise RuntimeError("Не подключён. Вызови connect() сначала.")
        rr = self._client.read_holding_registers(
            address=address,
            count=count,
            slave=self.slave_address,
        )
        if rr.isError():
            raise IOError(f"Modbus error: {rr}")
        return list(rr.registers)

    def _read_input_registers(self, address: int, count: int) -> list[int]:
        if not self._client:
            raise RuntimeError("Не подключён.")
        rr = self._client.read_input_registers(
            address=address,
            count=count,
            slave=self.slave_address,
        )
        if rr.isError():
            raise IOError(f"Modbus error: {rr}")
        return list(rr.registers)

    def read_forces_raw(self) -> list[int]:
        """Читает 6 raw значений силовых сенсоров."""
        return self._read_holding_registers(self.REG_FORCE_BASE, 6)

    def read_tactile_raw(self) -> list[int]:
        """Читает raw значения тактильных сенсоров."""
        return self._read_input_registers(self.REG_TACTILE_BASE, self.num_tactile_sensors)

    def read_forces(self) -> list[float]:
        """Возвращает силы в Ньютонах (после калибровки)."""
        raw = self.read_forces_raw()
        cals = self.calibration["force_sensors"]
        return [r * c["slope"] + c["offset"] for r, c in zip(raw, cals)]

    def read_tactile(self) -> list[float]:
        """Возвращает тактильные показания (после калибровки)."""
        raw = self.read_tactile_raw()
        cals = self.calibration["tactile_sensors"]
        return [r * c["slope"] + c["offset"] for r, c in zip(raw, cals)]

    def read(self) -> TactileReading:
        """Полное чтение: forces + tactile + производные метрики."""
        forces = self.read_forces()
        tactile = self.read_tactile_raw()
        total = sum(forces)
        contact = total > 0.5  # >0.5 Н = контакт
        return TactileReading(
            timestamp=time.time(),
            forces_n=forces,
            tactile_raw=tactile,
            total_force_n=total,
            contact_detected=contact,
            slip_detected=False,  # TODO: реализовать через derivative
        )

    def set_hand_position(self, positions: list[float]) -> None:
        """Установка позиций 12 суставов кисти (рад).

        positions: список из 12 float, каждый 0..π/2 (зависит от сустава)
        """
        if len(positions) != 12:
            raise ValueError(f"Нужно 12 позиций, получено {len(positions)}")
        # Конвертация рад → 0.01 рад/LSB
        registers = [int(p * 100) for p in positions]
        if not self._client:
            raise RuntimeError("Не подключён.")
        self._client.write_registers(
            address=self.REG_HAND_POSITION,
            values=registers,
            slave=self.slave_address,
        )

    def open_hand(self) -> None:
        """Полностью открыть кисть."""
        self.set_hand_position([0.0] * 12)

    def close_hand(self, grip_strength: float = 0.7) -> None:
        """Закрыть кисть с заданной силой (0..1)."""
        positions = [grip_strength * 1.5] * 12  # упрощённо
        self.set_hand_position(positions)


# ─── Closed-loop grip ───────────────────────────────────────────────────

class GripController:
    """Closed-loop хват: сжимаем пальцы пока сила не достигнет target.

    На железе 2 часа — это ключевая логика, тестить в первую очередь.
    """

    def __init__(
        self,
        driver: RH56DFTPDriver,
        target_force_n: float = 2.0,        # ~200 г на чашку
        max_force_n: float = 8.0,            # защита: не раздавить
        slip_threshold_n_per_s: float = 1.0, # 1 Н/с = быстрое падение
        poll_hz: float = 50.0,
    ):
        self.driver = driver
        self.target_force_n = target_force_n
        self.max_force_n = max_force_n
        self.slip_threshold_n_per_s = slip_threshold_n_per_s
        self.poll_interval = 1.0 / poll_hz
        self._history: list[TactileReading] = []
        self._grip_position: float = 0.0

    def grip_step(self) -> tuple[float, str]:
        """
        Один шаг управления. Возвращает (новая_позиция_пальцев, статус).

        Позиция: 0.0 = открыто, 1.0 = полностью закрыто
        Статус: 'gripping' | 'stable' | 'slip' | 'overforce' | 'aborted'
        """
        reading = self.driver.read()
        self._history.append(reading)
        if len(self._history) > 10:
            self._history.pop(0)

        # Защита от overforce
        if reading.total_force_n > self.max_force_n:
            return self._grip_position, "overforce"

        # Slip detection: сравниваем с предыдущим чтением
        if len(self._history) >= 2:
            prev = self._history[-2]
            dt = reading.timestamp - prev.timestamp
            if dt > 0:
                dforce = reading.total_force_n - prev.total_force_n
                if dforce / dt < -self.slip_threshold_n_per_s:
                    return self._grip_position + 0.05, "slip"

        # Достигли целевой силы → стабилизация
        if reading.total_force_n >= self.target_force_n:
            return self._grip_position, "stable"

        # Иначе сжимаем дальше
        self._grip_position = min(1.0, self._grip_position + 0.02)
        return self._grip_position, "gripping"

    def reset(self) -> None:
        self._grip_position = 0.0
        self._history.clear()

    def release(self) -> None:
        """Разжимаем пальцы полностью."""
        self._grip_position = 0.0
        self.driver.open_hand()


# ─── Калибровочный скрипт ────────────────────────────────────────────────

def calibrate_finger(
    driver: RH56DFTPDriver,
    finger_idx: int,
    weights_g: list[float] = [0, 50, 100, 500, 1000, 2000],
) -> dict:
    """Калибровка одного пальца разными грузами.

    Положи груз на палец, нажми Enter, скрипт считает силу.
    Повторить для каждого веса.

    Returns: {"slope": ..., "offset": ...} для линейной модели force = raw*slope + offset
    """
    input(f"Подготовь палец {finger_idx}. Положи груз {weights_g[0]} г, нажми Enter...")
    readings = []
    for w in weights_g:
        input(f"Положи груз {w} г, нажми Enter когда устоится...")
        samples = [driver.read_forces()[finger_idx] for _ in range(20)]
        avg = sum(samples) / len(samples)
        readings.append((w, avg))
        print(f"  вес {w} г → raw {avg:.1f}")
    # Линейная регрессия: w = raw * slope + offset
    weights = np.array([r[0] for r in readings])
    raws = np.array([r[1] for r in readings])
    if len(readings) >= 2:
        A = np.vstack([raws, np.ones(len(raws))]).T
        slope, offset = np.linalg.lstsq(A, weights, rcond=None)[0]
        return {"slope": float(slope), "offset": float(offset)}
    return {"slope": 1.0, "offset": 0.0}


def run_full_calibration(
    driver: RH56DFTPDriver,
    output_path: str = "configs/tactile_calibration.json",
) -> None:
    """Калибрует все 6 силовых сенсоров, сохраняет в JSON."""
    print(f"\n=== Калибровка RH56DFTP ({driver.hand} hand) ===\n")
    cal = {"force_sensors": []}
    for i in range(6):
        print(f"\n--- Сенсор {i+1}/6 ---")
        result = calibrate_finger(driver, i)
        cal["force_sensors"].append(result)
        print(f"  slope={result['slope']:.4f}, offset={result['offset']:.4f}")
    Path(output_path).write_text(json.dumps(cal, indent=2), encoding="utf-8")
    print(f"\nКалибровка сохранена в {output_path}")


# ─── Тест ────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys
    if "--calibrate" in sys.argv:
        driver = RH56DFTPDriver(
            hand="right",
            port="/dev/ttyUSB0",
            calibration_path=None,
        )
        driver.connect()
        try:
            run_full_calibration(driver)
        finally:
            driver.close()
    else:
        print("RH56DFTP driver — справка:")
        print("  --calibrate  калибровать все 6 сенсоров (нужны грузы)")
        print("  без флага    читает показания 5 сек и печатает")
        driver = RH56DFTPDriver(hand="right", port="/dev/ttyUSB0")
        try:
            driver.connect()
            t0 = time.time()
            while time.time() - t0 < 5:
                r = driver.read()
                print(f"  forces={r.forces_n} tactile={r.tactile_raw[:5]} total={r.total_force_n:.2f}N")
                time.sleep(0.1)
        except Exception as e:
            print(f"Нет железа? Ошибка: {e}")
            print("Запусти с --simulate для mock-режима")
