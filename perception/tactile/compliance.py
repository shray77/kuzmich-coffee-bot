"""
perception/tactile/compliance.py
─────────────────────────────────
Оценка "упругости" (compliance/stiffness) захваченного объекта по кривой
сила↔позиция пальцев во время сжатия хвата.

Зачем это нужно: чашка бывает бумажная/пластиковая (мнётся при небольшом
усилии — легко раздавить) или керамическая/полная (почти не поддаётся,
сила растёт резко при малом изменении позиции пальцев). Один и тот же
target_force_n/max_force_n в GripController, нормальный для керамической
кружки, может смять бумажный стаканчик. Зная жёсткость объекта в реальном
времени прямо во время сжатия — до того, как раздавили — можно снизить
целевую/предельную силу на лету (см. GripController.grip_step() в
rh56dftp.py, куда этот модуль подключён).

Модель: на участке монотонного сжатия сила ≈ k * позиция + b, где
позиция — 0.0 (открыто) .. 1.0 (полностью сжато), k — жёсткость в
Н/(единица позиции). Оценивается скользящим МНК (numpy.polyfit) по
последним WINDOW_SIZE сэмплам с контролем качества фита (R²) и
минимального диапазона позиции (иначе на "плато" почти без движения
пальцев жёсткость оценить нельзя — шум даёт случайный знак).

⚠️ SOFT_STIFFNESS_MAX_N / RIGID_STIFFNESS_MIN_N — PLACEHOLDER. Снимаются
калибровкой на реальной RH56DFTP: сжать по очереди бумажный стакан и
керамическую кружку тем же ComplianceEstimator, записать получившиеся k,
выставить пороги между ними. Точно так же, как калибровка force-сенсоров
в rh56dftp.py (calibrate_finger/run_full_calibration) — это разовая
процедура на стенде, не хардкод "на глаз".
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional

import numpy as np


class Compliance(Enum):
    UNKNOWN = "unknown"   # мало данных или пальцы почти не двигались — не оценить
    SOFT = "soft"         # низкая жёсткость — легко деформируется (бумага/тонкий пластик)
    MEDIUM = "medium"     # средняя жёсткость
    RIGID = "rigid"       # высокая жёсткость — почти не поддаётся (керамика/стекло/полная чашка)


@dataclass
class ComplianceReading:
    stiffness: Optional[float]     # Н на единицу позиции (0..1); None пока не оценить
    classification: Compliance
    samples_used: int
    r_squared: float = 0.0         # качество линейной аппроксимации, 0..1 (выше — надёжнее)


# ─── Пороги классификации — PLACEHOLDER, калибровать на железе ───────────
SOFT_STIFFNESS_MAX_N = 3.0
RIGID_STIFFNESS_MIN_N = 12.0

WINDOW_SIZE = 6           # сколько последних сэмплов (position, force) берём в МНК
MIN_SAMPLES = 3           # меньше — оценка не считается (слишком шумно)
MIN_POSITION_SPAN = 0.03  # пальцы должны реально сдвинуться, иначе фит бессмысленный
MIN_R_SQUARED = 0.5       # ниже — оценке жёсткости не доверяем (плохой линейный фит)
MIN_CONTACT_FORCE_N = 0.05  # ниже — пальцы ещё не коснулись объекта, это не "мягкий", а "нет контакта"


class ComplianceEstimator:
    """
    Онлайн-оценка жёсткости объекта по последовательности (position, force_n),
    собираемой во время фазы сжатия хвата (см. GripController.grip_step()).

    Использование:
        est = ComplianceEstimator()
        for position, force_n in grip_samples:   # только пока хват реально сжимается
            est.update(position, force_n)
        reading = est.estimate()
        if reading.classification == Compliance.SOFT and reading.r_squared >= MIN_R_SQUARED:
            ...снизить целевую силу...
    """

    def __init__(self, window_size: int = WINDOW_SIZE):
        self.window_size = window_size
        self._positions: list[float] = []
        self._forces: list[float] = []

    def reset(self) -> None:
        """Сбросить перед новой попыткой захвата — прошлый объект тут ни при чём."""
        self._positions.clear()
        self._forces.clear()

    def update(self, position: float, force_n: float) -> None:
        """Добавить сэмпл. Вызывать только пока хват реально закрывается
        (не во время 'stable'/'slip'/'overforce' — там позиция и/или сила
        ведут себя не по модели монотонного сжатия и портят фит)."""
        self._positions.append(position)
        self._forces.append(force_n)
        if len(self._positions) > self.window_size:
            self._positions.pop(0)
            self._forces.pop(0)

    def estimate(self) -> ComplianceReading:
        """Текущая оценка жёсткости по накопленному окну сэмплов."""
        n = len(self._positions)
        if n < MIN_SAMPLES:
            return ComplianceReading(stiffness=None, classification=Compliance.UNKNOWN, samples_used=n)

        positions = np.asarray(self._positions, dtype=float)
        forces = np.asarray(self._forces, dtype=float)

        span = float(positions.max() - positions.min())
        if span < MIN_POSITION_SPAN:
            return ComplianceReading(stiffness=None, classification=Compliance.UNKNOWN, samples_used=n)

        if forces.max() < MIN_CONTACT_FORCE_N:
            # Пальцы двигаются, но силы ещё нет — объект ещё не тронут.
            # Это "нет контакта", а не "нулевая жёсткость/мягкий объект".
            return ComplianceReading(stiffness=None, classification=Compliance.UNKNOWN, samples_used=n)

        # МНК: force = k*position + b
        k, b = np.polyfit(positions, forces, 1)

        predicted = k * positions + b
        ss_res = float(np.sum((forces - predicted) ** 2))
        ss_tot = float(np.sum((forces - forces.mean()) ** 2))
        # ss_tot≈0 значит сила на окне вообще не менялась (плато — типично для
        # грубо квантованных сенсоров). Для OLS всегда ss_res <= ss_tot, так что
        # ss_tot≈0 подразумевает и ss_res≈0 — это идеальный (пусть и тривиальный)
        # фит, а не "не доверяй": раньше тут стоял r_squared=0.0, из-за чего
        # is_confidently_soft() никогда не срабатывал на таких плато.
        r_squared = 1.0 - ss_res / ss_tot if ss_tot > 1e-9 else 1.0

        # Отрицательная жёсткость (сила падает по мере сжатия) нефизична для
        # этой модели — это слип/шум, а не свойство объекта. Клампим в 0,
        # r_squared всё равно передаём наружу как сигнал "не доверяй".
        stiffness = max(0.0, float(k))

        return ComplianceReading(
            stiffness=stiffness,
            classification=self._classify(stiffness),
            samples_used=n,
            r_squared=r_squared,
        )

    def _classify(self, stiffness: float) -> Compliance:
        if stiffness < SOFT_STIFFNESS_MAX_N:
            return Compliance.SOFT
        if stiffness > RIGID_STIFFNESS_MIN_N:
            return Compliance.RIGID
        return Compliance.MEDIUM


def is_confidently_soft(reading: ComplianceReading) -> bool:
    """True если можно доверять классификации SOFT (не шум/недостаток данных)."""
    return reading.classification == Compliance.SOFT and reading.r_squared >= MIN_R_SQUARED


# ─── Самотест на синтетических данных ─────────────────────────────────────

if __name__ == "__main__":
    print("=== ComplianceEstimator — синтетический тест ===\n")

    def simulate(k_true: float, noise_std: float = 0.05, steps: int = 20) -> ComplianceReading:
        est = ComplianceEstimator()
        pos = 0.0
        for _ in range(steps):
            pos += 0.03
            force = k_true * pos + np.random.normal(0, noise_std)
            est.update(pos, max(0.0, force))
        return est.estimate()

    np.random.seed(0)

    soft = simulate(k_true=1.5)
    print(f"Бумажный стакан (k_true=1.5): {soft}")
    assert soft.classification == Compliance.SOFT, soft

    medium = simulate(k_true=7.0)
    print(f"Средний объект (k_true=7.0):  {medium}")
    assert medium.classification == Compliance.MEDIUM, medium

    rigid = simulate(k_true=20.0)
    print(f"Керамика (k_true=20.0):       {rigid}")
    assert rigid.classification == Compliance.RIGID, rigid

    empty = ComplianceEstimator().estimate()
    print(f"Без данных:                   {empty}")
    assert empty.classification == Compliance.UNKNOWN

    plateau = ComplianceEstimator()
    for _ in range(5):
        plateau.update(0.5, 2.0 + np.random.normal(0, 0.01))
    stuck = plateau.estimate()
    print(f"Пальцы не двигаются (плато):  {stuck}")
    assert stuck.classification == Compliance.UNKNOWN, stuck

    print("\n✅ все синтетические сценарии прошли")
