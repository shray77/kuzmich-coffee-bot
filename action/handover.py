"""
action/handover.py
──────────────────
Передача чашки из руки робота в руку человека.

Сценарий:
1. Кузьмич держит чашку в правой руке (cup_grasped=True)
2. Поднимает руку на уровень груди человека (~1.2 м)
3. Подаёт голосовую фразу "держи, Олежа"
4. Ждёт пока человек возьмётся за чашку
5. Когда тактильные сенсоры фиксируют:
   - рост ВНЕШНЕЙ силы (палец человека давит на чашку снаружи)
   - И одновременное снижение ВНУТРЕННЕЙ силы (чашка не давит на пальцы робота)
   → это сигнал "человек держит чашку, можно отпускать"
6. Плавно разжимаем пальцы (0.5 сек)
7. Отводим руку обратно к бедру
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

# from perception.tactile.rh56dftp import RH56DFTPDriver, TactileReading
# from interfaces.unitree_sdk import UnitreeG1Interface


@dataclass
class HandoverResult:
    success: bool
    reason: str               # 'grip_released' | 'timeout' | 'cup_dropped' | 'aborted'
    duration_s: float
    final_force_g: float


class HandoverController:
    """Контроллер передачи чашки."""

    def __init__(
        self,
        robot,                # UnitreeG1Interface
        tactile_driver,       # RH56DFTPDriver
        hand: str = "right",
        # Триггеры передачи
        external_force_threshold_g: float = 100.0,   # палец человека давит >100 г
        internal_force_drop_g: float = 50.0,         # сила на пальцах робота упала на 50 г
        timeout_s: float = 15.0,
        release_duration_s: float = 0.5,
    ):
        self.robot = robot
        self.tactile = tactile_driver
        self.hand = hand
        self.ext_threshold = external_force_threshold_g
        self.int_drop = internal_force_drop_g
        self.timeout = timeout_s
        self.release_duration = release_duration_s
        self._baseline_force: Optional[float] = None

    def execute(self) -> HandoverResult:
        """Полный цикл передачи."""
        t0 = time.time()

        # 1. Поднять руку в позицию handover (~1.2 м)
        # 7 DoF arm: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
        # Дефолтная поза handover для G1 (заглушка — уточнить по калибровке)
        handover_pose = [0.2, -0.5, 1.2, -0.7, 0.0, 0.0, 0.3]  # rad
        self.robot.set_arm_joint_positions(self.hand, handover_pose, timeout=3.0)

        # 2. Записать baseline силу (статичная чашка в руке)
        reading = self.tactile.read_forces()
        self._baseline_force = reading.total_force_g
        print(f"  [Handover] baseline force: {self._baseline_force:.1f} g")

        # 3. Ждать контакт человека
        while time.time() - t0 < self.timeout:
            reading = self.tactile.read_forces()
            drop = self._baseline_force - reading.total_force_g

            # Триггер: человек давит извне ИЛИ наша сила резко упала
            # (в реальности нужно разделить сигналы, но RH56DFTP даёт суммарную силу)
            if drop > self.int_drop:
                # Человек взял чашку — наша сила упала
                print(f"  [Handover] grip release detected (drop={drop:.1f} g)")
                return self._release(t0)

            # Slip detection
            if reading.slip_detected:
                print("  [Handover] SLIP detected, aborting")
                return HandoverResult(False, "cup_dropped", time.time()-t0, reading.total_force_g)

            time.sleep(0.05)

        print("  [Handover] TIMEOUT — nobody took the cup")
        return HandoverResult(False, "timeout", time.time()-t0, 0.0)

    def _release(self, t0: float) -> HandoverResult:
        """Плавно разжимаем пальцы."""
        steps = 10
        dt = self.release_duration / steps
        for i in range(steps, 0, -1):
            pos = i / steps
            self.robot.set_hand_position(self.hand, pos)
            time.sleep(dt)

        # Полностью открыто
        self.robot.set_hand_position(self.hand, 0.0)

        reading = self.tactile.read_forces()
        return HandoverResult(
            success=True,
            reason="grip_released",
            duration_s=time.time() - t0,
            final_force_g=reading.total_force_g,
        )


if __name__ == "__main__":
    # Симуляция без реального робота
    print("Handover controller — SIMULATION")
    print("Подключи Unitree G1 + RH56DFTP для реального теста")
