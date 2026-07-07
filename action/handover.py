"""
action/handover.py
──────────────────
Передача чашки из руки робота в руку человека.

Сценарий:
1. Кузьмич держит чашку в правой руке (cup_grasped=True)
2. Поднимает руку в позицию "handover" (на уровень груди человека ~1.2 м)
3. Подаёт голосовую фразу "держи, Олежа"
4. Ждёт пока человек возьмётся за чашку
5. Когда тактильные сенсоры фиксируют:
   - рост ВНЕШНЕЙ силы (палец человека давит на чашку снаружи)
   - И одновременное снижение ВНУТРЕННЕЙ силы (чашка не давит на пальцы робота)
   → это сигнал "человек держит чашку, можно отпускать"
6. Плавно разжимаем пальцы (0.5 сек)
7. Отводим руку обратно в позицию "release"

⚠️ RH56DFTP даёт суммарную силу (force) на каждый палец, не разделяя
«внутреннюю» (от чашки) и «внешнюю» (от человека). Поэтому детектируем
**падение суммарной силы ниже baseline** — это означает, что чашку забрали.

API:
    handover = HandoverController(robot, tactile, arm_ctrl)
    result = handover.execute()
    if result.success: print("Передача успешна")
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional


@dataclass
class HandoverResult:
    success: bool
    reason: str               # 'grip_released' | 'timeout' | 'cup_dropped' | 'aborted'
    duration_s: float
    final_force_n: float
    samples_taken: int = 0


class HandoverController:
    """Контроллер передачи чашки человеку.

    Зависимости (внедряются через конструктор для mock-тестирования):
    - robot: UnitreeG1Interface | MockG1Interface
    - tactile: RH56DFTPDriver | MockRH56DFTPDriver  (нужен метод .read() → TactileReading)
    - arm_ctrl: ArmController | None  (опционально, для перемещения руки в handover pose)
    """

    def __init__(
        self,
        robot,
        tactile_driver,
        arm_ctrl=None,
        hand: str = "right",
        # Триггеры передачи
        internal_force_drop_n: float = 1.0,    # сила на пальцах упала на 1 Н = чашку забрали
        baseline_min_n: float = 0.3,            # минимальная стартовая сила (иначе нечего передавать)
        timeout_s: float = 15.0,
        release_duration_s: float = 0.5,
        poll_hz: float = 20.0,
    ):
        self.robot = robot
        self.tactile = tactile_driver
        self.arm_ctrl = arm_ctrl
        self.hand = hand
        self.int_drop_n = internal_force_drop_n
        self.baseline_min_n = baseline_min_n
        self.timeout = timeout_s
        self.release_duration = release_duration_s
        self.poll_interval = 1.0 / poll_hz

    def execute(self) -> HandoverResult:
        """Полный цикл передачи чашки."""
        t0 = time.time()
        samples = 0

        # 1. Поднять руку в позицию handover (~1.2 м)
        if self.arm_ctrl is not None:
            print(f"  [Handover] moving {self.hand} arm to 'handover' pose...")
            self.arm_ctrl.move_to_pose("handover", blocking=True)

        # 2. Записать baseline силу (статичная чашка в руке)
        try:
            reading = self.tactile.read()
            baseline = reading.total_force_n
        except Exception as e:
            return HandoverResult(False, "aborted", 0.0, 0.0, 0)

        print(f"  [Handover] baseline force: {baseline:.2f} N")
        samples += 1

        if baseline < self.baseline_min_n:
            print(f"  [Handover] ABORT: baseline too low ({baseline:.2f} < {self.baseline_min_n})")
            return HandoverResult(False, "aborted", time.time() - t0, baseline, samples)

        # 3. Голосовая фраза (если robot умеет TTS)
        try:
            tts = getattr(self.robot, "say", None)
            if callable(tts):
                tts("Держи, Олежа")
        except Exception:
            pass

        # 4. Ждать, пока сила не упадёт (человек забрал чашку)
        while time.time() - t0 < self.timeout:
            try:
                reading = self.tactile.read()
            except Exception as e:
                print(f"  [Handover] tactile read error: {e}")
                time.sleep(self.poll_interval)
                continue

            samples += 1
            drop = baseline - reading.total_force_n

            # Триггер: сила упала на > int_drop_n → человек взял чашку
            if drop > self.int_drop_n:
                print(f"  [Handover] grip release detected "
                      f"(drop={drop:.2f} N, total={reading.total_force_n:.2f} N)")
                return self._release(t0, samples)

            # Slip detection — чашка выскальзывает, прерываем
            if reading.slip_detected:
                print("  [Handover] SLIP detected, aborting")
                return HandoverResult(False, "cup_dropped", time.time() - t0,
                                       reading.total_force_n, samples)

            time.sleep(self.poll_interval)

        print(f"  [Handover] TIMEOUT — nobody took the cup ({self.timeout}s)")
        return HandoverResult(False, "timeout", time.time() - t0, 0.0, samples)

    def _release(self, t0: float, samples: int) -> HandoverResult:
        """Плавно разжимаем пальцы."""
        steps = 10
        dt = self.release_duration / steps
        for i in range(steps, 0, -1):
            grip_strength = i / steps
            try:
                self.tactile.close_hand(grip_strength)
            except Exception as e:
                print(f"  [Handover] close_hand failed: {e}")
            time.sleep(dt)

        # Полностью открыто
        try:
            self.tactile.open_hand()
        except Exception as e:
            print(f"  [Handover] open_hand failed: {e}")

        # Отвести руку в 'release' позицию
        if self.arm_ctrl is not None:
            self.arm_ctrl.move_to_pose("release", blocking=True)

        try:
            final = self.tactile.read().total_force_n
        except Exception:
            final = 0.0

        return HandoverResult(
            success=True,
            reason="grip_released",
            duration_s=time.time() - t0,
            final_force_n=final,
            samples_taken=samples,
        )


# ─── Mock-тест ────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).parent.parent))

    print("=== Handover controller — MOCK TEST ===\n")

    from interfaces.unitree_sdk import MockG1Interface
    from perception.tactile.rh56dftp import MockRH56DFTPDriver, TactileReading
    from control.arm_controller import ArmController

    robot = MockG1Interface()
    robot.connect()

    tactile = MockRH56DFTPDriver(hand="right", max_force_n=3.0)
    tactile.connect()
    tactile.close_hand(0.7)  # имитируем что держим чашку

    arm = ArmController(robot, side="right")
    arm.enable()

    handover = HandoverController(
        robot=robot,
        tactile_driver=tactile,
        arm_ctrl=arm,
        internal_force_drop_n=1.0,
        timeout_s=5.0,  # для теста меньше
    )

    # Запуск в фоновом потоке + симуляция: через 2 сек «человек берёт чашку»
    import threading
    def simulate_human():
        time.sleep(2.0)
        print("\n  [sim] human takes the cup — applying external force then releasing")
        tactile.apply_external_force(2.0)  # человек давит
        time.sleep(0.5)
        tactile.apply_external_force(0.0)  # забрал — давление упало
        tactile._grip_pos = 0.0           # имитируем что чашки больше нет

    threading.Thread(target=simulate_human, daemon=True).start()

    result = handover.execute()
    print(f"\nResult: success={result.success}, reason={result.reason}, "
          f"duration={result.duration_s:.2f}s, samples={result.samples_taken}")
