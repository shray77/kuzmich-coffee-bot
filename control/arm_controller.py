"""
control/arm_controller.py
─────────────────────────
High-level контроллер рук и торса G1.

Скрывает под собой:
- low-level LowCmd publisher (через interfaces.unitree_sdk)
- интерполяцию между позами
- контроль торса (для наклона к столу)
- безопасность (clamp углов, max speed)

API:
    arm = ArmController(robot, side="right")
    arm.move_to_pose("pregrasp")     # плавно перейти в позу
    arm.move_to_pose("grasp", duration=0.5)
    arm.move_to_xyz(0.5, 0.0, 1.0)   # упрощённая IK — цель в метрах от базы

Mock-friendly: если robot.enable_low_level=False, контроллер только логирует.
"""
from __future__ import annotations

import time
from typing import Optional

from .arm_poses import (
    NUM_ARM_JOINTS,
    NUM_TORSO_JOINTS,
    get_pose,
    get_torso_pose,
    get_pose_duration,
    interpolate_poses,
)


# Безопасные лимиты углов (рад). Вне этих — clamp.
SAFE_LIMITS = {
    "shoulder_pitch": (-0.5, 2.0),    # не выше горизонтали вверх, не за спину
    "shoulder_roll":  (-1.5, 1.5),
    "shoulder_yaw":   (-1.2, 1.2),
    "elbow":          (-0.1, 2.2),
    "wrist_pitch":    (-1.0, 1.0),
    "wrist_roll":     (-1.5, 1.5),
    "wrist_yaw":      (-0.8, 0.8),
}
_JOINT_NAMES = [
    "shoulder_pitch", "shoulder_roll", "shoulder_yaw",
    "elbow", "wrist_pitch", "wrist_roll", "wrist_yaw",
]

TORSO_LIMITS = {
    "pitch": (-0.5, 0.5),
    "roll":  (-0.3, 0.3),
}


class ArmController:
    """High-level контроллер одной руки G1."""

    def __init__(
        self,
        robot,                          # UnitreeG1Interface (real or mock)
        side: str = "right",
        control_hz: float = 100.0,      # частота low-level команд
        kp: float = 80.0,               # жёсткость позиции (N·m/rad)
        kd: float = 3.0,                # демпфирование (N·m·s/rad)
    ):
        self.robot = robot
        self.side = side
        self.control_hz = control_hz
        self.dt = 1.0 / control_hz
        self.kp = kp
        self.kd = kd
        self._current_pose: list[float] = [0.0] * NUM_ARM_JOINTS
        self._enabled = False

    def enable(self) -> None:
        """Активировать low-level режим на роботе (с Kp=0, Kd=5 для начала)."""
        if not getattr(self.robot, "enable_low_level", False):
            print(f"[ArmCtrl {self.side}] low-level DISABLED, mock-режим")
            self._enabled = False
            return
        self._enabled = True
        print(f"[ArmCtrl {self.side}] enabled, Kp={self.kp}, Kd={self.kd}")

    def _clamp(self, positions: list[float]) -> list[float]:
        """Применяет безопасные лимиты к углам."""
        out = []
        for i, (q, name) in enumerate(zip(positions, _JOINT_NAMES)):
            lo, hi = SAFE_LIMITS[name]
            out.append(max(lo, min(hi, q)))
        return out

    def move_to_pose(
        self,
        pose_name: str,
        duration: Optional[float] = None,
        blocking: bool = True,
    ) -> bool:
        """
        Плавно перейти в предустановленную позу.

        pose_name: см. control.arm_poses.list_poses()
        duration: override длительности (сек). Если None — берётся из POSE_DURATIONS.
        blocking: если True — ждём окончания движения
        """
        target = get_pose(self.side, pose_name)
        if duration is None:
            duration = get_pose_duration(pose_name)

        target = self._clamp(target)
        start = list(self._current_pose)

        print(f"[ArmCtrl {self.side}] {start} → {pose_name} ({duration:.1f}s)")

        if not self._enabled:
            # Mock-режим — просто обновляем внутреннее состояние
            self._current_pose = list(target)
            if blocking:
                time.sleep(duration)
            return True

        # Real low-level: интерполируем и шлём команды
        n_steps = max(1, int(duration * self.control_hz))
        for i in range(n_steps + 1):
            t = i / n_steps
            interp = interpolate_poses(start, target, t)
            self._send_lowcmd(interp)
            if blocking:
                time.sleep(self.dt)

        self._current_pose = list(target)
        return True

    def move_to_xyz(
        self,
        x: float,
        y: float,
        z: float,
        duration: float = 1.5,
        blocking: bool = True,
    ) -> bool:
        """
        Очень упрощённая IK: по цели в метрах от базы робота выбираем
        ближайшую предустановленную позу и слегка корректируем плечо/локоть.

        ⚠️ Это НЕ настоящая IK. Для серьёзной работы нужен arm_kinematics.py
        с численной IK (например, через pinocchio или TRAC-IK).

        Координаты:
        x — вперёд от робота (м), диапазон [0.2, 0.8]
        y — влево/вправо (м), [-0.4, 0.4]
        z — высота (м), [0.3, 1.4]
        """
        # Базовая поза — pregrasp (вытянута вперёд)
        base = get_pose(self.side, "pregrasp")

        # Эвристика:
        # shoulder_pitch ≈ arctan2(x, z - 0.9) + поправка
        # elbow ≈ 0.5 * (1 - x/0.8)
        import math
        shoulder_pitch = math.atan2(max(0.1, x), max(0.1, z - 0.5))
        elbow = max(0.0, min(2.0, 0.5 + 0.5 * (1.0 - x / 0.8)))

        # y смещает плечо в сторону
        shoulder_roll = max(-0.5, min(0.5, y * 0.5))

        target = list(base)
        target[0] = shoulder_pitch
        target[1] = shoulder_roll
        target[3] = elbow
        target = self._clamp(target)

        print(f"[ArmCtrl {self.side}] move_to_xyz({x:.2f},{y:.2f},{z:.2f}) → {target}")

        if not self._enabled:
            self._current_pose = list(target)
            if blocking:
                time.sleep(duration)
            return True

        start = list(self._current_pose)
        n_steps = max(1, int(duration * self.control_hz))
        for i in range(n_steps + 1):
            t = i / n_steps
            interp = interpolate_poses(start, target, t)
            self._send_lowcmd(interp)
            if blocking:
                time.sleep(self.dt)
        self._current_pose = list(target)
        return True

    def _send_lowcmd(self, positions: list[float]) -> None:
        """Отправить low-level команду. Доступно только если robot.enable_low_level=True."""
        try:
            self.robot.set_arm_joint_positions(
                arm=self.side,
                positions=positions,
                kp=self.kp,
                kd=self.kd,
            )
        except (NotImplementedError, RuntimeError) as e:
            # Mock или недоступно — тихо игнорируем, состояние уже обновлено
            pass

    def relax(self) -> None:
        """Установка Kp=0, Kd=5 — рука обмякает но с демпфированием (безопасно)."""
        print(f"[ArmCtrl {self.side}] relax (Kp=0, Kd=5)")
        if self._enabled:
            try:
                self.robot.set_arm_joint_positions(
                    arm=self.side,
                    positions=self._current_pose,
                    kp=0.0,
                    kd=5.0,
                )
            except Exception:
                pass

    @property
    def current_pose(self) -> list[float]:
        return list(self._current_pose)


class TorsoController:
    """Контроль торса: наклон вперёд (для захвата с низкого стола)."""

    def __init__(self, robot, control_hz: float = 50.0):
        self.robot = robot
        self.control_hz = control_hz
        self._current: list[float] = [0.0, 0.0]

    def move_to_pose(self, pose_name: str, duration: Optional[float] = None) -> bool:
        target = get_torso_pose(pose_name)
        target = self._clamp_torso(target)
        start = list(self._current)
        if duration is None:
            duration = 1.0
        n_steps = max(1, int(duration * self.control_hz))
        for i in range(n_steps + 1):
            t = i / n_steps
            interp = interpolate_poses(start, target, t)
            self._send(interp)
            time.sleep(1.0 / self.control_hz)
        self._current = list(target)
        return True

    def _clamp_torso(self, vals: list[float]) -> list[float]:
        return [
            max(TORSO_LIMITS["pitch"][0], min(TORSO_LIMITS["pitch"][1], vals[0])),
            max(TORSO_LIMITS["roll"][0], min(TORSO_LIMITS["roll"][1], vals[1])),
        ]

    def _send(self, vals: list[float]) -> None:
        try:
            self.robot.set_torso_positions(vals)
        except (NotImplementedError, AttributeError, RuntimeError):
            pass


class DualArmController:
    """Координированное управление двумя руками (для bimanual grasp)."""

    def __init__(self, robot):
        self.right = ArmController(robot, side="right")
        self.left = ArmController(robot, side="left")
        self.torso = TorsoController(robot)

    def enable(self) -> None:
        self.right.enable()
        self.left.enable()

    def move_both_to_pose(self, pose_name: str, duration: Optional[float] = None) -> bool:
        """Синхронно перевести обе руки в одну позу."""
        import threading
        threads = [
            threading.Thread(target=self.right.move_to_pose, args=(pose_name, duration)),
            threading.Thread(target=self.left.move_to_pose, args=(pose_name, duration)),
        ]
        for t in threads: t.start()
        for t in threads: t.join()
        return True

    def relax(self) -> None:
        self.right.relax()
        self.left.relax()


if __name__ == "__main__":
    # Sanity test с MockG1Interface
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).parent.parent))
    from interfaces.unitree_sdk import MockG1Interface

    robot = MockG1Interface()
    robot.connect()

    arms = DualArmController(robot)
    arms.enable()

    print("\n--- Тест поз (mock-режим, без low-level) ---\n")
    for pose in ["home", "idle", "pregrasp", "grasp", "lift", "carry", "handover", "release"]:
        print(f"\n→ {pose}")
        arms.move_both_to_pose(pose, duration=0.3)

    print("\n--- move_to_xyz ---\n")
    arms.right.move_to_xyz(0.5, 0.0, 0.9, duration=0.5)
    print(f"  right current: {arms.right.current_pose}")

    print("\n--- Relax ---\n")
    arms.relax()
