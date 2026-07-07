"""
interfaces/unitree_sdk.py
─────────────────────────
Тонкая обёртка над Unitree G1 SDK2 Python.

Документация: https://github.com/unitreerobotics/unitree_sdk2
Python binding: `pip install unitree_sdk2py`

Кузьмич (G1 EDU Ultimate) exposes:
- Low-level: 43 joint commands (position/velocity/torque, 500 Гц)
- High-level: locomotion policy (ходьба, повороты, вставание)
- Hand: 6 DoF × 2 + тактильные сенсоры RH56DFTP
- Sensors: IMU, RealSense D435, Livox MID-360

Этот модуль изолирует rest of code от деталей SDK.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional


@dataclass
class JointState:
    name: str
    position: float    # rad
    velocity: float    # rad/s
    torque: float      # N·m
    temperature: float # °C


@dataclass
class HandState:
    """Состояние кисти RH56DFTP."""
    finger_positions: list[float]   # 5 пальцев, 0..1
    finger_forces_g: list[float]    # 5 пальцев, в граммах
    temperature: float = 20.0


class UnitreeG1Interface:
    """Обёртка над Unitree SDK2 для G1 EDU Ultimate."""

    def __init__(
        self,
        host: str = "192.168.123.161",   # дефолтный IP G1
        enable_low_level: bool = False,  # осторожно: low-level = прямой control
    ):
        self.host = host
        self.enable_low_level = enable_low_level
        self._client = None  # Lazy import — SDK может быть не установлен на dev-машине

    def connect(self) -> None:
        """Подключение к роботу."""
        try:
            from unitree_sdk2py.core.channel import ChannelFactory
            from unitree_sdk2py.go2.robot.connection import RobotClient
        except ImportError as e:
            raise RuntimeError(
                "unitree_sdk2py не установлен. Установи: "
                "pip install unitree_sdk2py"
            ) from e
        # TODO: реальная инициализация клиента
        print(f"[UnitreeG1] connecting to {self.host}...")

    # ─── Locomotion (high-level) ─────────────────────────────────────────

    def stand_up(self) -> None:
        """Встать из положения сидя/лежа."""
        # TODO: отправить high-level команду
        print("[UnitreeG1] stand_up")

    def sit_down(self) -> None:
        """Сесть."""
        print("[UnitreeG1] sit_down")

    def walk_to(self, x: float, y: float, theta: float = 0.0) -> None:
        """
        Идти в точку (x, y) в метрах, theta — конечный поворот в радианах.
        Использует предобученную RL LOCO policy.
        """
        print(f"[UnitreeG1] walk_to ({x:.2f}, {y:.2f}, θ={theta:.2f})")

    def stop_walking(self) -> None:
        """Остановиться."""
        print("[UnitreeG1] stop_walking")

    # ─── Manipulation (low-level) ────────────────────────────────────────

    def get_joint_states(self) -> list[JointState]:
        """Чтение текущих позиций всех 43 суставов."""
        # TODO: реальный опрос
        return []

    def set_arm_joint_positions(
        self,
        arm: str,                          # "left" | "right"
        positions: list[float],            # 5 позиций плечо→запястье
        velocities: Optional[list[float]] = None,
        timeout: float = 2.0,
    ) -> None:
        """Команда на руку."""
        print(f"[UnitreeG1] set_arm {arm}: {positions}")

    def set_hand_position(self, hand: str, position: float) -> None:
        """
        Открыть/закрыть кисть.
        position: 0.0 = открыта, 1.0 = полностью закрыта
        """
        print(f"[UnitreeG1] set_hand {hand}: pos={position:.2f}")

    def get_hand_state(self, hand: str) -> HandState:
        """Чтение состояния кисти RH56DFTP (позиции + тактильные сенсоры)."""
        # TODO: реальный опрос через SDK2 или RS-485
        return HandState(
            finger_positions=[0.0] * 5,
            finger_forces_g=[0.0] * 5,
        )

    # ─── Sensors ─────────────────────────────────────────────────────────

    def get_imu(self) -> dict:
        """IMU: orientation, angular_velocity, linear_acceleration."""
        return {"orientation": [0, 0, 0, 1], "gyro": [0, 0, 0], "accel": [0, 0, 9.8]}

    def get_lidar_scan(self) -> dict:
        """Livox MID-360 point cloud."""
        # TODO: подключить livox_sdk2
        return {"points": [], "ranges_m": []}

    # ─── Safety ──────────────────────────────────────────────────────────

    def emergency_stop(self) -> None:
        """Аварийная остановка: все моторы в режим удержания позиции."""
        print("[UnitreeG1] E-STOP!")

    def release_motors(self) -> None:
        """Отпустить все моторы (робот обмякает). Только для emergency recovery."""
        print("[UnitreeG1] release all motors")


# ─── Mock для тестов без реального робота ────────────────────────────────

class MockG1Interface(UnitreeG1Interface):
    """Та же сигнатура, но ничего не делает — для dev/test."""

    def connect(self) -> None:
        print(f"[MockG1] would connect to {self.host}")

    def walk_to(self, x, y, theta=0.0):
        print(f"[MockG1] would walk to ({x:.2f}, {y:.2f}, θ={theta:.2f})")


if __name__ == "__main__":
    # На dev-машине используем Mock
    robot = MockG1Interface()
    robot.connect()
    robot.stand_up()
    robot.walk_to(1.0, 0.0, 0.0)
    robot.set_hand_position("right", 0.5)
    hand = robot.get_hand_state("right")
    print(f"Hand forces: {hand.finger_forces_g}")
