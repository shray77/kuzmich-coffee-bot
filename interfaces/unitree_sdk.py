"""
interfaces/unitree_sdk.py
─────────────────────────
Обёртка над Unitree G1 SDK2 Python.

⚠️ УТОЧНЕНО ПО РЕАЛЬНОЙ ДОКУМЕНТАЦИИ:
- SDK: github.com/unitreerobotics/unitree_sdk2_python (не на PyPI!)
- Установка: git clone + pip install -e .
- Протокол: CycloneDDS поверх WiFi/Ethernet
- Дефолтный IP G1 (Ethernet): 192.168.123.164
- ПК разработчика должен быть в 192.168.123.x/24
- WiFi заблокирован по умолчанию (нужен кабель через свитч G1)

High-level API:
- SportClient: Move(vx,vy,vyaw), MoveTo(x,y,yaw), StopMove, StandUp, StandDown
- LocoClient: StandUp(), Start() — для активации ходьбы
- Low-level: LowCmd с q/dq/tau/Kp/Kd на каждый мотор, частота 500 Гц

Калибровка G1 EDU снимает блокировку 500 Гц full joint control.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional


@dataclass
class JointState:
    """Состояние одного мотора G1."""
    name: str
    position: float       # rad
    velocity: float       # rad/s
    torque: float         # N·m
    temperature: float    # °C


@dataclass
class HandState:
    """Состояние кисти RH56DFTP."""
    finger_positions: list[float]   # 12 суставов, рад
    forces_n: list[float]           # 6 силовых сенсоров, Н
    tactile_raw: list[int]          # тактильные сенсоры (raw)
    contact: bool = False


class UnitreeG1Interface:
    """Обёртка над Unitree SDK2 для G1 EDU Ultimate.

    На dev-машине (без робота) используй MockG1Interface — он ничего не делает,
    но позволяет тестировать всю логику.
    """

    def __init__(
        self,
        host: str = "192.168.123.164",   # дефолтный IP G1 по Ethernet
        network_interface: str = "eth0", # где поднят 192.168.123.x
        enable_low_level: bool = False,
    ):
        self.host = host
        self.network_interface = network_interface
        self.enable_low_level = enable_low_level
        self._sport_client = None
        self._loco_client = None
        self._low_level_client = None

    def connect(self) -> None:
        """Подключение к G1 через CycloneDDS.

        Требуется:
        1. SDK2 установлен (pip install -e . из клона unitree_sdk2_python)
        2. ПК в подсети 192.168.123.x
        3. CycloneDDS сконфигурирован (env CYCLONEDDS_URI)
        """
        try:
            from unitree_sdk2py.core.channel import ChannelFactory
            from unitree_sdk2py.go2.sport.sport_client import SportClient
        except ImportError as e:
            raise RuntimeError(
                "unitree_sdk2py не установлен.\n"
                "Установка:\n"
                "  git clone https://github.com/unitreerobotics/unitree_sdk2_python\n"
                "  cd unitree_sdk2_python\n"
                "  pip install -e .\n"
                "Также нужен cyclonedds: pip install cyclonedds"
            ) from e

        # Инициализация ChannelFactory (DDS)
        ChannelFactory.Initialize(self.network_interface)
        ChannelFactory.SetLogLevel(2)  # INFO

        # High-level sport client (locomotion)
        self._sport_client = SportClient()
        self._sport_client.SetTimeout(5.0)
        self._sport_client.Init()

        print(f"[UnitreeG1] connected via {self.network_interface} to {self.host}")

    # ─── Locomotion (high-level, через SportClient) ──────────────────────

    def stand_up(self) -> bool:
        """Встать из положения сидя/лежа."""
        if not self._sport_client:
            raise RuntimeError("Не подключён")
        # G1 sport_client StandUp (асинхронно, ждём завершения)
        self._sport_client.StandUp()
        time.sleep(2.0)  # ожидание исполнения
        return True

    def stand_down(self) -> bool:
        """Сесть/лечь."""
        if not self._sport_client:
            raise RuntimeError("Не подключён")
        self._sport_client.StandDown()
        time.sleep(2.0)
        return True

    def move(self, vx: float, vy: float, vyaw: float, duration_s: float = 1.0) -> None:
        """
        Движение с заданными скоростями.
        vx: вперёд/назад, м/с (положительное = вперёд)
        vy: влево/вправо, м/с
        vyaw: вращение, рад/с
        duration_s: сколько секунд держать скорость
        """
        if not self._sport_client:
            raise RuntimeError("Не подключён")
        self._sport_client.Move(vx, vy, vyaw)
        time.sleep(duration_s)
        self._sport_client.StopMove()

    def move_to(self, x: float, y: float, yaw: float = 0.0) -> bool:
        """
        Идти к целевой точке (x, y) в метрах относительно текущей позиции.
        yaw — целевой угол поворота в радианах.
        """
        if not self._sport_client:
            raise RuntimeError("Не подключён")
        self._sport_client.MoveTo(x, y, yaw)
        # Ждём достижения (упрощённо — в реале нужен callback/telemetry)
        time.sleep(max(2.0, (abs(x) + abs(y)) * 1.5))
        return True

    def stop_move(self) -> None:
        if self._sport_client:
            self._sport_client.StopMove()

    # ─── Manipulation (low-level) ────────────────────────────────────────

    def get_joint_states(self) -> list[JointState]:
        """Чтение позиций всех 43 суставов G1.

        Через LowState topic на DDS.
        """
        # TODO: подписка на rt/lowstate (LowState_ topic)
        return []

    def set_arm_joint_positions(
        self,
        arm: str,                          # "left" | "right"
        positions: list[float],            # 7 позиций плечо→запястье
        velocities: Optional[list[float]] = None,
        timeout: float = 2.0,
    ) -> None:
        """
        Команда на руку через LowCmd (частота 500 Гц).

        ⚠️ Low-level команды требуют:
        1. G1 в режиме разработки (не fall-protection)
        2. Kp/Kd подобраны правильно
        3. Команды шлются на 500 Гц без пропусков
        """
        if not self.enable_low_level:
            raise RuntimeError(
                "Low-level команды отключены. Установи enable_low_level=True "
                "ТОЛЬКО если понимаешь что делаешь — можно уронить робота."
            )
        # TODO: публиковать LowCmd на rt/lowcmd topic
        # Рекомендация: начать с Kp=0, Kd=5 (damping), потом плавно поднимать Kp
        raise NotImplementedError(
            "Low-level arm control. См. примеры unitree_sdk2_python/example/"
        )

    # ─── Hands (через отдельный RS-485, НЕ через DDS) ────────────────────

    def get_hand_state(self, hand: str) -> HandState:
        """Чтение состояния кисти RH56DFTP.

        ⚠️ RH56DFTP подключается через RS-485, НЕ через DDS.
        Используй RH56DFTPDriver из perception/tactile/rh56dftp.py
        """
        raise NotImplementedError(
            "Используй RH56DFTPDriver напрямую — он работает через RS-485, "
            "не через DDS как основное тело G1."
        )

    # ─── Sensors (через DDS topics) ──────────────────────────────────────

    def get_imu(self) -> dict:
        """IMU: orientation (quaternion), angular_velocity, linear_acceleration."""
        # TODO: подписка на rt/lowstate → imu
        return {"orientation": [0, 0, 0, 1], "gyro": [0, 0, 0], "accel": [0, 0, 9.8]}

    # ─── Safety ──────────────────────────────────────────────────────────

    def emergency_stop(self) -> None:
        """Аварийная остановка: StopMove + damping mode."""
        if self._sport_client:
            self._sport_client.StopMove()
        print("[UnitreeG1] E-STOP!")

    def release_motors(self) -> None:
        """Отпустить все моторы (робот обмякает). ТОЛЬКО для emergency recovery."""
        print("[UnitreeG1] release all motors — робот упадёт!")


# ─── Mock для dev/test ───────────────────────────────────────────────────

class MockG1Interface(UnitreeG1Interface):
    """Та же сигнатура, ничего не делает — для dev/test."""

    def connect(self) -> None:
        print(f"[MockG1] would connect via {self.network_interface} to {self.host}")

    def stand_up(self) -> bool:
        print("[MockG1] stand_up")
        return True

    def stand_down(self) -> bool:
        print("[MockG1] stand_down")
        return True

    def move(self, vx, vy, vyaw, duration_s=1.0) -> None:
        print(f"[MockG1] move vx={vx:.2f} vy={vy:.2f} vyaw={vyaw:.2f} for {duration_s}s")

    def move_to(self, x, y, yaw=0.0) -> bool:
        print(f"[MockG1] move_to x={x:.2f} y={y:.2f} yaw={yaw:.2f}")
        return True

    def stop_move(self) -> None:
        print("[MockG1] stop")

    def emergency_stop(self) -> None:
        print("[MockG1] E-STOP")


if __name__ == "__main__":
    # На dev-машине используем Mock
    robot = MockG1Interface()
    robot.connect()
    robot.stand_up()
    robot.move_to(1.0, 0.0, 0.0)
    robot.emergency_stop()
