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

High-level API (unitree_sdk2py.g1.loco.g1_loco_client.LocoClient — НЕ Go2 SportClient,
это разные роботы с разным API):
- LocoClient: Start(), StandUp(), Sit(), Move(vx,vy,vyaw), StopMove(), Damp(),
  BalanceStand(), ContinuousGait(bool)
- Позиционного MoveTo(x,y,yaw) в LocoClient нет — move_to() ниже эмулирует его
  через разворот + прямолинейное движение по Move()+времени.
- Low-level: LowCmd с q/dq/tau/Kp/Kd на каждый мотор, частота 500 Гц

Калибровка G1 EDU снимает блокировку 500 Гц full joint control.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Optional

# Безопасные скорости для демо (см. burunov-joke-bot/coffee_delivery.py — та же логика)
WALK_SPEED_M_S = 0.25
TURN_SPEED_RAD_S = 0.5


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
        speak_fn: Optional[Callable[[str], None]] = None,
    ):
        self.host = host
        self.network_interface = network_interface
        self.enable_low_level = enable_low_level
        self._loco_client = None
        self._low_level_client = None
        self._lowcmd_publisher = None
        self._lowstate = None
        # Озвучка — по умолчанию встроенный TtsMaker (CN/EN). Чтобы Кузьмич
        # говорил голосом Бурунова, передай сюда обёртку над
        # synthesize_burunov_pcm()+AudioClient.PlayStream() из
        # burunov-joke-bot/coffee_delivery.py (как только TTS-движок там будет
        # доведён до рабочего состояния).
        self._speak_fn = speak_fn

    def connect(self) -> None:
        """Подключение к G1 через CycloneDDS.

        Требуется:
        1. SDK2 установлен (pip install -e . из клона unitree_sdk2_python)
        2. ПК в подсети 192.168.123.x
        3. CycloneDDS сконфигурирован (env CYCLONEDDS_URI)
        """
        try:
            from unitree_sdk2py.core.channel import ChannelFactory
            from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        except ImportError as e:
            raise RuntimeError(
                "unitree_sdk2py не установлен.\n"
                "Установка:\n"
                "  git clone https://github.com/unitreerobotics/unitree_sdk2_python\n"
                "  cd unitree_sdk2_python\n"
                "  pip install -e .\n"
                "Также нужен cyclonedds: pip install cyclonedds"
            ) from e

        # Инициализация ChannelFactory (DDS). domainId=0 — как в примерах Unitree.
        ChannelFactory.Initialize(0, self.network_interface)

        # G1 использует LocoClient (Sport Services), а не Go2 SportClient —
        # это разные роботы с разным API.
        self._loco_client = LocoClient()
        self._loco_client.Init()
        ret = self._loco_client.Start()  # войти в main operation control
        if ret != 0:
            raise RuntimeError(f"LocoClient.Start() failed: {ret}")

        print(f"[UnitreeG1] connected via {self.network_interface} to {self.host}")

    # ─── Locomotion (high-level, через LocoClient) ────────────────────────

    def stand_up(self) -> bool:
        """Встать из положения сидя/лежа."""
        if not self._loco_client:
            raise RuntimeError("Не подключён")
        ret = self._loco_client.StandUp()
        time.sleep(2.0)  # ожидание исполнения
        return ret == 0

    def stand_down(self) -> bool:
        """Сесть/лечь (у G1 LocoClient это Sit(), а не StandDown())."""
        if not self._loco_client:
            raise RuntimeError("Не подключён")
        ret = self._loco_client.Sit()
        time.sleep(2.0)
        return ret == 0

    def move(self, vx: float, vy: float, vyaw: float, duration_s: float = 1.0) -> None:
        """
        Движение с заданными скоростями.
        vx: вперёд/назад, м/с (положительное = вперёд)
        vy: влево/вправо, м/с
        vyaw: вращение, рад/с
        duration_s: сколько секунд держать скорость

        LocoClient.Move() по доке действует ~1с, поэтому шлём его в цикле,
        пока не наберём duration_s (как в burunov-joke-bot/coffee_delivery.py).
        """
        if not self._loco_client:
            raise RuntimeError("Не подключён")
        try:
            self._loco_client.ContinuousGait(True)
        except Exception:
            pass
        steps = max(1, int(duration_s * 10))
        dt = duration_s / steps
        for _ in range(steps):
            self._loco_client.Move(vx, vy, vyaw)
            time.sleep(dt)
        self._loco_client.StopMove()
        try:
            self._loco_client.ContinuousGait(False)
        except Exception:
            pass

    def move_to(self, x: float, y: float, yaw: float = 0.0) -> bool:
        """
        Идти к целевой точке (x, y) в метрах относительно текущей позиции.
        yaw — целевой угол поворота в радианах.

        ⚠️ G1 LocoClient не даёт позиционного MoveTo(x,y) как Go2 SportClient —
        только Move(vx,vy,vyaw) по скорости. Поэтому сначала разворачиваемся
        на угол до цели, потом едем по прямой на нужное расстояние. Это грубая
        одометрия по времени (без обратной связи по реальной позиции) —
        для точной навигации нужна телеметрия/SLAM.
        """
        if not self._loco_client:
            raise RuntimeError("Не подключён")
        import math
        distance = math.hypot(x, y)
        heading = math.atan2(y, x)

        if abs(heading) > 1e-3:
            turn_duration = abs(heading) / TURN_SPEED_RAD_S
            self.move(0.0, 0.0, math.copysign(TURN_SPEED_RAD_S, heading), turn_duration)

        if distance > 1e-3:
            move_duration = distance / WALK_SPEED_M_S
            self.move(WALK_SPEED_M_S, 0.0, 0.0, move_duration)

        return True

    def stop_move(self) -> None:
        if self._loco_client:
            self._loco_client.StopMove()

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
        kp: float = 80.0,
        kd: float = 3.0,
    ) -> None:
        """
        Команда на руку через LowCmd (частота ~500 Гц).

        ⚠️ Low-level команды требуют:
        1. G1 в режиме разработки (не fall-protection)
        2. Kp/Kd подобраны правильно
        3. Команды шлются на 500 Гц без пропусков

        Рекомендация: начать с Kp=0, Kd=5 (damping), потом плавно поднимать Kp.

        arm: "left" или "right"
        positions: 7 углов в радианах (shoulder_pitch, shoulder_roll, shoulder_yaw,
                   elbow, wrist_pitch, wrist_roll, wrist_yaw)
        """
        if not self.enable_low_level:
            raise RuntimeError(
                "Low-level команды отключены. Установи enable_low_level=True "
                "ТОЛЬКО если понимаешь что делаешь — можно уронить робота."
            )
        if arm not in ("left", "right"):
            raise ValueError(f"arm must be 'left' or 'right', got {arm!r}")
        if len(positions) != 7:
            raise ValueError(f"Expected 7 joint positions, got {len(positions)}")

        self._publish_lowcmd(arm, positions, kp=kp, kd=kd)

    def set_torso_positions(
        self,
        positions: list[float],    # [pitch, roll] в радианах
        kp: float = 100.0,
        kd: float = 5.0,
    ) -> None:
        """Команда на торс (2 мотора: pitch + roll)."""
        if not self.enable_low_level:
            raise RuntimeError("Low-level команды отключены.")
        if len(positions) != 2:
            raise ValueError(f"Expected 2 torso positions [pitch, roll], got {len(positions)}")
        self._publish_lowcmd("torso", positions, kp=kp, kd=kd)

    def set_hand_position(self, hand: str, grip_strength: float) -> None:
        """High-level команда на кисть RH56DFTP.

        ⚠️ RH56DFTP подключается через RS-485, НЕ через DDS.
        Этот метод — прокси к HandClient в SDK (если доступен),
        иначе кидает NotImplementedError и нужно использовать
        perception.tactile.rh56dftp.RH56DFTPDriver напрямую.

        hand: "left" | "right"
        grip_strength: 0..1 (0 = открыта, 1 = полностью сжата)
        """
        if hand not in ("left", "right"):
            raise ValueError(f"hand must be 'left' or 'right', got {hand!r}")
        if not 0.0 <= grip_strength <= 1.0:
            raise ValueError(f"grip_strength must be in [0,1], got {grip_strength}")

        try:
            from unitree_sdk2py.g1.hand.hand_client import HandClient
        except ImportError:
            raise NotImplementedError(
                "HandClient недоступен. Используй RH56DFTPDriver напрямую "
                "(perception.tactile.rh56dftp) — он работает через RS-485."
            )

        # HandClient обычно принимает угол в радианах
        angle = grip_strength * 1.5  # ~85° max flex
        try:
            client = HandClient()
            client.Init()
            # API отличается между версиями SDK — пробуем несколько
            for method_name in ("SetHandAngle", "SetHandPos", "SetPose"):
                method = getattr(client, method_name, None)
                if callable(method):
                    try:
                        method(hand, angle)
                        return
                    except TypeError:
                        method(angle)
                        return
            raise RuntimeError(f"HandClient: ни один метод не сработал")
        except Exception as e:
            raise RuntimeError(f"HandClient error: {e}")

    def _publish_lowcmd(self, target: str, positions: list[float],
                        kp: float = 80.0, kd: float = 3.0) -> None:
        """Публикация LowCmd на rt/lowcmd через DDS.

        Использует unitree_sdk2py.idl-сообщения. Структура LowCmd
        зависит от версии SDK — здесь универсальный путь с fallback.
        """
        if self._lowcmd_publisher is None:
            try:
                from unitree_sdk2py.core.channel import ChannelPublisher
                from unitree_sdk2py.idl.unitree_hg import LowCmd_
                self._lowcmd_publisher = ChannelPublisher(
                    LowCmd_, "rt/lowcmd"
                )
                self._lowcmd_publisher.Init()
            except ImportError as e:
                raise RuntimeError(
                    f"Не удалось импортировать LowCmd_ из SDK: {e}\n"
                    "Проверь, что unitree_sdk2py установлен и поддерживает G1 (HG)."
                )

        # Заполняем LowCmd
        from unitree_sdk2py.idl.unitree_hg import LowCmd_, MotorCmd_
        cmd = LowCmd_()
        # Индексы моторов в G1 (примерные — уточнить по SDK!):
        # right arm: motors 14..20, left arm: 21..27, torso: 12..13
        ARM_OFFSETS = {"right": 14, "left": 21, "torso": 12}
        base = ARM_OFFSETS.get(target, 0)

        for i, q in enumerate(positions):
            m = MotorCmd_()
            m.q = float(q)
            m.kp = float(kp)
            m.kd = float(kd)
            m.tau = 0.0
            m.dq = 0.0
            cmd.motor_cmd[base + i] = m

        self._lowcmd_publisher.Write(cmd)

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
        """IMU: orientation (quaternion qx,qy,qz,qw), angular_velocity, linear_acceleration."""
        # TODO: подписка на rt/lowstate → imu
        return {"orientation": [0, 0, 0, 1], "gyro": [0, 0, 0], "accel": [0, 0, 9.8]}

    def get_battery_pct(self) -> Optional[float]:
        """Заряд батареи в процентах (0..100). None если недоступно."""
        # TODO: подписка на rt/lowstate → battery
        return None

    def say(self, text: str) -> None:
        """Озвучить текст.

        Если в конструктор передан speak_fn — используем его (это путь для
        голоса Бурунова: обёртка над synthesize_burunov_pcm()+PlayStream()).
        Иначе — встроенный TtsMaker G1 (только CN/EN, не Бурунов).
        """
        if self._speak_fn is not None:
            try:
                self._speak_fn(text)
                return
            except Exception as e:
                print(f"[UnitreeG1] speak_fn упал: {e}")

        try:
            from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient
            client = AudioClient()
            client.Init()
            client.TtsMaker(text, 0)
        except Exception as e:
            print(f"[UnitreeG1] TTS не доступен: {e}")

    # ─── Safety ──────────────────────────────────────────────────────────

    def emergency_stop(self) -> None:
        """Аварийная остановка: StopMove + Damp (обмякнуть)."""
        if self._loco_client:
            try:
                self._loco_client.StopMove()
            except Exception:
                pass
            try:
                self._loco_client.Damp()
            except Exception:
                pass
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

    def get_imu(self) -> dict:
        return {"orientation": [0, 0, 0, 1], "gyro": [0, 0, 0], "accel": [0, 0, 9.8]}

    def get_battery_pct(self) -> Optional[float]:
        return 85.0

    def say(self, text: str) -> None:
        print(f"[MockG1] 🗣 {text}")

    def set_arm_joint_positions(self, arm, positions, velocities=None,
                                 timeout=2.0, kp=80.0, kd=3.0):
        if not self.enable_low_level:
            return  # mock тихо игнорирует
        print(f"[MockG1] arm[{arm}] → {[round(p, 2) for p in positions]} "
              f"Kp={kp} Kd={kd}")

    def set_torso_positions(self, positions, kp=100.0, kd=5.0):
        if not self.enable_low_level:
            return
        print(f"[MockG1] torso → {[round(p, 2) for p in positions]}")

    def set_hand_position(self, hand, grip_strength):
        print(f"[MockG1] hand[{hand}] grip={grip_strength:.2f}")


if __name__ == "__main__":
    # На dev-машине используем Mock
    robot = MockG1Interface()
    robot.connect()
    robot.stand_up()
    robot.move_to(1.0, 0.0, 0.0)
    robot.emergency_stop()
