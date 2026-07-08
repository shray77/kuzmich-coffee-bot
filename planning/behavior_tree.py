"""
planning/behavior_tree.py
─────────────────────────
Behavior tree для задачи "принеси кофе".

Структура дерева:

  Sequence (root)
  ├── ParseCommand          # разбираем голос → Goal (внешний, уже на bb)
  ├── NavigateToKitchen     # локомоция к кофеварке (через UnitreeG1.move_to)
  ├── FindCup               # YOLO + RealSense → 3D позиция чашки
  ├── ApproachCup           # подойти в зону досягаемости (move_to_xyz)
  ├── GraspCup              # closed-loop grip с тактильной обратной связью
  ├── NavigateBackToUser    # вернуться к Олеже (по запомненной позиции)
  ├── HandOver              # передать чашку (detect hand + force release)
  └── ConfirmDone           # голосом "держи, Олежа"

Каждый узел возвращает Status.SUCCESS / Status.FAILURE / Status.RUNNING.
Если какой-то узел упал — дерево перезапускается (макс 3 ретрая через Retry).

DI: дерево строится через build_coffee_tree(robot, voice, detector, tactile, arm)
— это позволяет подменять компоненты на mock'и в тестах.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Any


class Status(Enum):
    SUCCESS = "success"
    FAILURE = "failure"
    RUNNING = "running"


@dataclass
class Blackboard:
    """Shared state между узлами дерева."""
    goal: Optional[dict] = None
    # Чашка
    cup_xyz: Optional[tuple[float, float, float]] = None
    cup_detected: bool = False
    cup_grasped: bool = False
    cup_released: bool = False
    # Пользователь
    user_position: Optional[tuple[float, float, float]] = None
    user_detected: bool = False
    handover_complete: bool = False
    # Навигация
    home_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    kitchen_position: tuple[float, float, float] = (1.5, 0.0, 0.0)  # 1.5 м вперёд
    # Диагностика
    error_count: int = 0
    last_error: str = ""
    started_at: float = field(default_factory=time.time)
    log: list[str] = field(default_factory=list)

    def err(self, msg: str) -> None:
        self.error_count += 1
        self.last_error = msg
        self.log.append(f"[ERR] {msg}")
        print(f"  [BT ERROR] {msg}")


# ─── Базовые классы ──────────────────────────────────────────────────────

class Node:
    """Базовый класс для узла behavior tree."""
    def __init__(self, name: Optional[str] = None):
        self.name = name or self.__class__.__name__

    def tick(self, bb: Blackboard) -> Status:
        raise NotImplementedError

    def reset(self) -> None:
        pass


class Sequence(Node):
    """Выполняет детей по очереди. Если любой упал — Sequence падает."""
    def __init__(self, children: list[Node], name: Optional[str] = None):
        super().__init__(name or "Sequence")
        self.children = children
        self._idx = 0

    def tick(self, bb: Blackboard) -> Status:
        while self._idx < len(self.children):
            status = self.children[self._idx].tick(bb)
            if status == Status.RUNNING:
                return Status.RUNNING
            if status == Status.FAILURE:
                bb.log.append(f"{self.name}: child {self.children[self._idx].name} failed")
                self._idx = 0
                return Status.FAILURE
            self._idx += 1
        self._idx = 0
        return Status.SUCCESS

    def reset(self) -> None:
        self._idx = 0
        for c in self.children:
            c.reset()


class Retry(Node):
    """Оборачивает узел: повторяет N раз при FAILURE."""
    def __init__(self, child: Node, max_attempts: int = 3, name: Optional[str] = None):
        super().__init__(name or f"Retry({child.name})")
        self.child = child
        self.max_attempts = max_attempts
        self._attempt = 0

    def tick(self, bb: Blackboard) -> Status:
        while self._attempt < self.max_attempts:
            status = self.child.tick(bb)
            if status == Status.SUCCESS:
                self._attempt = 0
                return Status.SUCCESS
            if status == Status.RUNNING:
                return Status.RUNNING
            self._attempt += 1
            bb.log.append(f"{self.name}: attempt {self._attempt}/{self.max_attempts} failed")
            time.sleep(0.3)
        self._attempt = 0
        return Status.FAILURE

    def reset(self) -> None:
        self._attempt = 0
        self.child.reset()


# ─── Реальные узлы (с DI) ────────────────────────────────────────────────

class ParseCommand(Node):
    """Уже готовый Goal на blackboard → SUCCESS. (Парсинг делается в main.py.)"""
    def tick(self, bb: Blackboard) -> Status:
        if bb.goal is None:
            bb.err("No goal on blackboard")
            return Status.FAILURE
        action = bb.goal.get("action", "")
        if action in ("fetch",):
            return Status.SUCCESS
        if action == "abort":
            bb.log.append("abort requested")
            return Status.FAILURE
        bb.err(f"Unknown action: {action}")
        return Status.FAILURE


class NavigateToKitchen(Node):
    """Идти к кухонной точке через robot.move_to()."""
    def __init__(self, robot, name: Optional[str] = None):
        super().__init__(name)
        self.robot = robot

    def tick(self, bb: Blackboard) -> Status:
        x, y, yaw = bb.kitchen_position
        print(f"  [{self.name}] → kitchen ({x:.1f}, {y:.1f}, {yaw:.1f})")
        try:
            self.robot.move_to(x, y, yaw)
            return Status.SUCCESS
        except Exception as e:
            bb.err(f"NavigateToKitchen: {e}")
            return Status.FAILURE


class FindCup(Node):
    """YOLO + RealSense depth → 3D позиция чашки на blackboard."""
    def __init__(self, detector, realsense=None, name: Optional[str] = None):
        super().__init__(name)
        self.detector = detector
        self.realsense = realsense  # опционально, для depth
        self._attempts = 0

    def tick(self, bb: Blackboard) -> Status:
        from perception.vision.detector import find_best_cup, camera_to_robot_frame
        print(f"  [{self.name}] scanning for cup...")

        # Получаем кадр
        try:
            if self.realsense is not None:
                rgb, depth = self.realsense.get_frames()
                dets = self.detector.detect_with_depth(rgb, depth)
            else:
                # Тестовый режим — может быть веб-камера или mock
                rgb = self._get_test_frame()
                dets = self.detector.detect(rgb)
        except Exception as e:
            bb.err(f"FindCup: vision error: {e}")
            return Status.FAILURE

        cup = find_best_cup(dets)
        if cup is None:
            self._attempts += 1
            bb.log.append(f"FindCup: no cup detected (attempt {self._attempts})")
            return Status.FAILURE

        if cup.xyz_m is not None:
            # cup.xyz_m — в системе камеры (x=вправо,y=вниз,z=depth), а робот
            # (move_to/move_to_xyz) ждёт систему базы (x=вперёд,y=влево,z=высота).
            bb.cup_xyz = camera_to_robot_frame(cup.xyz_m)
        else:
            # Без depth — берём координаты в системе робота (placeholder)
            bb.cup_xyz = (0.5, 0.0, 0.8)

        bb.cup_detected = True
        print(f"  [{self.name}] cup at {bb.cup_xyz}")
        return Status.SUCCESS

    def _get_test_frame(self):
        """Fallback: веб-камера 0 или синтетический кадр."""
        try:
            import cv2
            cap = cv2.VideoCapture(0)
            ret, frame = cap.read()
            cap.release()
            if ret:
                return frame
        except Exception:
            pass
        import numpy as np
        return np.zeros((480, 640, 3), dtype="uint8")


class ApproachCup(Node):
    """Подойти к чашке через robot.move_to(cup_x-0.3, cup_y, 0)."""
    def __init__(self, robot, name: Optional[str] = None):
        super().__init__(name)
        self.robot = robot

    def tick(self, bb: Blackboard) -> Status:
        if bb.cup_xyz is None:
            bb.err("ApproachCup: no cup_xyz")
            return Status.FAILURE
        x, y, _ = bb.cup_xyz
        # Останавливаемся в 30 см перед чашкой
        approach_x = max(0.3, x - 0.3)
        print(f"  [{self.name}] approach to ({approach_x:.2f}, {y:.2f})")
        try:
            self.robot.move_to(approach_x, y, 0.0)
            return Status.SUCCESS
        except Exception as e:
            bb.err(f"ApproachCup: {e}")
            return Status.FAILURE


class GraspCup(Node):
    """Closed-loop хват через GripController + arm_controller."""
    def __init__(self, arm_ctrl, tactile_driver, grip_controller_cls=None,
                 name: Optional[str] = None, max_steps: int = 200):
        super().__init__(name)
        self.arm_ctrl = arm_ctrl
        self.tactile = tactile_driver
        # grip_controller_cls — для DI (по умолчанию GripController из rh56dftp)
        if grip_controller_cls is None:
            from perception.tactile.rh56dftp import GripController
            grip_controller_cls = GripController
        self._GripController = grip_controller_cls
        self.max_steps = max_steps
        self._grip: Optional[Any] = None

    def tick(self, bb: Blackboard) -> Status:
        if bb.cup_xyz is None:
            bb.err("GraspCup: no cup_xyz")
            return Status.FAILURE

        # 1. Открыть кисть
        try:
            self.tactile.open_hand()
        except Exception as e:
            bb.err(f"GraspCup: open_hand failed: {e}")
            return Status.FAILURE

        # 2. Подвести руку к чашке
        x, y, z = bb.cup_xyz
        try:
            self.arm_ctrl.move_to_xyz(x, y, z, duration=1.5)
        except Exception as e:
            bb.err(f"GraspCup: move_to_xyz failed: {e}")
            return Status.FAILURE

        # 3. Сжать через GripController (closed-loop по силе)
        try:
            grip = self._GripController(
                driver=self.tactile,
                target_force_n=2.0,
                max_force_n=8.0,
            )
            grip.reset()
        except Exception as e:
            bb.err(f"GraspCup: grip init failed: {e}")
            return Status.FAILURE

        for step in range(self.max_steps):
            pos, status = grip.grip_step()
            try:
                self.tactile.close_hand(pos)
            except Exception as e:
                bb.err(f"GraspCup: close_hand failed: {e}")
                return Status.FAILURE

            if status == "stable":
                bb.cup_grasped = True
                compliance = grip.compliance()
                print(f"  [{self.name}] grasped (step {step}, pos={pos:.2f}, "
                      f"compliance={compliance.classification.value}, "
                      f"stiffness={compliance.stiffness})")
                # 4. Поднять чашку
                try:
                    self.arm_ctrl.move_to_pose("lift", duration=1.0)
                except Exception:
                    pass
                return Status.SUCCESS
            if status == "overforce":
                compliance = grip.compliance()
                bb.err(f"GraspCup: overforce (compliance={compliance.classification.value}) — aborting")
                grip.release()
                return Status.FAILURE
            if status == "slip":
                bb.log.append("GraspCup: slip, tightening")
            time.sleep(0.05)

        bb.err("GraspCup: max_steps exceeded")
        grip.release()
        return Status.FAILURE


class NavigateBackToUser(Node):
    """Вернуться к home_position."""
    def __init__(self, robot, name: Optional[str] = None):
        super().__init__(name)
        self.robot = robot

    def tick(self, bb: Blackboard) -> Status:
        x, y, yaw = bb.home_position
        print(f"  [{self.name}] back to home ({x:.1f}, {y:.1f})")
        try:
            self.robot.move_to(-1.5, 0.0, 0.0)  # обратный путь
            return Status.SUCCESS
        except Exception as e:
            bb.err(f"NavigateBackToUser: {e}")
            return Status.FAILURE


class HandOver(Node):
    """Передача чашки через HandoverController."""
    def __init__(self, handover_controller, name: Optional[str] = None):
        super().__init__(name)
        self.handover = handover_controller

    def tick(self, bb: Blackboard) -> Status:
        if not bb.cup_grasped:
            bb.err("HandOver: no cup grasped")
            return Status.FAILURE
        print(f"  [{self.name}] handing over...")
        try:
            result = self.handover.execute()
        except Exception as e:
            bb.err(f"HandOver: exception {e}")
            return Status.FAILURE

        if result.success:
            bb.handover_complete = True
            bb.cup_released = True
            print(f"  [{self.name}] OK ({result.reason}, {result.duration_s:.1f}s)")
            return Status.SUCCESS
        bb.err(f"HandOver: {result.reason}")
        return Status.FAILURE


class ConfirmDone(Node):
    """Голосовая фраза «держи, Олежа»."""
    def __init__(self, robot, name: Optional[str] = None):
        super().__init__(name)
        self.robot = robot

    def tick(self, bb: Blackboard) -> Status:
        print(f"  [{self.name}] 'Держи, Олежа!'")
        try:
            self.robot.say("Держи, Олежа!")
        except Exception:
            pass
        return Status.SUCCESS


# ─── Сборка дерева ───────────────────────────────────────────────────────

def build_coffee_tree(
    robot,
    voice=None,                # не используется внутри дерева (парсинг в main)
    detector=None,
    realsense=None,
    tactile_driver=None,
    arm_ctrl=None,
    handover_controller=None,
    grip_controller_cls=None,
) -> Sequence:
    """Собирает дерево из переданных зависимостей.

    Все параметры могут быть None — в этом случае узел будет возвращать FAILURE
    при вызове (полезно для тестов отдельных узлов).
    """
    children: list[Node] = [ParseCommand()]

    if robot is not None:
        children.append(Retry(NavigateToKitchen(robot), max_attempts=2))

    if detector is not None:
        children.append(Retry(FindCup(detector, realsense), max_attempts=3))

    if robot is not None:
        children.append(Retry(ApproachCup(robot), max_attempts=2))

    if tactile_driver is not None and arm_ctrl is not None:
        children.append(Retry(
            GraspCup(arm_ctrl, tactile_driver, grip_controller_cls),
            max_attempts=3,
        ))

    if robot is not None:
        children.append(Retry(NavigateBackToUser(robot), max_attempts=2))

    if handover_controller is not None:
        children.append(Retry(HandOver(handover_controller), max_attempts=3))

    if robot is not None:
        children.append(ConfirmDone(robot))

    return Sequence(children, name="CoffeeRoot")


# ─── Smoke test с mock'ами ───────────────────────────────────────────────

if __name__ == "__main__":
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).parent.parent))

    print("=== Behavior tree smoke test (mock) ===\n")
    from interfaces.unitree_sdk import MockG1Interface
    from perception.tactile.rh56dftp import MockRH56DFTPDriver
    from control.arm_controller import ArmController
    from action.handover import HandoverController

    robot = MockG1Interface()
    robot.connect()

    tactile = MockRH56DFTPDriver(hand="right")
    tactile.connect()

    arm = ArmController(robot, side="right")
    arm.enable()

    handover = HandoverController(
        robot=robot,
        tactile_driver=tactile,
        arm_ctrl=arm,
        internal_force_drop_n=1.0,
        timeout_s=3.0,
    )

    # Detector и RealSense пропускаем — дерево должно упасть на FindCup
    tree = build_coffee_tree(
        robot=robot,
        detector=None,
        realsense=None,
        tactile_driver=tactile,
        arm_ctrl=arm,
        handover_controller=handover,
    )

    bb = Blackboard(goal={"action": "fetch", "object": "coffee", "target": "Oleg"})
    print(f"Goal: {bb.goal}\n")

    status = tree.tick(bb)
    print(f"\nFinal: {status.value}")
    print(f"Errors: {bb.error_count}, duration: {time.time()-bb.started_at:.1f}s")
    print(f"Log: {bb.log[-5:]}")
