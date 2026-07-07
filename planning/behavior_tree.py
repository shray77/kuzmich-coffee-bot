"""
planning/behavior_tree.py
─────────────────────────
Behavior tree для задачи "принеси кофе".

Структура дерева:

  Sequence (root)
  ├── ParseCommand          # разбираем голос → Goal
  ├── NavigateToKitchen     # локомоция к кофеварке
  ├── FindCup               # YOLO + RealSense → 3D позиция чашки
  ├── ApproachCup           # подойти в зону досягаемости
  ├── GraspCup              # closed-loop grip с тактильной обратной связью
  ├── NavigateBackToUser    # вернуться к Олеже
  ├── HandOver              # передать чашку (detect hand + force release)
  └── ConfirmDone           # голосом "держи, Олежа"

Каждый узел возвращает Status.SUCCESS / Status.FAILURE / Status.RUNNING.
Если какой-то узел упал — дерево перезапускается (макс 3 ретрая).
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class Status(Enum):
    SUCCESS = "success"
    FAILURE = "failure"
    RUNNING = "running"


@dataclass
class Blackboard:
    """Shared state между узлами дерева."""
    goal: Optional[dict] = None
    cup_xyz: Optional[tuple[float, float, float]] = None
    cup_detected: bool = False
    cup_grasped: bool = False
    user_position: Optional[tuple[float, float, float]] = None
    user_detected: bool = False
    handover_complete: bool = False
    error_count: int = 0
    last_error: str = ""
    started_at: float = field(default_factory=time.time)


class Node:
    """Базовый класс для узла behavior tree."""
    def tick(self, bb: Blackboard) -> Status:
        raise NotImplementedError


class Sequence(Node):
    """Выполняет детей по очереди. Если любой упал — Sequence падает."""
    def __init__(self, children: list[Node]):
        self.children = children
        self._idx = 0

    def tick(self, bb: Blackboard) -> Status:
        while self._idx < len(self.children):
            status = self.children[self._idx].tick(bb)
            if status == Status.RUNNING:
                return Status.RUNNING
            if status == Status.FAILURE:
                self._idx = 0
                return Status.FAILURE
            self._idx += 1
        self._idx = 0
        return Status.SUCCESS


class Retry(Node):
    """Оборачивает узел: повторяет N раз при FAILURE."""
    def __init__(self, child: Node, max_attempts: int = 3):
        self.child = child
        self.max_attempts = max_attempts
        self._attempt = 0

    def tick(self, bb: Blackboard) -> Status:
        while self._attempt < self.max_attempts:
            status = self.child.tick(bb)
            if status in (Status.SUCCESS, Status.RUNNING):
                return status
            self._attempt += 1
            bb.error_count += 1
            bb.last_error = f"{self.child.__class__.__name__} failed (attempt {self._attempt})"
            time.sleep(0.5)
        self._attempt = 0
        return Status.FAILURE


# ─── Конкретные узлы (пока заглушки) ─────────────────────────────────────

class ParseCommand(Node):
    def tick(self, bb: Blackboard) -> Status:
        # TODO: дёрнуть VoiceAssistant.listen_and_parse()
        if bb.goal is None:
            bb.goal = {"action": "fetch", "object": "coffee", "target": "self"}
        return Status.SUCCESS


class NavigateToKitchen(Node):
    def tick(self, bb: Blackboard) -> Status:
        # TODO: Nav2 / Unitree LOCO policy → идти к точке кухни
        print("  [NavigateToKitchen] walking...")
        return Status.SUCCESS


class FindCup(Node):
    def tick(self, bb: Blackboard) -> Status:
        # TODO: CupDetector.detect_with_depth() → bb.cup_xyz
        print("  [FindCup] scanning for cup...")
        bb.cup_xyz = (0.4, 0.0, 0.8)  # placeholder: 40 см вперёд, 80 см высота
        bb.cup_detected = True
        return Status.SUCCESS


class ApproachCup(Node):
    def tick(self, bb: Blackboard) -> Status:
        # TODO: подойти так, чтобы чашка была в зоне досягаемости руки
        print(f"  [ApproachCup] approaching {bb.cup_xyz}")
        return Status.SUCCESS


class GraspCup(Node):
    def tick(self, bb: Blackboard) -> Status:
        # TODO: GripController.closed-loop grip с тактильной обратной связью
        print("  [GraspCup] gripping...")
        bb.cup_grasped = True
        return Status.SUCCESS


class NavigateBackToUser(Node):
    def tick(self, bb: Blackboard) -> Status:
        # TODO: вернуться к пользователю (по запомненной позиции)
        print("  [NavigateBackToUser] walking back...")
        return Status.SUCCESS


class HandOver(Node):
    def tick(self, bb: Blackboard) -> Status:
        # TODO: поднять руку с чашкой к пользователю, дождаться контакта,
        #       ослабить grip когда внешняя сила > X
        print("  [HandOver] handing over...")
        bb.handover_complete = True
        return Status.SUCCESS


class ConfirmDone(Node):
    def tick(self, bb: Blackboard) -> Status:
        # TODO: TTS "держи, Олежа"
        print("  [ConfirmDone] 'Держи, Олежа!'")
        return Status.SUCCESS


# ─── Сборка дерева ───────────────────────────────────────────────────────

def build_coffee_tree() -> Sequence:
    return Sequence([
        ParseCommand(),
        Retry(NavigateToKitchen(), max_attempts=2),
        Retry(FindCup(), max_attempts=3),
        Retry(ApproachCup(), max_attempts=2),
        Retry(GraspCup(), max_attempts=3),
        Retry(NavigateBackToUser(), max_attempts=2),
        Retry(HandOver(), max_attempts=3),
        ConfirmDone(),
    ])


if __name__ == "__main__":
    tree = build_coffee_tree()
    bb = Blackboard()
    print("=== Кузьмич: принеси кофе ===\n")
    status = tree.tick(bb)
    print(f"\nFinal: {status.value}")
    print(f"Errors: {bb.error_count}, duration: {time.time()-bb.started_at:.1f}s")
