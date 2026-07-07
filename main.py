"""
Кузьмич — main entry point.

Полный цикл "принеси кофе":
1. Ждём голосовую команду
2. Парсим через Whisper + LLM
3. Запускаем behavior tree
"""
import time
from perception.voice.assistant import VoiceAssistant, Goal
from planning.behavior_tree import build_coffee_tree, Blackboard
from interfaces.unitree_sdk import UnitreeG1Interface, MockG1Interface


def main():
    print("=" * 60)
    print("КУЗЬМИЧ v0.1 — Coffee Bot")
    print("=" * 60)

    # Инициализация (пока всё в Mock-режиме)
    # voice = VoiceAssistant(whisper_model="medium", whisper_device="cuda:0")
    robot = MockG1Interface()
    robot.connect()
    robot.stand_up()
    print("Кузьмич стоит и ждёт команду.\n")

    # Главная петля
    while True:
        cmd = input(">>> ").strip().lower()
        if cmd in ("quit", "exit", "q"):
            print("Кузьмич: до свидания.")
            break
        if not cmd:
            continue

        # Mock-goal (в реале — voice.listen_and_parse("mic.wav"))
        if "кофе" in cmd or "чай" in cmd or "принеси" in cmd:
            goal = Goal(action="fetch", object="coffee", target="self", raw_text=cmd)
        elif "стой" in cmd or "хватит" in cmd:
            goal = Goal(action="abort", raw_text=cmd)
        else:
            goal = Goal(action="unknown", raw_text=cmd)

        print(f"\nКузьмич понял: {goal}\n")

        if goal.action == "fetch":
            bb = Blackboard(goal={"raw": goal.raw_text})
            tree = build_coffee_tree()
            status = tree.tick(bb)
            print(f"\nРезультат: {status.value} (ошибок: {bb.error_count})")
        elif goal.action == "abort":
            robot.emergency_stop()
        else:
            print("Кузьмич: не понял, повтори.")

        print()

    robot.sit_down()


if __name__ == "__main__":
    main()
