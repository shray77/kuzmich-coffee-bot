"""
Кузьмич — main entry point.

Полный цикл "принеси кофе":
1. Ждём голосовую команду (или текстовый ввод в dev-режиме)
2. Парсим через Whisper + LLM (или keyword-match fallback)
3. Запускаем behavior tree с реальными компонентами

Режимы:
  python main.py                  — текстовый ввод, mock-робот (dev)
  python main.py --mock           — явно mock-режим (всё симулировано)
  python main.py --robot          — реальный G1 (нужен SDK + Ethernet)
  python main.py --voice          — слушать микрофон через Whisper
  python main.py --lowlevel       — включить low-level arm control (ОСТОРОЖНО)
  python main.py --hand right|left— выбрать руку (default: right)
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

# Локальные импорты
from perception.voice.assistant import VoiceAssistant, Goal
from planning.behavior_tree import build_coffee_tree, Blackboard, Status
from interfaces.unitree_sdk import UnitreeG1Interface, MockG1Interface

# RAG-сервис анекдотов из соседнего репо burunov-joke-bot (api.py, POST /tell)
RAG_URL = os.environ.get("RAG_URL", "http://127.0.0.1:8000")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Кузьмич — Coffee Bot")
    p.add_argument("--mock", action="store_true",
                   help="Принудительный mock-режим (без реального железа)")
    p.add_argument("--robot", action="store_true",
                   help="Подключиться к реальному G1 (нужен SDK + Ethernet)")
    p.add_argument("--lowlevel", action="store_true",
                   help="Включить low-level arm control (ОПАСНО — можно уронить робота)")
    p.add_argument("--voice", action="store_true",
                   help="Слушать микрофон через Whisper (нужен openai-whisper)")
    p.add_argument("--hand", choices=["left", "right"], default="right",
                   help="Какой рукой брать чашку (default: right)")
    p.add_argument("--whisper-model", default="medium",
                   help="Whisper model: tiny/base/small/medium/large-v3")
    p.add_argument("--ollama-model", default="gemma4:e2b",
                   help="Ollama model для парсинга команд (Gemma 4, апрель 2026)")
    p.add_argument("--ollama-host", default="http://localhost:11434",
                   help="Ollama API host")
    return p.parse_args()


def setup_robot(args) -> object:
    """Инициализация интерфейса робота (real или mock)."""
    if args.robot and not args.mock:
        robot = UnitreeG1Interface(
            enable_low_level=args.lowlevel,
        )
        print("[main] Подключение к реальному G1...")
    else:
        robot = MockG1Interface(enable_low_level=args.lowlevel)
        print("[main] Mock-режим (нет --robot флага)")

    robot.connect()
    robot.stand_up()
    return robot


def setup_voice(args) -> VoiceAssistant | None:
    """Инициализация голосового ассистента. None если недоступен."""
    if not args.voice:
        return None
    try:
        assistant = VoiceAssistant(
            whisper_model=args.whisper_model,
            whisper_device="cuda:0" if _torch_cuda_available() else "cpu",
            ollama_host=args.ollama_host,
            ollama_model=args.ollama_model,
        )
        print(f"[main] VoiceAssistant: Whisper={args.whisper_model}, "
              f"Ollama={args.ollama_model}")
        return assistant
    except Exception as e:
        print(f"[main] VoiceAssistant недоступен: {e}")
        print("[main] Fallback на keyword-match из текстового ввода")
        return None


def _torch_cuda_available() -> bool:
    try:
        import torch
        return torch.cuda.is_available()
    except ImportError:
        return False


JOKE_TOPICS = ["Штирлиц", "Вовочка", "Ржевский", "Чапаев", "Новые русские"]


def keyword_fallback(text: str) -> Goal:
    """Простой keyword-match если LLM недоступен."""
    t = text.lower().strip()
    if "кофе" in t:
        return Goal(action="fetch", object="coffee", target="self", raw_text=text, confidence=0.7)
    if "чай" in t:
        return Goal(action="fetch", object="tea", target="self", raw_text=text, confidence=0.7)
    if "принеси" in t and "олег" in t:
        return Goal(action="fetch", object="coffee", target="Oleg", raw_text=text, confidence=0.6)
    if "анекдот" in t or "шутк" in t or "расскажи" in t:
        topic = next((tp for tp in JOKE_TOPICS if tp.lower() in t), "")
        return Goal(action="tell_joke", topic=topic, raw_text=text, confidence=0.7)
    if any(w in t for w in ("стой", "хватит", "стоп")):
        return Goal(action="abort", raw_text=text, confidence=0.9)
    return Goal(action="unknown", raw_text=text, confidence=0.0)


def tell_joke(robot, topic: str) -> None:
    """
    Если topic попадает в один из 5 фиксированных пресетов — говорим
    intro+joke ГОЛОСОМ БУРУНОВА (preset_client.py, реальный синтезированный
    голос). Для любой другой темы — RAG /tell из burunov-joke-bot даёт
    текст на лету, но озвучить его Бурунов не может (XTTS слишком тяжёлый
    для live на борту G1), так что это встроенный TTS через robot.say().
    """
    from perception.voice.preset_client import match_joke_topic, JOKE_TOPIC_PRESETS, speak_preset

    preset_key = match_joke_topic(topic)
    if preset_key is not None:
        intro_name, joke_name = JOKE_TOPIC_PRESETS[preset_key]
        print(f"  [joke] пресет голосом Бурунова: {preset_key}")
        speak_preset(robot, intro_name)
        speak_preset(robot, joke_name)
        return

    import httpx
    print(f"  [joke] тема {topic!r} не в пресетах — RAG {RAG_URL}/tell (НЕ голос Бурунова)")
    try:
        with httpx.Client(timeout=15.0) as c:
            r = c.post(f"{RAG_URL}/tell", json={"topic": topic or "1986"})
            r.raise_for_status()
            text = r.json().get("text", "")
    except Exception as e:
        print(f"  [joke] RAG недоступен: {e}")
        robot.say("Э, анекдот не грузится, RAG видимо не запущен.")
        return
    if not text:
        robot.say("Не придумал анекдот, извини.")
        return
    print(f"  [joke] {text}")
    robot.say(text)


def run_coffee_task(robot, voice: VoiceAssistant | None, args) -> None:
    """Запуск задачи 'принеси кофе'."""
    # Lazy-импорт остальных компонентов (чтобы не тормозить startup)
    from perception.tactile.rh56dftp import (
        RH56DFTPDriver, MockRH56DFTPDriver, GripController,
    )
    from control.arm_controller import ArmController, DualArmController
    from control.safety import SafetyMonitor
    from action.handover import HandoverController

    # Tactile
    if args.mock or not args.robot:
        tactile = MockRH56DFTPDriver(hand=args.hand)
    else:
        # TODO: читать порт из configs/tactile_port.json
        tactile = RH56DFTPDriver(hand=args.hand, port="/dev/ttyUSB0")
    tactile.connect()

    # Arm controller
    arm = ArmController(robot, side=args.hand)
    arm.enable()

    # Handover
    handover = HandoverController(
        robot=robot,
        tactile_driver=tactile,
        arm_ctrl=arm,
        internal_force_drop_n=1.0,
        timeout_s=15.0,
    )

    # Detector: в mock-режиме — фейковая чашка (без torch/ultralytics/камеры),
    # на реальном роботе — настоящий YOLOv8
    detector = None
    if args.mock or not args.robot:
        from perception.vision.detector import MockCupDetector
        detector = MockCupDetector()
        print("[main] MockCupDetector (fake cup)")
    else:
        try:
            from perception.vision.detector import CupDetector
            device = "cuda:0" if _torch_cuda_available() else "cpu"
            detector = CupDetector(model_path="yolov8m.pt", device=device)
            print(f"[main] CupDetector loaded on {device}")
        except Exception as e:
            print(f"[main] CupDetector недоступен: {e}")
            print("[main] FindCup будет возвращать FAILURE (нужно CV для работы)")

    # Safety monitor (фоновый)
    safety = SafetyMonitor(robot, tactile)
    safety.start()

    # Сборка дерева
    tree = build_coffee_tree(
        robot=robot,
        detector=detector,
        realsense=None,         # TODO: обёртка над pyrealsense2
        tactile_driver=tactile,
        arm_ctrl=arm,
        handover_controller=handover,
        grip_controller_cls=GripController,
    )

    try:
        print("\n" + "=" * 60)
        print("🤖 Кузьмич готов. Жду команду.")
        print("   Введи текст или 'голос' для записи (если --voice)")
        print("   'выход' / 'quit' — завершить")
        print("=" * 60 + "\n")

        while True:
            try:
                cmd = input(">>> ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not cmd:
                continue
            if cmd.lower() in ("выход", "quit", "exit", "q"):
                print("Кузьмич: до свидания.")
                break

            # Голосовой ввод
            if cmd.lower() == "голос" and voice is not None:
                cmd = _record_and_transcribe(voice)
                if not cmd:
                    continue

            # Парсинг команды
            if voice is not None:
                try:
                    goal = voice.parse_command(cmd)
                except Exception as e:
                    print(f"[main] LLM parse failed ({e}), используем keyword-match")
                    goal = keyword_fallback(cmd)
            else:
                goal = keyword_fallback(cmd)

            print(f"\nКузьмич понял: {goal}\n")

            if goal.action == "abort":
                robot.emergency_stop()
                safety.emergency_stop()
                continue
            if goal.action == "tell_joke":
                tell_joke(robot, goal.topic)
                continue
            if goal.action != "fetch":
                print("Кузьмич: не понял, повтори.")
                continue

            # Запуск behavior tree
            from perception.voice.preset_client import speak_preset

            recipient = goal.target if goal.target and goal.target != "self" else "Олег"
            speak_preset(robot, "coffee_intro",
                         fallback_text=f"Угу, щас, {recipient} Тарасыч... кофеварку найду...")

            bb = Blackboard(goal={
                "action": goal.action,
                "object": goal.object,
                "target": goal.target,
                "raw": goal.raw_text,
            })
            print(f"\n--- Запуск behavior tree ---")
            t0 = time.time()
            status = tree.tick(bb)
            dt = time.time() - t0
            print(f"\n--- Результат: {status.value} за {dt:.1f}с, "
                  f"ошибок: {bb.error_count} ---")
            if bb.last_error:
                print(f"    последняя ошибка: {bb.last_error}")

            # coffee_got_it/coffee_done уже озвучены внутри дерева
            # (GraspCup/ConfirmDone). Тут — только исходы, до которых дерево
            # само не договаривает голосом.
            if status == Status.FAILURE:
                if not bb.cup_detected:
                    speak_preset(robot, "coffee_no_cup",
                                 fallback_text=f"Не вижу никакой чашки, {recipient}. Где кофе-то?")
                elif bb.cup_grasped and not bb.handover_complete:
                    speak_preset(robot, "coffee_dropped", fallback_text="Ой... выронил, бля.")

            tree.reset()
            print()

    finally:
        safety.stop()
        try:
            tactile.close()
        except Exception:
            pass
        try:
            arm.relax()
        except Exception:
            pass
        try:
            robot.stand_down()
        except Exception:
            pass


def _record_and_transcribe(voice: VoiceAssistant) -> str:
    """Запись 5 сек с микрофона → WAV → Whisper STT."""
    try:
        import wave
        import tempfile
        import os
        try:
            import pyaudio
        except ImportError:
            print("[main] pyaudio не установлен — запись невозможна")
            return ""

        print("[voice] Запись 5 сек...")
        chunk = 1024
        rate = 16000
        channels = 1
        p = pyaudio.PyAudio()
        stream = p.open(format=pyaudio.paInt16, channels=channels,
                        rate=rate, input=True, frames_per_buffer=chunk)
        frames = []
        for _ in range(0, int(rate / chunk * 5)):
            data = stream.read(chunk)
            frames.append(data)
        stream.stop_stream()
        stream.close()
        p.terminate()

        # Сохранить во временный WAV
        with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
            wf = wave.open(f.name, "wb")
            wf.setnchannels(channels)
            wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
            wf.setframerate(rate)
            wf.writeframes(b"".join(frames))
            wf.close()
            tmp_path = f.name

        text = voice.transcribe(tmp_path)
        os.unlink(tmp_path)
        print(f"[voice] STT: {text!r}")
        return text
    except Exception as e:
        print(f"[voice] error: {e}")
        return ""


def main():
    args = parse_args()
    print("=" * 60)
    print("☕ КУЗЬМИЧ v0.2 — Coffee Bot for Unitree G1 EDU Ultimate")
    print("=" * 60)

    robot = setup_robot(args)
    voice = setup_voice(args)

    try:
        run_coffee_task(robot, voice, args)
    finally:
        try:
            robot.stand_down()
        except Exception:
            pass


if __name__ == "__main__":
    main()
