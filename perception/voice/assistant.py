"""
perception/voice/assistant.py
─────────────────────────────
Голосовой интерфейс Кузьмича.

Пайплайн:
1. VAD (Voice Activity Detection) — silero-vad
2. Whisper STT → текст
3. LLM парсит команду в JSON goal
4. Goal → planning/

Примеры:
  "Кузьмич, принеси кофе"        → {"action":"fetch","object":"coffee","target":"self"}
  "Принеси Олеже чай"            → {"action":"fetch","object":"tea","target":"Oleg"}
  "Стой на месте"                 → {"action":"freeze"}
  "Вернись на базу"               → {"action":"return_home"}
  "Что ты видишь?"                → {"action":"describe_scene"}
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Optional

import httpx


@dataclass
class Goal:
    action: str              # fetch | freeze | return_home | describe_scene | abort
    object: str = ""         # coffee | tea | water | ...
    target: str = "self"     # self | Oleg | ...
    raw_text: str = ""
    confidence: float = 0.0


# Промпт для LLM — парсит фразу в JSON goal
SYSTEM_PROMPT = """Ты — парсер голосовых команд для робота-гуманоида Кузьмич.
Пользователь говорит фразу на русском. Ты возвращаешь ТОЛЬКО JSON без объяснений.

Возможные действия:
- fetch: принести объект. Поля: object (coffee/tea/water/beer/newspaper/phone),
  target (self/Oleg/Mom/Dad/имя)
- freeze: остановиться и стоять
- return_home: вернуться на базу (зарядка)
- describe_scene: описать что видит
- abort: отменить текущую задачу
- wave_hand: помахать рукой
- dance: станцевать

Примеры:
"принеси кофе" → {"action":"fetch","object":"coffee","target":"self"}
"Олежа хочет чай" → {"action":"fetch","object":"tea","target":"Oleg"}
"стой" → {"action":"freeze"}
"хватит" → {"action":"abort"}
"покажи что видишь" → {"action":"describe_scene"}

Если фраза неразборчива или не входит в список действий — верни:
{"action":"unknown","raw":"...фраза..."}

Возвращай ТОЛЬКО JSON, без markdown, без пояснений."""


class VoiceAssistant:
    """Голосовой ассистент: Whisper STT + Ollama LLM."""

    def __init__(
        self,
        whisper_model: str = "medium",     # tiny/base/small/medium/large-v3
        whisper_device: str = "cuda:0",
        ollama_host: str = "http://localhost:11434",
        ollama_model: str = "gemma3:4b",
    ):
        import whisper
        self.whisper = whisper.load_model(whisper_model).to(whisper_device)
        self.ollama_host = ollama_host
        self.ollama_model = ollama_model

    def transcribe(self, audio_path: str) -> str:
        """STT: аудио-файл → текст."""
        result = self.whisper.transcribe(audio_path, language="ru", fp16=True)
        return result["text"].strip()

    def parse_command(self, text: str) -> Goal:
        """LLM: текст → Goal."""
        if not text:
            return Goal(action="unknown", raw_text=text, confidence=0.0)

        payload = {
            "model": self.ollama_model,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": text},
            ],
            "stream": False,
            "options": {"temperature": 0.1, "num_predict": 100},
        }
        with httpx.Client(timeout=15.0) as c:
            r = c.post(f"{self.ollama_host}/api/chat", json=payload)
            r.raise_for_status()
            content = r.json()["message"]["content"].strip()

        # Парсим JSON (LLM может обернуть в markdown)
        if "```" in content:
            content = content.split("```")[1]
            if content.startswith("json"):
                content = content[4:]
        try:
            data = json.loads(content)
            return Goal(
                action=data.get("action", "unknown"),
                object=data.get("object", ""),
                target=data.get("target", "self"),
                raw_text=text,
                confidence=0.9,
            )
        except json.JSONDecodeError:
            return Goal(action="unknown", raw_text=text, confidence=0.0)

    def listen_and_parse(self, audio_path: str) -> Goal:
        """End-to-end: аудио → Goal."""
        text = self.transcribe(audio_path)
        print(f"  STT: {text!r}")
        goal = self.parse_command(text)
        print(f"  Goal: {goal}")
        return goal


# ─── Тест ────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys
    assistant = VoiceAssistant(
        whisper_model="medium",
        whisper_device="cuda:0" if __import__("torch").cuda.is_available() else "cpu",
    )
    if len(sys.argv) > 1:
        audio = sys.argv[1]
        goal = assistant.listen_and_parse(audio)
        print(f"\nGoal: {goal}")
    else:
        # Тест парсера на тексте
        for phrase in ["принеси кофе", "Олежа хочет чай", "стой", "что ты видишь"]:
            print(f"\n>>> {phrase}")
            goal = assistant.parse_command(phrase)
            print(f"Goal: {goal}")
