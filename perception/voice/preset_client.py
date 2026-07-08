"""
perception/voice/preset_client.py
──────────────────────────────────
HTTP-клиент к готовым wav-пресетам голоса Бурунова, которые раздаёт
burunov-joke-bot/api.py (GET /presets, GET /presets/{name}/audio).

Зачем отдельный клиент, а не прямой импорт burunov-joke-bot: это два
разных процесса/репозитория (см. main.py — RAG-текст анекдотов уже
дёргается так же, по HTTP, а не импортом). burunov-joke-bot — "мозг"
(RAG-текст + синтезированный голос), kuzmich-coffee-bot — "тело"
(локомоция, руки, реально говорит через AudioClient.PlayStream).

⚠️ Голосом Бурунова можно сказать ТОЛЬКО фразы из фиксированного набора
пресетов (5 тем анекдотов + 6 фраз доставки кофе) — XTTS слишком тяжёлый
для live-синтеза на борту G1. Для всего остального (текст анекдота на
случайную тему из RAG) голоса Бурунова нет — см. JOKE_TOPIC_PRESETS ниже.
"""
from __future__ import annotations

import os
from typing import Optional

import httpx

RAG_URL = os.environ.get("RAG_URL", "http://127.0.0.1:8000")

# Дублирует burunov-joke-bot/preset_audio.py — держим списком тут тоже,
# чтобы не тянуть импорт из соседнего репо. Если темы там поменяются,
# поменять и здесь (или спросить /presets и матчить динамически, см. ниже).
JOKE_TOPIC_PRESETS = {
    "Штирлиц": ("shtirlits_intro", "shtirlits_joke"),
    "Вовочка": ("volodka_intro", "volodka_joke"),
    "Ржевский": ("rzhevsky_intro", "rzhevsky_joke"),
    "Новые русские": ("new_russians_intro", "new_russians_joke"),
    "Чапаев": ("chapaev_intro", "chapaev_joke"),
}

COFFEE_PRESETS = {
    "intro": "coffee_intro",
    "obstacle": "coffee_obstacle",
    "no_cup": "coffee_no_cup",
    "got_it": "coffee_got_it",
    "dropped": "coffee_dropped",
    "done": "coffee_done",
}


def match_joke_topic(topic: str) -> Optional[str]:
    """Найти ключ JOKE_TOPIC_PRESETS, который соответствует произвольной теме
    (регистронезависимо, по вхождению подстроки в обе стороны)."""
    if not topic:
        return None
    t = topic.strip().lower()
    for key in JOKE_TOPIC_PRESETS:
        if key.lower() in t or t in key.lower():
            return key
    return None


def fetch_preset_pcm(name: str, timeout: float = 10.0) -> Optional[bytes]:
    """Скачать PCM пресета (16kHz/mono/16-bit, без заголовка) по имени.
    None если сервис недоступен или пресета нет — вызывающий код должен
    сам решить, деградировать ли на robot.say() (не-Бурунов голос)."""
    try:
        with httpx.Client(timeout=timeout) as c:
            r = c.get(f"{RAG_URL}/presets/{name}/audio")
            if r.status_code == 404:
                return None
            r.raise_for_status()
            return r.content
    except Exception as e:
        print(f"[preset_client] не удалось получить пресет {name!r}: {e}")
        return None


def speak_preset(robot, name: str, fallback_text: str = "") -> bool:
    """Проиграть пресет голосом Бурунова через robot.play_pcm(). Если пресет
    недоступен (RAG-сервис лежит, пресета нет) — деградирует на robot.say()
    со fallback_text (НЕ голосом Бурунова), если тот передан, иначе молчит.
    Возвращает True если реально сказано голосом Бурунова."""
    pcm = fetch_preset_pcm(name)
    if pcm is not None:
        return bool(robot.play_pcm(pcm))
    if fallback_text:
        robot.say(fallback_text)
    return False
