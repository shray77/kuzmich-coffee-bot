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
пресетов — XTTS слишком тяжёлый для live-синтеза на борту G1. Для всего
остального (текст анекдота на случайную тему из RAG) голоса Бурунова нет.

Темы (topics) больше НЕ хардкодятся тут — берём их из GET /presets
(поле "topics"), которое burunov-joke-bot строит динамически из
manifest.json (см. preset_audio.topics_available()). Так расширение
набора анекдотов (scripts/select_curated_jokes.py + Colab-озвучка) сразу
даёт больше вариантов ответа тут, без правки этого файла. JOKE_TOPIC_PRESETS
ниже — только аварийный fallback, если /presets недоступен при старте.
"""
from __future__ import annotations

import os
import random
import time
from typing import Optional

import httpx

RAG_URL = os.environ.get("RAG_URL", "http://127.0.0.1:8000")
TOPICS_CACHE_TTL_S = 60.0

# Fallback на случай, если burunov-joke-bot недоступен при старте —
# те же 5 тем что были захардкожены раньше. Обновляется живыми данными
# из /presets при первом успешном запросе (см. _get_topics()).
JOKE_TOPIC_PRESETS = {
    "Штирлиц": ("shtirlits_intro", "shtirlits_joke"),
    "Вовочка": ("volodka_intro", "volodka_joke"),
    "Ржевский": ("rzhevsky_intro", "rzhevsky_joke"),
    "Новый русский": ("new_russians_intro", "new_russians_joke"),
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

_topics_cache: Optional[dict[str, list[str]]] = None
_topics_cache_t: float = 0.0


def _get_topics(timeout: float = 5.0) -> dict[str, list[str]]:
    """Тема -> список пресетов с готовым звуком. Кэш на TOPICS_CACHE_TTL_S,
    чтобы не дёргать HTTP на каждую фразу. Если сервис недоступен —
    деградирует на статический JOKE_TOPIC_PRESETS (по одному анекдоту на тему,
    без вариативности, но хоть что-то)."""
    global _topics_cache, _topics_cache_t
    now = time.time()
    if _topics_cache is not None and (now - _topics_cache_t) < TOPICS_CACHE_TTL_S:
        return _topics_cache
    try:
        with httpx.Client(timeout=timeout) as c:
            r = c.get(f"{RAG_URL}/presets")
            r.raise_for_status()
            topics = r.json().get("topics") or {}
            if topics:
                _topics_cache = topics
                _topics_cache_t = now
                return topics
    except Exception as e:
        print(f"[preset_client] /presets недоступен ({e}) — используем статический fallback")
    # fallback: топ-1 анекдот на тему из старого хардкода
    return {k: [v[1]] for k, v in JOKE_TOPIC_PRESETS.items()}


def match_joke_topic(topic: str) -> Optional[str]:
    """Найти тему (регистронезависимо, по вхождению подстроки в обе стороны)
    среди живых тем с готовым звуком."""
    if not topic:
        return None
    t = topic.strip().lower()
    for key in _get_topics():
        if key.lower() in t or t in key.lower():
            return key
    return None


def pick_joke_preset(topic_key: str) -> Optional[str]:
    """Случайный анекдот-пресет для уже сматченной темы (реальная
    вариативность, если под тему озвучено несколько анекдотов)."""
    names = _get_topics().get(topic_key)
    if not names:
        return None
    return random.choice(names)


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
