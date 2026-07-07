"""
control/arm_poses.py
────────────────────
Библиотека предустановленных поз для рук Unitree G1.

G1 EDU Ultimate: 7 DoF на руку (shoulder_pitch, shoulder_roll, shoulder_yaw,
elbow, wrist_pitch, wrist_roll, wrist_yaw) + 2 мотора торса (torso_pitch, torso_roll).

Углы в РАДИАНАХ. Ноль = нейтральное положение (рука вдоль тела).

⚠️ ВСЕ ПОЗЫ — НАЧАЛЬНЫЕ ПРИБЛИЖЕНИЯ. Подлежат калибровке на реальном роботе:
1. Поставить робота в калибровочную раму (есть в комплекте G1 EDU)
2. Зафиксировать JointStates в позе 'home'
3. Скорректировать смещения в `JOINT_OFFSETS` для каждой руки
4. Проверить 'pregrasp' и 'handover' — рука не должна бить по корпусу

Использование:
    from control.arm_poses import POSES, get_pose
    pose = get_pose("right", "pregrasp")  # → list[7] радиан
"""
from __future__ import annotations

from typing import Literal

ArmSide = Literal["left", "right"]

# 7 суставов руки G1 в порядке SDK2:
#   0: shoulder_pitch  (вперёд/назад, 0 = вниз, +π/2 = вперёд горизонтально)
#   1: shoulder_roll   (в стороны, 0 = вдоль тела, +π/2 = рука в сторону)
#   2: shoulder_yaw    (вращение вокруг вертикальной оси руки)
#   3: elbow           (0 = выпрямлено, +π/2 = согнуто под 90°)
#   4: wrist_pitch     (вперёд/назад кистью)
#   5: wrist_roll      (вращение кисти вокруг своей оси)
#   6: wrist_yaw       (боковой наклон кисти)
NUM_ARM_JOINTS = 7

# Торс: 2 мотора (P = pitch, R = roll) — общие для всего тела
NUM_TORSO_JOINTS = 2

# Поправки на зеркальность левой/правой руки.
# Знаки roll/yaw инвертируются для левой.
_SIGN_MIRROR = {
    "shoulder_roll": -1.0,
    "wrist_roll":    -1.0,
    "wrist_yaw":     -1.0,
}
_JOINT_INDEX_FOR_MIRROR = {1: "shoulder_roll", 5: "wrist_roll", 6: "wrist_yaw"}

# Калибровочные смещения (заполняются после стенда).
# Структура: {"left": [7 смещений], "right": [7 смещений]}
JOINT_OFFSETS: dict[str, list[float]] = {
    "left":  [0.0] * NUM_ARM_JOINTS,
    "right": [0.0] * NUM_ARM_JOINTS,
}

# Торс: pitch (наклон вперёд), roll (наклон в сторону)
TORSO_OFFSETS: list[float] = [0.0, 0.0]


# ─── Базовые позы (для правой руки — для левой будет зеркально) ──────────
# Все углы в радианах, безопасный диапазон ±1.5 от нейтрали
_BASE_POSES_RIGHT: dict[str, list[float]] = {
    # Полностью вытянута вдоль тела — стартовая/безопасная поза
    "home":       [0.0,  0.0,  0.0,  0.0,  0.0, 0.0, 0.0],

    # Рука расслаблена, слегка согнута в локте (15°), кисть нейтрально
    "idle":       [0.10, 0.05, 0.0,  0.25, 0.0, 0.0, 0.0],

    # Протянуть руку вперёд горизонтально, ладонь вниз — для захвата
    # чашки на столе высотой ~0.75 м (стандартная кухня)
    "pregrasp":   [1.45, 0.10, 0.0,  0.50, 0.0, 0.0, 0.0],

    # Слегка опустить кисть, чтобы обхватить чашку (post-pregrasp)
    "grasp":      [1.50, 0.15, 0.0,  0.75, 0.10, 0.0, 0.0],

    # Поднять чашку со стола (локоть сгибается, плечо поднимается выше)
    "lift":       [0.80, 0.15, 0.0,  1.20, 0.40, 0.0, 0.0],

    # Нести чашку у корпуса (стабильная поза для ходьбы)
    "carry":      [0.30, 0.20, 0.0,  1.40, 0.40, 0.0, 0.0],

    # Передача: рука вытянута вперёд-вверх на высоту груди человека (~1.2 м)
    # Чашка повернута к получателю
    "handover":   [1.20, 0.30, 0.0,  0.90, 0.20, 0.0, 0.0],

    # Сброс: опустить руку вниз (после передачи, перед возвратом в home)
    "release":    [0.0,  0.10, 0.0,  0.30, 0.0, 0.0, 0.0],

    # Жест «приветствие» (помахать) — правая рука поднята в сторону
    "wave":       [0.50, 1.20, 0.0,  0.30, 0.0, 0.0, 0.0],

    # Указать на объект (вперёд)
    "point":      [1.50, 0.0,  0.0,  0.0,  0.0, 0.0, 0.0],
}

# Торс в разных позах: [pitch, roll]
TORSO_POSES: dict[str, list[float]] = {
    "home":       [0.0, 0.0],
    "idle":       [0.0, 0.0],
    "pregrasp":   [0.15, 0.0],   # слегка наклониться вперёд к столу
    "grasp":      [0.25, 0.0],   # больше наклон для захвата
    "lift":       [0.0,  0.0],   # выпрямиться
    "carry":      [0.0,  0.0],
    "handover":   [0.0,  0.0],
    "release":    [0.0,  0.0],
    "wave":       [0.0,  0.0],
    "point":      [0.10, 0.0],
}

# Скорость интерполяции между позами (секунд на переход)
POSE_DURATIONS: dict[str, float] = {
    "home":       2.0,
    "idle":       1.0,
    "pregrasp":   1.5,
    "grasp":      0.8,
    "lift":       1.2,
    "carry":      1.0,
    "handover":   1.5,
    "release":    0.8,
    "wave":       0.6,
    "point":      1.0,
}


def get_pose(side: ArmSide, pose_name: str) -> list[float]:
    """
    Возвращает массив из 7 углов (рад) для заданной руки и позы.

    side: "left" | "right"
    pose_name: см. ключи _BASE_POSES_RIGHT

    Для левой руки инвертируются знаки roll/yaw суставов.
    Применяются JOINT_OFFSETS (калибровка).
    """
    if side not in ("left", "right"):
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")
    if pose_name not in _BASE_POSES_RIGHT:
        raise ValueError(
            f"Unknown pose {pose_name!r}. Available: {list(_BASE_POSES_RIGHT)}"
        )

    base = list(_BASE_POSES_RIGHT[pose_name])
    offsets = JOINT_OFFSETS[side]

    if side == "left":
        # Зеркалим roll/yaw
        for idx, name in _JOINT_INDEX_FOR_MIRROR.items():
            base[idx] *= _SIGN_MIRROR[name]

    # Применяем калибровочные смещения
    pose = [b + o for b, o in zip(base, offsets)]
    return pose


def get_torso_pose(pose_name: str) -> list[float]:
    """Возвращает [pitch, roll] торса для заданной позы (с калибровкой)."""
    if pose_name not in TORSO_POSES:
        return list(TORSO_OFFSETS)  # безопасный дефолт
    base = TORSO_POSES[pose_name]
    return [b + o for b, o in zip(base, TORSO_OFFSETS)]


def get_pose_duration(pose_name: str) -> float:
    """Сколько секунд должно занять перемещение в эту позу."""
    return POSE_DURATIONS.get(pose_name, 1.5)


def list_poses() -> list[str]:
    return list(_BASE_POSES_RIGHT.keys())


# ─── Утилита: интерполяция между двумя позами ────────────────────────────
def interpolate_poses(p_start: list[float], p_end: list[float], t: float) -> list[float]:
    """
    Линейная интерполяция между двумя позами.
    t в диапазоне [0, 1]. t=0 → p_start, t=1 → p_end.
    """
    if len(p_start) != len(p_end):
        raise ValueError(f"Pose length mismatch: {len(p_start)} vs {len(p_end)}")
    t = max(0.0, min(1.0, t))
    # smoothstep для более естественного движения
    t_smooth = t * t * (3 - 2 * t)
    return [s + (e - s) * t_smooth for s, e in zip(p_start, p_end)]


# ─── Самотестирование ───────────────────────────────────────────────────
def _round(x: float) -> float:
    return round(x, 3)


if __name__ == "__main__":
    print("=== Кузьмич arm_poses — sanity check ===\n")
    print(f"Available poses: {list_poses()}\n")
    for pose in list_poses():
        rp = get_pose("right", pose)
        lp = get_pose("left",  pose)
        tp = get_torso_pose(pose)
        print(f"  {pose:10s}  R={ [_round(x) for x in rp] }")
        print(f"  {'':10s}  L={ [_round(x) for x in lp] }")
        print(f"  {'':10s}  T={ [_round(x) for x in tp] }")
        print(f"  {'':10s}  duration={get_pose_duration(pose)}s\n")
