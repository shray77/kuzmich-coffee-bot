"""
tests/test_unit.py
──────────────────
Unit-тесты для модулей kuzmich-coffee-bot.
Запускаются БЕЗ железа — все компоненты mock.

    pytest tests/test_unit.py -v
"""
import sys
import time
from pathlib import Path

# Добавить корень проекта в sys.path
sys.path.insert(0, str(Path(__file__).parent.parent))


def test_arm_poses():
    """Все предустановленные позы возвращают 7 углов."""
    from control.arm_poses import get_pose, list_poses, NUM_ARM_JOINTS
    for pose_name in list_poses():
        for side in ("left", "right"):
            pose = get_pose(side, pose_name)
            assert len(pose) == NUM_ARM_JOINTS, \
                f"{side}/{pose_name}: expected {NUM_ARM_JOINTS} joints, got {len(pose)}"


def test_arm_poses_mirror():
    """Левая и правая рука — зеркальны по roll/yaw суставам."""
    from control.arm_poses import get_pose, list_poses
    for pose_name in list_poses():
        r = get_pose("right", pose_name)
        l = get_pose("left", pose_name)
        # shoulder_pitch, shoulder_yaw, elbow, wrist_pitch — те же
        assert abs(r[0] - l[0]) < 1e-6, f"{pose_name}: shoulder_pitch differs"
        # shoulder_roll (1), wrist_roll (5), wrist_yaw (6) — зеркальны
        assert abs(r[1] + l[1]) < 1e-6, f"{pose_name}: shoulder_roll not mirrored"
        assert abs(r[5] + l[5]) < 1e-6, f"{pose_name}: wrist_roll not mirrored"
        assert abs(r[6] + l[6]) < 1e-6, f"{pose_name}: wrist_yaw not mirrored"


def test_arm_poses_safe_limits():
    """Все позы — в безопасных пределах."""
    from control.arm_poses import get_pose, list_poses
    from control.arm_controller import SAFE_LIMITS
    joint_names = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw",
                   "elbow", "wrist_pitch", "wrist_roll", "wrist_yaw"]
    for pose_name in list_poses():
        for side in ("left", "right"):
            pose = get_pose(side, pose_name)
            for i, (q, name) in enumerate(zip(pose, joint_names)):
                lo, hi = SAFE_LIMITS[name]
                assert lo - 0.01 <= q <= hi + 0.01, \
                    f"{side}/{pose_name}/{name}: {q} out of [{lo}, {hi}]"


def test_mock_tactile():
    """MockRH56DFTPDriver возвращает корректные показания."""
    from perception.tactile.rh56dftp import MockRH56DFTPDriver
    d = MockRH56DFTPDriver(hand="right")
    d.connect()
    d.close_hand(0.5)
    r = d.read()
    assert r.total_force_n > 0, "Expected non-zero force when gripping"
    assert len(r.forces_n) == 6
    d.open_hand()
    r = d.read()
    assert r.total_force_n < 0.5, "Force should be ~0 when open"


def test_mock_tactile_slip():
    """Slip detection срабатывает при резком падении силы."""
    from perception.tactile.rh56dftp import MockRH56DFTPDriver
    d = MockRH56DFTPDriver(hand="right", max_force_n=10.0)
    d.connect()
    d.apply_external_force(5.0)  # держим
    # Несколько чтений для установления baseline
    for _ in range(3):
        d.read()
        time.sleep(0.02)
    # Резко убираем силу — должно сработать slip
    d.apply_external_force(0.0)
    slip_detected = False
    for _ in range(3):
        r = d.read()
        if r.slip_detected:
            slip_detected = True
            break
        time.sleep(0.02)
    assert slip_detected, "Slip should be detected on rapid force drop"


def test_compliance_estimator():
    """ComplianceEstimator корректно классифицирует мягкий/средний/жёсткий объект."""
    import numpy as np
    from perception.tactile.compliance import ComplianceEstimator, Compliance

    def simulate(k_true: float, steps: int = 20):
        est = ComplianceEstimator()
        pos = 0.0
        for _ in range(steps):
            pos += 0.03
            est.update(pos, max(0.0, k_true * pos))
        return est.estimate()

    soft = simulate(k_true=1.5)
    assert soft.classification == Compliance.SOFT, soft
    rigid = simulate(k_true=20.0)
    assert rigid.classification == Compliance.RIGID, rigid

    empty = ComplianceEstimator().estimate()
    assert empty.classification == Compliance.UNKNOWN, empty


def test_grip_controller_soft_object_caps_force():
    """GripController ловит мягкий объект и стабилизируется на пониженной цели,
    вместо того чтобы бесконечно давить к недостижимому target_force_n."""
    from perception.tactile.rh56dftp import MockRH56DFTPDriver, GripController
    from perception.tactile.compliance import Compliance

    # У мока force = grip_pos * max_force_n. max_force_n=2.0 => жёсткость ~2 Н/ед,
    # это ниже SOFT_STIFFNESS_MAX_N (3.0) — эмулирует мягкий/хрупкий объект,
    # который физически не может дать больше ~2Н, сколько ни дави.
    driver = MockRH56DFTPDriver(hand="right", max_force_n=2.0, noise_std=0.0)
    driver.connect()

    # target_force_n=5.0 — цель, разумная для жёсткой кружки, но недостижимая
    # для этого объекта. Без определения жёсткости хват просто не сойдётся.
    grip = GripController(
        driver, target_force_n=5.0, max_force_n=8.0,
        soft_object_target_force_n=1.0, soft_object_max_force_n=2.0,
    )
    grip.reset()

    final_status = None
    for _ in range(300):
        pos, status = grip.grip_step()
        driver.close_hand(pos)
        final_status = status
        if status in ("stable", "overforce"):
            break

    assert final_status == "stable", \
        f"Ожидали stable на пониженной цели для мягкого объекта, получили: {final_status}"
    compliance = grip.compliance()
    assert compliance.classification == Compliance.SOFT, compliance


def test_behavior_tree_smoke():
    """Behavior tree с моками — должен дойти до ConfirmDone или упасть на FindCup."""
    from interfaces.unitree_sdk import MockG1Interface
    from perception.tactile.rh56dftp import MockRH56DFTPDriver, GripController
    from control.arm_controller import ArmController
    from action.handover import HandoverController
    from planning.behavior_tree import build_coffee_tree, Blackboard, Status

    robot = MockG1Interface()
    robot.connect()
    tactile = MockRH56DFTPDriver(hand="right")
    tactile.connect()
    arm = ArmController(robot, side="right")
    arm.enable()
    handover = HandoverController(robot, tactile, arm, timeout_s=1.0)

    tree = build_coffee_tree(
        robot=robot,
        detector=None,  # CV недоступен — FindCup упадёт
        tactile_driver=tactile,
        arm_ctrl=arm,
        handover_controller=handover,
        grip_controller_cls=GripController,
    )
    bb = Blackboard(goal={"action": "fetch", "object": "coffee", "target": "Oleg"})
    status = tree.tick(bb)
    # Detector=None → FindCup упадёт после 3 retry → tree FAILURE
    assert status == Status.FAILURE, f"Expected FAILURE (no CV), got {status}"


def test_safety_monitor():
    """SafetyMonitor стартует и стартовая проверка = OK."""
    from interfaces.unitree_sdk import MockG1Interface
    from perception.tactile.rh56dftp import MockRH56DFTPDriver
    from control.safety import SafetyMonitor, SafetyLevel

    robot = MockG1Interface()
    robot.connect()
    tactile = MockRH56DFTPDriver(hand="right")
    tactile.connect()

    mon = SafetyMonitor(robot, tactile)
    s = mon.check()
    assert s.level == SafetyLevel.OK, f"Expected OK, got {s.level}: {s.reasons}"


def test_arm_controller_mock():
    """ArmController в mock-режиме корректно обновляет состояние."""
    from interfaces.unitree_sdk import MockG1Interface
    from control.arm_controller import ArmController
    from control.arm_poses import get_pose

    robot = MockG1Interface()
    robot.connect()
    arm = ArmController(robot, side="right")
    arm.enable()

    arm.move_to_pose("pregrasp", duration=0.1)
    expected = get_pose("right", "pregrasp")
    actual = arm.current_pose
    for i, (e, a) in enumerate(zip(expected, actual)):
        assert abs(e - a) < 1e-3, f"Joint {i}: expected {e}, got {a}"


def test_handover_mock():
    """HandoverController работает с mock tactile."""
    from interfaces.unitree_sdk import MockG1Interface
    from perception.tactile.rh56dftp import MockRH56DFTPDriver
    from control.arm_controller import ArmController
    from action.handover import HandoverController, HandoverResult

    robot = MockG1Interface()
    robot.connect()
    tactile = MockRH56DFTPDriver(hand="right", max_force_n=3.0)
    tactile.connect()
    tactile.close_hand(0.7)  # держим чашку
    arm = ArmController(robot, side="right")
    arm.enable()

    h = HandoverController(robot, tactile, arm,
                            internal_force_drop_n=1.0,
                            timeout_s=5.0)
    # В фоновом потоке имитируем что человек забрал чашку ПОСЛЕ baseline
    # baseline читается после move_to_pose("handover") который длится 1.5с
    import threading
    def sim():
        time.sleep(2.5)  # ждём пока baseline прочитается
        # имитируем: человек берёт — grip_pos обнуляется
        tactile._grip_pos = 0.0
        tactile._external_force_n = 0.0
    threading.Thread(target=sim, daemon=True).start()
    result = h.execute()
    assert result.success, f"Expected success, got {result.reason}"


if __name__ == "__main__":
    # Запуск без pytest
    tests = [
        test_arm_poses, test_arm_poses_mirror, test_arm_poses_safe_limits,
        test_mock_tactile, test_mock_tactile_slip,
        test_compliance_estimator, test_grip_controller_soft_object_caps_force,
        test_behavior_tree_smoke, test_safety_monitor,
        test_arm_controller_mock, test_handover_mock,
    ]
    passed = 0
    failed = 0
    for t in tests:
        try:
            t()
            print(f"✅ {t.__name__}")
            passed += 1
        except Exception as e:
            print(f"❌ {t.__name__}: {e}")
            failed += 1
    print(f"\n{passed} passed, {failed} failed")
    sys.exit(0 if failed == 0 else 1)
