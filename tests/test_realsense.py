"""
tests/test_realsense.py
───────────────────────
Подключение Intel RealSense D435, чтение intrinsics, тест depth.
"""
import json
import sys
import time
from pathlib import Path


def test_realsense():
    print("[1/5] Импорт pyrealsense2...")
    try:
        import pyrealsense2 as rs
        import numpy as np
        print("  OK")
    except ImportError as e:
        print(f"  FAIL: {e}")
        print("  pip install pyrealsense2")
        return False

    print("[2/5] Поиск устройств...")
    ctx = rs.context()
    devices = list(ctx.query_devices())
    if not devices:
        print("  FAIL: RealSense не найдена. Проверь USB 3.0 подключение")
        return False
    for d in devices:
        print(f"  Found: {d.get_info(rs.camera_info.name)} (S/N: {d.get_info(rs.camera_info.serial_number)})")

    print("[3/5] Запуск pipeline (1280x720 @30fps)...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    try:
        pipeline.start(config)
        print("  OK")
    except Exception as e:
        print(f"  FAIL: {e}")
        return False

    print("[4/5] Чтение intrinsics (СОХРАНЯЕМ В ФАЙЛ)...")
    profile = pipeline.get_active_profile()
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    intrinsics = {
        'fx': float(intr.fx),
        'fy': float(intr.fy),
        'ppx': float(intr.ppx),
        'ppy': float(intr.ppy),
        'coeffs': list(intr.coeffs),
        'width': intr.width,
        'height': intr.height,
        'model': str(intr.model),
    }
    Path("configs").mkdir(exist_ok=True)
    Path("configs/realsense_intrinsics.json").write_text(
        json.dumps(intrinsics, indent=2), encoding="utf-8"
    )
    print(f"  fx={intrinsics['fx']:.1f} fy={intrinsics['fy']:.1f} ppx={intrinsics['ppx']:.1f} ppy={intrinsics['ppy']:.1f}")
    print(f"  → configs/realsense_intrinsics.json")

    print("[5/5] Тест depth (5 секунд)...")
    align = rs.align(rs.stream.color)
    t0 = time.time()
    samples = []
    while time.time() - t0 < 5:
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        depth = aligned.get_depth_frame()
        if not depth:
            continue
        # центр кадра
        cx, cy = 640, 360
        d = depth.get_distance(cx, cy)
        if d > 0:
            samples.append(d)
        time.sleep(0.1)

    pipeline.stop()

    if samples:
        avg = sum(samples) / len(samples)
        print(f"  Средняя depth в центре: {avg:.3f} м (из {len(samples)} сэмплов)")
        if 0.3 < avg < 5.0:
            print("\n✅ REALSENSE РАБОТАЕТ!")
            return True
        else:
            print(f"\n⚠️  Depth вне ожидаемого диапазона 0.3-5м: {avg:.3f}м")
            print("   Наведи камеру на объект в 1-2 метрах")
            return False
    else:
        print("\n❌ Не получили ни одного depth-сэмпла. Возможно, освещение плохое")
        return False


if __name__ == "__main__":
    success = test_realsense()
    sys.exit(0 if success else 1)
