"""
scripts/run_test_vision.py
──────────────────────────
Тест YOLOv8 на детекцию чашки.

Запуск:
    python scripts/run_test_vision.py                 # веб-камера 0
    python scripts/run_test_vision.py --source 1      # камера 1
    python scripts/run_test_vision.py --model yolov8s.pt
    python scripts/run_test_vision.py --realsense     # Intel RealSense D435
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--source", default="0",
                   help="Camera index или путь к видео (default: 0)")
    p.add_argument("--model", default="yolov8m.pt",
                   help="YOLOv8 model (yolov8n/s/m/l/x)")
    p.add_argument("--realsense", action="store_true",
                   help="Использовать Intel RealSense D435 + depth")
    p.add_argument("--conf", type=float, default=0.5)
    p.add_argument("--out", default=None,
                   help="Сохранить аннотированный кадр в файл")
    args = p.parse_args()

    try:
        import cv2
        import numpy as np
        from ultralytics import YOLO
        import torch
    except ImportError as e:
        print(f"❌ Не хватает зависимостей: {e}")
        print("   pip install ultralytics opencv-python torch numpy")
        sys.exit(1)

    from perception.vision.detector import CupDetector, find_best_cup, TARGET_CLASSES

    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    print(f"[vision] Loading {args.model} on {device}...")
    detector = CupDetector(model_path=args.model, device=device,
                           conf_threshold=args.conf)
    print(f"[vision] Loaded. Target classes: {TARGET_CLASSES}")

    # RealSense
    rs_pipeline = None
    rs_intrinsics = None
    if args.realsense:
        try:
            import pyrealsense2 as rs
            rs_pipeline = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
            cfg.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
            rs_pipeline.start(cfg)
            align = rs.align(rs.stream.color)
            profile = rs_pipeline.get_active_profile()
            intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            rs_intrinsics = {"fx": intr.fx, "fy": intr.fy, "ppx": intr.ppx, "ppy": intr.ppy,
                             "coeffs": list(intr.coeffs)}
            print(f"[vision] RealSense intrinsics: fx={intr.fx:.1f}, fy={intr.fy:.1f}")
        except ImportError:
            print("[vision] pyrealsense2 не установлен — fallback на веб-камеру")
            rs_pipeline = None

    # Камера
    cap = None
    if rs_pipeline is None:
        try:
            source = int(args.source)
        except ValueError:
            source = args.source
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            print(f"❌ Не удалось открыть камеру: {args.source}")
            sys.exit(1)

    print("\n[vision] Запуск. Наведи на чашку. ESC для выхода.\n")

    detections_count = 0
    fps_avg = 0.0
    try:
        while True:
            t0 = time.time()

            if rs_pipeline is not None:
                frames = rs_pipeline.wait_for_frames()
                aligned = rs.align(rs.stream.color).process(frames)
                color_frame = aligned.get_color_frame()
                depth_frame = aligned.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue
                rgb = np.asanyarray(color_frame.get_data())
                depth = np.asanyarray(depth_frame.get_data())
                dets = detector.detect_with_depth(rgb, depth, intrinsics=rs_intrinsics)
            else:
                ret, rgb = cap.read()
                if not ret:
                    print("[vision] Конец потока")
                    break
                dets = detector.detect(rgb)

            dt = time.time() - t0
            fps_avg = 0.9 * fps_avg + 0.1 * (1.0 / max(1e-3, dt))

            # Рисуем детекции
            frame = rgb.copy() if hasattr(rgb, 'copy') else rgb
            for d in dets:
                x1, y1, x2, y2 = d.bbox_xyxy
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{d.cls_name} {d.confidence:.2f}"
                if d.xyz_m is not None:
                    label += f" ({d.xyz_m[0]:.2f},{d.xyz_m[1]:.2f},{d.xyz_m[2]:.2f})m"
                cv2.putText(frame, label, (x1, max(0, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                if d.cls_id == 41:  # cup
                    detections_count += 1

            cv2.putText(frame, f"FPS: {fps_avg:.1f} | cups: {detections_count}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow("Kuzmich vision test (ESC to quit)", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:
                break
            if key == ord('s') and args.out:
                cv2.imwrite(args.out, frame)
                print(f"  Saved: {args.out}")
    except KeyboardInterrupt:
        pass
    finally:
        if cap is not None:
            cap.release()
        if rs_pipeline is not None:
            rs_pipeline.stop()
        cv2.destroyAllWindows()

    print(f"\n[vision] Сессия завершена. Детекций чашки: {detections_count}")


if __name__ == "__main__":
    main()
