"""
tests/test_yolo_cup.py
──────────────────────
Тест YOLOv8m COCO-pretrained на детекцию чашки.
Поднеси чашку к камере — должна детектиться с conf > 0.5.
"""
import sys
import time


def test_yolo():
    print("[1/3] Импорт ultralytics...")
    try:
        from ultralytics import YOLO
        import torch
        print(f"  OK (torch CUDA: {torch.cuda.is_available()})")
    except ImportError as e:
        print(f"  FAIL: {e}")
        print("  pip install ultralytics")
        return False

    print("[2/3] Загрузка YOLOv8m COCO-pretrained...")
    try:
        model = YOLO("yolov8m.pt")
        device = "cuda:0" if torch.cuda.is_available() else "cpu"
        print(f"  OK на {device}")
    except Exception as e:
        print(f"  FAIL: {e}")
        return False

    # COCO класс 'cup' = 41
    print("[3/3] Тест на кадре с чашкой...")
    import cv2
    import numpy as np

    cap = cv2.VideoCapture(0)  # веб-камера или RealSense color
    if not cap.isOpened():
        print("  FAIL: не удалось открыть камеру")
        return False

    print("  Наведи камеру на чашку. Нажми 'q' для выхода, 's' для скриншота.")
    detections_count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        t0 = time.time()
        results = model.predict(
            source=frame,
            device=device,
            conf=0.4,
            classes=[41],  # только 'cup'
            verbose=False,
        )
        dt = time.time() - t0

        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                conf = float(box.conf[0])
                detections_count += 1
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"cup {conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.putText(frame, f"FPS: {1/dt:.1f} | detections: {detections_count}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.imshow("YOLOv8 cup test (press q to quit)", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            import datetime
            fname = f"cup_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg"
            cv2.imwrite(fname, frame)
            print(f"  Скриншот сохранён: {fname}")

    cap.release()
    cv2.destroyAllWindows()

    if detections_count > 5:
        print(f"\n✅ YOLO РАБОТАЕТ: {detections_count} детекций за сессию")
        return True
    else:
        print(f"\n⚠️  Мало детекций: {detections_count}")
        print("   Попробуй:")
        print("   - Лучше освещение")
        print("   - Чашка ближе к камере (30-50 см)")
        print("   - Дообучить модель на своих фото (см. cup_dataset/)")
        return False


if __name__ == "__main__":
    success = test_yolo()
    sys.exit(0 if success else 1)
