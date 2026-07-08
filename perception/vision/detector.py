"""
perception/vision/detector.py
─────────────────────────────
YOLOv8 detector для чашек/кружек/бутылок.

Стратегия:
1. Сначала тестим COCO-pretrained YOLOv8m на классах 'cup' (41), 'bottle' (39),
   'wine glass' (40), 'bowl' (45). Если mAP ≥ 0.7 — оставляем.
2. Если точность хуёвая — собираем 500-1000 фото кофейных чашек (разные ракурсы,
   освещение, столы), дообучаем YOLOv8m на 50 эпох.
3. Для каждой детекции берём bbox центр, через RealSense получаем depth → 3D.
"""
from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np


@dataclass
class Detection:
    cls_id: int
    cls_name: str
    confidence: float
    bbox_xyxy: tuple[int, int, int, int]  # x1, y1, x2, y2 в пикселях
    center_xy: tuple[int, int]
    depth_m: Optional[float] = None       # глубина в метрах (от RealSense)
    xyz_m: Optional[tuple[float, float, float]] = None  # 3D в системе координат камеры


# COCO классы, которые нам интересны
TARGET_CLASSES = {
    41: "cup",          # кружки, чашки
    39: "bottle",       # бутылки (если попросили воду/молоко)
    40: "wine_glass",   # бокалы (для сока)
    45: "bowl",         # пиалы (для супа)
    47: "scissors",     # на всякий случай
    73: "book",         # если Олежа просит газету
    65: "remote",       # пульт
}


class CupDetector:
    """YOLOv8-based detector для чашек и родственных объектов."""

    def __init__(
        self,
        model_path: str = "yolov8m.pt",     # COCO-pretrained
        device: str = "cuda:0",              # RTX 4060 / Jetson Orin
        conf_threshold: float = 0.5,
        iou_threshold: float = 0.45,
    ):
        from ultralytics import YOLO
        self.model = YOLO(model_path)
        self.device = device
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold

    def detect(self, rgb: np.ndarray) -> list[Detection]:
        """Детекция в RGB-кадре. Возвращает список Detection без depth."""
        results = self.model.predict(
            source=rgb,
            device=self.device,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            classes=list(TARGET_CLASSES.keys()),
            verbose=False,
        )
        out: list[Detection] = []
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                out.append(Detection(
                    cls_id=cls_id,
                    cls_name=TARGET_CLASSES.get(cls_id, str(cls_id)),
                    confidence=float(box.conf[0]),
                    bbox_xyxy=(int(x1), int(y1), int(x2), int(y2)),
                    center_xy=(int(cx), int(cy)),
                ))
        return out

    def detect_with_depth(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,           # RealSense depth frame (aligned to color), mm
        depth_scale: float = 0.001,  # обычно 0.001 (mm → m)
        intrinsics: Optional[dict] = None,
    ) -> list[Detection]:
        """
        Детекция + 3D координаты через RealSense depth.

        ⚠️ intrinsics УСТАНАВЛИВАЮТСЯ НА ЗАВОДЕ ДЛЯ КАЖДОГО ЭКЗЕМПЛЯРА D435.
        НЕ хардкодь — читай с устройства через pyrealsense2:
            profile = pipeline.get_active_profile()
            intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            intrinsics = {'fx': intr.fx, 'fy': intr.fy, 'ppx': intr.ppx, 'ppy': intr.ppy, 'coeffs': intr.coeffs}

        Для dev/test можно передать None — используем типичные D435 значения
        (но в реале ВЫНУЖДЕННО будут другие!).

        depth должен быть ALIGNED to color (rs.align(rs.stream.color)).
        """
        if intrinsics is None:
            # Типичные значения D435 @ 1280x720, но НАДО ЧИТАТЬ С УСТРОЙСТВА
            intrinsics = {"fx": 915.0, "fy": 915.0, "ppx": 640, "ppy": 360, "coeffs": [0, 0, 0, 0, 0]}
        fx, fy = intrinsics["fx"], intrinsics["fy"]
        ppx, ppy = intrinsics["ppx"], intrinsics["ppy"]

        dets = self.detect(rgb)
        for d in dets:
            cx, cy = d.center_xy
            # Усредняем depth в окне 5×5 вокруг центра (боремся с шумом)
            x0, x1 = max(0, cx - 2), min(rgb.shape[1], cx + 3)
            y0, y1 = max(0, cy - 2), min(rgb.shape[0], cy + 3)
            patch = depth[y0:y1, x0:x1]
            valid = patch[patch > 0]
            if len(valid) == 0:
                continue
            z_mm = float(np.median(valid))
            z_m = z_mm * depth_scale
            d.depth_m = z_m
            # Проекция в 3D (в системе координат камеры):
            # x вправо, y вниз, z вперёд (оптическая ось)
            x_m = (cx - ppx) * z_m / fx
            y_m = (cy - ppy) * z_m / fy
            d.xyz_m = (x_m, y_m, z_m)
        return dets


def find_best_cup(detections: list[Detection]) -> Optional[Detection]:
    """Из всех детекций выбираем самую уверенную чашку (cls=41)."""
    cups = [d for d in detections if d.cls_id == 41]
    if not cups:
        return None
    return max(cups, key=lambda d: d.confidence)


# ─── Камера → база робота ───────────────────────────────────────────────
#
# ⚠️ PLACEHOLDER-калибровка. Предполагает RealSense, жёстко закреплённый на
# груди/голове G1, смотрящий строго вперёд без наклона/крена. Реальные
# значения (высота крепления, смещение вперёд, поправка на наклон камеры)
# нужно снять на физическом роботе и положить сюда или в отдельный
# configs/camera_extrinsics.json (по аналогии с configs/realsense_intrinsics.json).
CAMERA_MOUNT_HEIGHT_M = 1.1      # высота объектива камеры над полом/базой робота
CAMERA_FORWARD_OFFSET_M = 0.05   # смещение камеры вперёд от базы робота


def camera_to_robot_frame(
    xyz_camera: tuple[float, float, float],
) -> tuple[float, float, float]:
    """
    Переводит координаты объекта из системы камеры (x=вправо, y=вниз,
    z=вперёд/depth — см. Detection.xyz_m) в систему базы робота
    (x=вперёд, y=влево, z=высота от пола), которую ждут robot.move_to()
    и ArmController.move_to_xyz() (см. их докстринги).

    Без этой трансформации depth (реальное расстояние до объекта) никогда
    не попадает в "вперёд" для навигации/руки — раньше он терялся.
    """
    cx, cy, cz = xyz_camera
    robot_x = cz + CAMERA_FORWARD_OFFSET_M
    robot_y = -cx
    robot_z = CAMERA_MOUNT_HEIGHT_M - cy
    return (robot_x, robot_y, robot_z)


# ─── Mock для dev/test (без torch/ultralytics/камеры) ────────────────────

class MockCupDetector:
    """Симуляция CupDetector — та же сигнатура (detect/detect_with_depth),
    но всегда «видит» одну чашку в заданной точке (система камеры:
    x=вправо, y=вниз, z=depth), без torch/ultralytics и без реальной камеры.

    Нужен чтобы FindCup/ApproachCup/GraspCup можно было прогнать в
    scripts/run_e2e_demo.py --mock целиком, а не падать на "нет детектора".
    """

    def __init__(
        self,
        fake_cup_camera_xyz: tuple[float, float, float] = (0.05, 0.15, 0.6),
        confidence: float = 0.9,
    ):
        self.fake_cup_camera_xyz = fake_cup_camera_xyz
        self.confidence = confidence
        self.device = "mock"

    def detect(self, rgb: np.ndarray) -> list[Detection]:
        return self._fake_detection()

    def detect_with_depth(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
        depth_scale: float = 0.001,
        intrinsics: Optional[dict] = None,
    ) -> list[Detection]:
        return self._fake_detection()

    def _fake_detection(self) -> list[Detection]:
        x, y, z = self.fake_cup_camera_xyz
        return [Detection(
            cls_id=41,
            cls_name="cup",
            confidence=self.confidence,
            bbox_xyxy=(300, 200, 340, 260),
            center_xy=(320, 230),
            depth_m=z,
            xyz_m=(x, y, z),
        )]


# ─── Тест ────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import cv2
    import sys

    detector = CupDetector(model_path="yolov8m.pt", device="cuda:0" if __import__("torch").cuda.is_available() else "cpu")
    print(f"Loaded YOLOv8m on {detector.device}")

    # Тест на камере или файле
    source = sys.argv[1] if len(sys.argv) > 1 else 0
    cap = cv2.VideoCapture(source)

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        t0 = time.time()
        dets = detector.detect(frame)
        dt = time.time() - t0

        for d in dets:
            x1, y1, x2, y2 = d.bbox_xyxy
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{d.cls_name} {d.confidence:.2f}"
            if d.xyz_m:
                label += f" ({d.xyz_m[0]:.2f}, {d.xyz_m[1]:.2f}, {d.xyz_m[2]:.2f})m"
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.putText(frame, f"FPS: {1/dt:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("Kuzmich vision", frame)
        if cv2.waitKey(1) == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
