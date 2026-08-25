"""
YOLO11n 对照推理脚本
- 与 faster_rcnn/detect_image.py 使用相同的两张图片, 便于直接对比
- 置信度阈值同样设为 0.25
- 记录每张图推理耗时与检测框
"""
import time
from pathlib import Path

from ultralytics import YOLO
from torchvision.io import read_image
from torchvision.utils import draw_bounding_boxes, save_image

CONF_THRESH = 0.25


def run(model, image_path: Path, out_dir: Path):
    print(f"\n=== image: {image_path.name} ===")
    t0 = time.time()
    results = model.predict(
        source=str(image_path),
        conf=CONF_THRESH,
        save=False,
        verbose=False,
    )
    dt = time.time() - t0
    print(f"inference time: {dt*1000:.1f} ms (CPU, single image)")

    r = results[0]
    names = r.names
    boxes = r.boxes
    print(f"detections (conf>{CONF_THRESH}): {len(boxes)}")
    print("-" * 80)
    for i, box in enumerate(boxes):
        cid = int(box.cls[0])
        cname = names[cid]
        s = float(box.conf[0])
        x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
        print(f"[{i}] class_name={cname}")
        print(f"    confidence={s:.3f}")
        print(f"    bbox_xyxy=({x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f})")
    print("-" * 80)

    # 绘制可视化
    img = read_image(str(image_path))
    if len(boxes) > 0:
        tb = boxes.xyxy
        tl = [f"{names[int(c)]} {float(s):.2f}" for c, s in zip(boxes.cls, boxes.conf)]
        drawn = draw_bounding_boxes(img, tb, labels=tl, width=4)
    else:
        drawn = img
    out_path = out_dir / f"{image_path.stem}_yolo.jpg"
    save_image(drawn / 255.0, str(out_path))
    print(f"visual result saved to: {out_path.relative_to(out_dir.parent.parent)}")
    return dt, len(boxes)


def main():
    base = Path(__file__).resolve().parents[1]  # robot_yolo_demo/
    img_dir = base / "images"
    out_dir = base / "outputs/yolo_compare"
    out_dir.mkdir(parents=True, exist_ok=True)
    model_path = base / "models/yolo11n.pt"
    model = YOLO(str(model_path))

    images = [
        img_dir / "soccer_practice_field.png",
        img_dir / "soccer_studio_closeup.png",
    ]

    times, counts = [], []
    for p in images:
        if not p.exists():
            print(f"SKIP missing: {p}")
            continue
        dt, n = run(model, p, out_dir)
        times.append(dt)
        counts.append(n)

    print("\n=== summary ===")
    for p, dt, n in zip(images, times, counts):
        print(f"{p.name}: {n} boxes, {dt*1000:.1f} ms")


if __name__ == "__main__":
    main()
