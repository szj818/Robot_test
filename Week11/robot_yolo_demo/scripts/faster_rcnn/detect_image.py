"""
Faster R-CNN (ResNet-50-FPN V2) 目标检测推理脚本
- 模型来源: torchvision 官方 COCO 预训练权重
- 与课程 YOLO11n (单阶段) 形成对比: 本方法为两阶段检测器
"""
import time
from pathlib import Path

import torch
from torchvision.io import read_image
from torchvision.models.detection import (
    fasterrcnn_resnet50_fpn_v2,
    FasterRCNN_ResNet50_FPN_V2_Weights,
)
from torchvision.utils import draw_bounding_boxes

# COCO 91 类 -> 可读名称 (0 为 __background__)
# torchvision 直接提供 weights.meta['categories']
WEIGHTS = FasterRCNN_ResNet50_FPN_V2_Weights.DEFAULT
CATEGORIES = WEIGHTS.meta["categories"]
# 去掉背景
NUM_CLASSES = len(CATEGORIES)  # 91, index 0 = __background__

CONF_THRESH = 0.25  # 与课程 YOLO 推理保持一致


def box_label(name: str, conf: float) -> str:
    return f"{name} {conf:.2f}"


def run(image_path: Path, out_dir: Path, device: str):
    print(f"\n=== image: {image_path.name} ===")
    img = read_image(str(image_path))  # C,H,W uint8
    # 预处理: 官方 weights.transforms() 会做归一化, 但 torchvision 的 detection
    # 模型在 eval 模式下直接接受 uint8 [C,H,W] 张量 (内部会处理),
    # 为简单与稳定, 直接喂入原始图像张量.
    img_in = img.unsqueeze(0).to(device).to(torch.float32) / 255.0

    model = fasterrcnn_resnet50_fpn_v2(weights=WEIGHTS).to(device)
    model.eval()
    torch.manual_seed(0)

    # warmup (一次不计入计时, 避免首次开销影响速度对比)
    with torch.no_grad():
        _ = model(img_in)

    t0 = time.time()
    with torch.no_grad():
        out = model(img_in)[0]
    dt = time.time() - t0
    print(f"inference time: {dt*1000:.1f} ms (CPU, single image)")

    boxes = out["boxes"]
    scores = out["scores"]
    labels = out["labels"]

    keep = scores > CONF_THRESH
    boxes = boxes[keep]
    scores = scores[keep]
    labels = labels[keep]

    print(f"detections (conf>{CONF_THRESH}): {len(boxes)}")
    print("-" * 80)
    for i in range(boxes.shape[0]):
        cid = int(labels[i])
        cname = CATEGORIES[cid] if cid < NUM_CLASSES else str(cid)
        s = float(scores[i])
        x1, y1, x2, y2 = [float(v) for v in boxes[i]]
        print(f"[{i}] class_name={cname}")
        print(f"    confidence={s:.3f}")
        print(f"    bbox_xyxy=({x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f})")
    print("-" * 80)

    # 绘制可视化
    label_texts = [
        box_label(
            CATEGORIES[int(l)] if int(l) < NUM_CLASSES else str(int(l)),
            float(s),
        )
        for l, s in zip(labels, scores)
    ]
    drawn = draw_bounding_boxes(
        img, boxes, labels=label_texts, width=4
    )
    out_path = out_dir / f"{image_path.stem}_frcnn.jpg"
    from torchvision.utils import save_image
    save_image(drawn / 255.0, str(out_path))
    print(f"visual result saved to: {out_path.relative_to(out_dir.parent.parent)}")
    return dt, len(boxes)


def main():
    base = Path(__file__).resolve().parents[2]  # robot_yolo_demo/
    img_dir = base / "images"
    out_dir = base / "outputs/faster_rcnn"
    out_dir.mkdir(parents=True, exist_ok=True)
    device = "cpu"

    images = [
        img_dir / "soccer_practice_field.png",  # 必含
        img_dir / "soccer_studio_closeup.png",  # 第二张
    ]

    times = []
    counts = []
    for p in images:
        if not p.exists():
            print(f"SKIP missing: {p}")
            continue
        dt, n = run(p, out_dir, device)
        times.append(dt)
        counts.append(n)

    print("\n=== summary ===")
    for p, dt, n in zip(images, times, counts):
        print(f"{p.name}: {n} boxes, {dt*1000:.1f} ms")


if __name__ == "__main__":
    main()
