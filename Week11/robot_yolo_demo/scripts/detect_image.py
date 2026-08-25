from pathlib import Path

from ultralytics import YOLO


def main():
    base_dir = Path(__file__).resolve().parents[1]
    model_path = base_dir / "models/yolo11n.pt"
    image_path = base_dir / "images/soccer_practice_field.png"
    output_dir = base_dir / "outputs/python_predict"

    model = YOLO(model_path)
    results = model.predict(
        source=str(image_path),
        conf=0.25,
        save=True,
        project=str(output_dir.parent.resolve()),
        name=output_dir.name,
        exist_ok=True,
    )

    result = results[0]
    names = result.names

    print("image:", image_path.relative_to(base_dir))
    print("detections:", len(result.boxes))
    print("-" * 80)

    for i, box in enumerate(result.boxes):
        cls_id = int(box.cls[0])
        class_name = names[cls_id]
        confidence = float(box.conf[0])
        x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]

        print(f"[{i}] class_name={class_name}")
        print(f"    confidence={confidence:.3f}")
        print(f"    bbox_xyxy=({x1:.1f}, {y1:.1f}, {x2:.1f}, {y2:.1f})")

    print("-" * 80)
    print("visual result saved to:", output_dir.relative_to(base_dir))


if __name__ == "__main__":
    main()
