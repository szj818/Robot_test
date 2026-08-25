# robot_yolo_demo

本资料包用于第十一讲：机器视觉：深度学习目标检测。

请将本文件夹放到 Ubuntu 虚拟机的 home 目录下，最终路径为：

```text
~/robot_yolo_demo
```

## 目录说明

- `models/`：YOLO11n 预训练权重，课堂使用 `yolo11n.pt`
- `images/`：测试图片，课堂主图为 `soccer_practice_field.png`
- `labels/`：YOLO 标签格式样例
- `scripts/`：Python 推理脚本
- `outputs/`：推理结果输出目录

## 推荐运行流程

1. 安装 Miniconda。
2. 创建 `robot_yolo` 环境。
3. 安装 `ultralytics`。
4. 运行 CLI 推理。
5. 运行 Python 脚本读取检测结果。

## 常用命令

进入资料包目录：

```bash
cd ~/robot_yolo_demo
```

激活环境：

```bash
conda activate robot_yolo
```

运行单图推理：

```bash
yolo predict model=models/yolo11n.pt source=images/soccer_practice_field.png conf=0.25 project=outputs name=cli_predict exist_ok=True
```

运行 Python 脚本：

```bash
python scripts/detect_image.py
```

说明：足球在 YOLO11n 预训练模型中通常输出为 `sports ball`，不是 `football`。
