#!/usr/bin/env python3
"""对已保存的 observer RGB 样本做基础可视性检查。

该检查评估曝光、对比度、边缘密度和颜色多样性；它不能证明 target 已在画面中，
更不能替代后续 6D pose estimator 的精度评测。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image


def evaluate(rgb_path: Path) -> dict[str, float | bool | str]:
    """从单张 RGB 图像提取不依赖 OpenCV 的保守质量指标。"""

    rgb = np.asarray(Image.open(rgb_path).convert("RGB"), dtype=np.float32)
    gray = rgb @ np.asarray((0.299, 0.587, 0.114), dtype=np.float32)
    p05, p50, p95 = np.percentile(gray, (5.0, 50.0, 95.0))

    # 仅统计一阶像素差，避免在服务器端为一次验收额外安装 OpenCV。
    gradient = np.maximum(np.abs(np.diff(gray, axis=0, prepend=gray[:1])), np.abs(np.diff(gray, axis=1, prepend=gray[:, :1])))
    edge_ratio = float(np.mean(gradient >= 18.0))
    saturation = (rgb.max(axis=2) - rgb.min(axis=2)) / np.maximum(rgb.max(axis=2), 1.0)
    saturation_ratio = float(np.mean(saturation >= 0.12))

    readable = bool(
        p05 >= 4.0
        and p95 <= 251.0
        and p95 - p05 >= 38.0
        and edge_ratio >= 0.018
        and saturation_ratio >= 0.008
    )
    return {
        "image": str(rgb_path),
        "width": int(rgb.shape[1]),
        "height": int(rgb.shape[0]),
        "gray_p05": round(float(p05), 3),
        "gray_median": round(float(p50), 3),
        "gray_p95": round(float(p95), 3),
        "contrast_p95_minus_p05": round(float(p95 - p05), 3),
        "edge_ratio": round(edge_ratio, 5),
        "saturated_color_ratio": round(saturation_ratio, 5),
        "basic_observer_rgb_readable": readable,
        "six_d_pose_ready": "not_assessed: target visibility and pose accuracy need a separate evaluation",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("rgb_png", type=Path, help="capture_rgbd_sample.py 保存的 rgb.png")
    args = parser.parse_args()
    if not args.rgb_png.is_file():
        raise FileNotFoundError(f"找不到 RGB 样本: {args.rgb_png}")
    print(json.dumps(evaluate(args.rgb_png), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
