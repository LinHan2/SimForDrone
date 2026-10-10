"""Conservative RGB-D target center extraction around a calibrated search hint."""

import math

import cv2
import numpy as np


def detect_target_pixel(rgb, depth, expected_uv, expected_depth, *, radius=90, depth_tolerance=1.0):
    if (rgb.ndim != 3 or rgb.shape[2] != 3 or depth.shape != rgb.shape[:2]
            or not np.isfinite(expected_uv).all() or not math.isfinite(expected_depth)
            or expected_depth <= 0):
        return None
    height, width = depth.shape
    u, v = expected_uv
    left, right = max(0, int(u) - radius), min(width, int(u) + radius + 1)
    top, bottom = max(0, int(v) - radius), min(height, int(v) + radius + 1)
    if right - left < 12 or bottom - top < 12:
        return None
    patch = np.asarray(depth[top:bottom, left:right], dtype=np.float32)
    candidate = np.isfinite(patch) & (patch > 0) & (np.abs(patch - expected_depth) <= depth_tolerance)
    if candidate.sum() < 25 or candidate.mean() > 0.65:
        return None
    labels_count, labels, stats, centers = cv2.connectedComponentsWithStats(candidate.astype(np.uint8), 8)
    matches = []
    for index in range(1, labels_count):
        x, y, box_width, box_height, area = stats[index]
        if not 25 <= area <= patch.size * 0.45 or box_width < 4 or box_height < 4:
            continue
        if x == 0 or y == 0 or x + box_width == patch.shape[1] or y + box_height == patch.shape[0]:
            continue
        pixel = np.array((centers[index, 0] + left, centers[index, 1] + top))
        if np.linalg.norm(pixel - expected_uv) <= radius * 0.65:
            matches.append((float(np.linalg.norm(pixel - expected_uv)), pixel, index))
    if len(matches) != 1:
        return None
    _, pixel, index = matches[0]
    selected = labels == index
    foreground = rgb[top:bottom, left:right][selected].astype(np.float32)
    background = rgb[top:bottom, left:right][~candidate].astype(np.float32)
    if not len(background) or np.linalg.norm(foreground.mean(axis=0) - background.mean(axis=0)) < 12:
        return None
    return pixel.tolist()