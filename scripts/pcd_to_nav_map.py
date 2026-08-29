#!/usr/bin/env python3
"""Create a conservative Nav2 occupancy layer from an XYZ binary PCD map."""

import argparse
import json
import math
from pathlib import Path

import numpy as np


def read_xyz_pcd(path: Path) -> np.ndarray:
    data = path.read_bytes()
    marker = b"DATA binary\n"
    offset = data.find(marker)
    if offset < 0:
        raise ValueError("只支持DATA binary格式的XYZ PCD")
    header = data[:offset].decode("ascii", errors="replace")
    if "FIELDS x y z" not in header or "SIZE 4 4 4" not in header:
        raise ValueError("PCD必须包含float32 x/y/z字段")
    payload = data[offset + len(marker):]
    if len(payload) % 12:
        raise ValueError("PCD点数据长度不是12字节的整数倍")
    points = np.frombuffer(payload, dtype="<f4").reshape(-1, 3)
    return points[np.isfinite(points).all(axis=1)]


def shifted(mask: np.ndarray, dy: int, dx: int) -> np.ndarray:
    result = np.zeros_like(mask)
    ys = slice(max(0, dy), mask.shape[0] + min(0, dy))
    xs = slice(max(0, dx), mask.shape[1] + min(0, dx))
    source_y = slice(max(0, -dy), mask.shape[0] - max(0, dy))
    source_x = slice(max(0, -dx), mask.shape[1] - max(0, dx))
    result[ys, xs] = mask[source_y, source_x]
    return result


def dilate(mask: np.ndarray, radius: int) -> np.ndarray:
    result = np.zeros_like(mask)
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy <= radius * radius:
                result |= shifted(mask, dy, dx)
    return result


def create_map(points: np.ndarray, resolution: float, margin: float):
    if len(points) < 100:
        raise ValueError("PCD点数过少")
    # Ignore isolated long-range returns so one outlier cannot create a huge map.
    low = np.percentile(points[:, :2], 0.2, axis=0)
    high = np.percentile(points[:, :2], 99.8, axis=0)
    keep = ((points[:, :2] >= low) & (points[:, :2] <= high)).all(axis=1)
    points = points[keep]
    xmin = math.floor((points[:, 0].min() - margin) / resolution) * resolution
    ymin = math.floor((points[:, 1].min() - margin) / resolution) * resolution
    xmax = math.ceil((points[:, 0].max() + margin) / resolution) * resolution
    ymax = math.ceil((points[:, 1].max() + margin) / resolution) * resolution
    width = int(round((xmax - xmin) / resolution)) + 1
    height = int(round((ymax - ymin) / resolution)) + 1
    if width * height > 8_000_000:
        raise ValueError(f"导航网格过大：{width}x{height}")

    gx = np.clip(((points[:, 0] - xmin) / resolution).astype(np.int64), 0, width - 1)
    gy = np.clip(((points[:, 1] - ymin) / resolution).astype(np.int64), 0, height - 1)
    flat = gy * width + gx
    cells = width * height
    minimum = np.full(cells, np.inf, dtype=np.float32)
    np.minimum.at(minimum, flat, points[:, 2])
    base = minimum[flat]
    relative = points[:, 2] - base
    ground_hits = np.zeros(cells, dtype=np.uint16)
    obstacle_hits = np.zeros(cells, dtype=np.uint16)
    np.add.at(ground_hits, flat[relative <= 0.14], 1)
    # Ignore the ceiling; only geometry intersecting the dog's body height is lethal.
    obstacle_band = (relative >= 0.18) & (relative <= 1.20)
    np.add.at(obstacle_hits, flat[obstacle_band], 1)
    ground = (ground_hits.reshape(height, width) >= 2)
    # Require repeated vertical returns in one cell.  Two-hit speckles from
    # moving legs / grazing ground returns created false walls in the 0922 map;
    # real walls and solid objects remain densely sampled by MID360.
    obstacle = (obstacle_hits.reshape(height, width) >= 3)

    # Reject isolated returns before inflation. Real walls/objects have spatial
    # support, while multipath and moving-point noise usually occupy one cell.
    support = np.zeros_like(obstacle, dtype=np.uint8)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            support += shifted(obstacle, dy, dx)
    obstacle &= support >= 3

    # Bridge tiny sampling holes, but never paint through an observed obstacle.
    free = dilate(ground, 1) & ~obstacle
    # Keep the static extraction at measured geometry.  Nav2 applies the real
    # GO2 footprint and its own inflation layer at planning time; dilating here
    # as well double-counted clearance and disconnected valid corridors.
    occupied = obstacle
    image = np.full((height, width), 205, dtype=np.uint8)  # unknown
    image[free] = 254
    image[occupied] = 0
    return image, xmin, ymin, points, free, occupied


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("pcd", type=Path)
    parser.add_argument("--resolution", type=float, default=0.10)
    parser.add_argument("--margin", type=float, default=0.50)
    args = parser.parse_args()
    points = read_xyz_pcd(args.pcd)
    image, xmin, ymin, used, free, occupied = create_map(points, args.resolution, args.margin)
    pgm = args.pcd.with_suffix(".pgm")
    yaml_path = args.pcd.with_suffix(".yaml")
    # PGM row zero is the top; occupancy-grid Y increases upward.
    raster = np.flipud(image)
    pgm.write_bytes(f"P5\n{raster.shape[1]} {raster.shape[0]}\n255\n".encode("ascii") + raster.tobytes())
    yaml_path.write_text(
        f"image: {pgm.name}\nmode: trinary\nresolution: {args.resolution:.6f}\n"
        f"origin: [{xmin:.6f}, {ymin:.6f}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n",
        encoding="utf-8",
    )
    result = {
        "yaml": str(yaml_path), "pgm": str(pgm), "resolution": args.resolution,
        "width": int(image.shape[1]), "height": int(image.shape[0]),
        "origin": [xmin, ymin], "source_points": int(len(used)),
        "free_cells": int(free.sum()), "occupied_cells": int(occupied.sum()),
    }
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
