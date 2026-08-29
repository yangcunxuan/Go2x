#!/usr/bin/env python3
"""Generate local_planner/paths/correspondences.txt.

Replicates the correspondence step of paths/path_generator.m (MATLAB) in
Python, because Go2_planner_suite committed the .ply path files but not the
generated correspondences.txt that localPlanner requires at startup.

For every voxel of the 161 x 451 local planning grid it records the sorted,
deduplicated list of precomputed path IDs passing within searchRadius.
Vectorized with numpy: the reference pure-Python version needed >15 min on
the validation laptop.
"""

import numpy as np
from pathlib import Path

VOXEL_SIZE = 0.02
SEARCH_RADIUS = 0.45
OFFSET_X = 3.2
OFFSET_Y = 4.5
VOXEL_NUM_X = 161
VOXEL_NUM_Y = 451


def read_paths_ply(path: Path):
    lines = path.read_text().splitlines()
    header_end = lines.index("end_header")
    count = 0
    for line in lines[:header_end]:
        if line.startswith("element vertex"):
            count = int(line.split()[-1])
    data = np.array([line.split() for line in lines[header_end + 1:header_end + 1 + count]],
                    dtype=np.float64)
    return data[:, :2], data[:, 3].astype(np.int64)  # x,y and path_id


def main():
    base = Path(__file__).resolve().parent
    points, path_ids = read_paths_ply(base / "paths.ply")
    print(f"路径点: {len(points)}, 路径ID范围: {path_ids.min()}~{path_ids.max()}")

    # Bucket path points into searchRadius-sized cells for fast radius lookup.
    cell = SEARCH_RADIUS
    cell_x = np.floor(points[:, 0] / cell).astype(np.int64)
    cell_y = np.floor(points[:, 1] / cell).astype(np.int64)
    order = np.lexsort((np.arange(len(points)), cell_y, cell_x))
    sorted_x, sorted_y = cell_x[order], cell_y[order]
    sorted_ids = path_ids[order]
    # Cell boundaries in the sorted arrays.
    starts = np.flatnonzero(np.r_[True, (sorted_x[1:] != sorted_x[:-1]) | (sorted_y[1:] != sorted_y[:-1])])
    bounds = np.r_[starts, len(points)]
    cell_map = {(sorted_x[s], sorted_y[s]): (s, e) for s, e in zip(bounds[:-1], bounds[1:])}

    # Voxel grid, matching path_generator.m ordering (ind_x outer, ind_y inner).
    ind_x = np.arange(VOXEL_NUM_X)
    xs = OFFSET_X - VOXEL_SIZE * ind_x
    scale_ys = xs / OFFSET_X + SEARCH_RADIUS / OFFSET_Y * (OFFSET_X - xs) / OFFSET_X

    r2 = SEARCH_RADIUS * SEARCH_RADIUS
    out = []
    for xi in range(VOXEL_NUM_X):
        x = xs[xi]
        vx = int(np.floor(x / cell))
        for yi in range(VOXEL_NUM_Y):
            y = scale_ys[xi] * (OFFSET_Y - VOXEL_SIZE * yi)
            vy = int(np.floor(y / cell))
            segs = [cell_map[(bx, by)] for bx in (vx - 1, vx, vx + 1)
                    for by in (vy - 1, vy, vy + 1) if (bx, by) in cell_map]
            if segs:
                idx = np.concatenate([order[s:e] for s, e in segs])
                dx = points[idx, 0] - x
                dy = points[idx, 1] - y
                hit_idx = idx[dx * dx + dy * dy <= r2]
                # MATLAB prints path IDs ordered by first-appearing point index.
                ids = path_ids[np.sort(hit_idx)]
                deduped = np.unique(ids)
            else:
                deduped = np.empty(0, dtype=np.int64)
            voxel_id = xi * VOXEL_NUM_Y + yi
            out.append(f"{voxel_id} " + " ".join(map(str, deduped.tolist() + [-1])) + "\n")
        if xi % 40 == 0:
            print(f"  已处理 {xi}/{VOXEL_NUM_X} 列")

    (base / "correspondences.txt").write_text("".join(out))
    print(f"已生成 {len(out)} 行 correspondences.txt")


if __name__ == "__main__":
    main()
