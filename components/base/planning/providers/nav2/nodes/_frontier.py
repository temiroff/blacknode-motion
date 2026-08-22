"""Dependency-light occupancy-grid frontier selection for Nav2 exploration."""
from __future__ import annotations

import math
from collections import deque
from typing import Any, Iterable


def _cell_to_world(
    column: int,
    row: int,
    *,
    resolution: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
) -> tuple[float, float]:
    local_x = (column + 0.5) * resolution
    local_y = (row + 0.5) * resolution
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    return (
        origin_x + cosine * local_x - sine * local_y,
        origin_y + sine * local_x + cosine * local_y,
    )


def _is_frontier(data: list[int], width: int, height: int, index: int) -> bool:
    value = data[index]
    if value < 0 or value >= 50:
        return False
    row, column = divmod(index, width)
    for delta_column, delta_row in ((-1, 0), (1, 0), (0, -1), (0, 1)):
        neighbor_column = column + delta_column
        neighbor_row = row + delta_row
        if 0 <= neighbor_column < width and 0 <= neighbor_row < height:
            if data[neighbor_row * width + neighbor_column] < 0:
                return True
    return False


def _clusters(frontiers: set[int], width: int, height: int) -> list[list[int]]:
    remaining = set(frontiers)
    groups: list[list[int]] = []
    while remaining:
        seed = remaining.pop()
        group = [seed]
        pending = deque([seed])
        while pending:
            current = pending.popleft()
            row, column = divmod(current, width)
            for delta_row in (-1, 0, 1):
                for delta_column in (-1, 0, 1):
                    if not delta_column and not delta_row:
                        continue
                    next_column = column + delta_column
                    next_row = row + delta_row
                    if not (0 <= next_column < width and 0 <= next_row < height):
                        continue
                    neighbor = next_row * width + next_column
                    if neighbor in remaining:
                        remaining.remove(neighbor)
                        group.append(neighbor)
                        pending.append(neighbor)
        groups.append(group)
    return groups


def _occupied_within(
    data: list[int],
    width: int,
    height: int,
    column: int,
    row: int,
    radius_cells: int,
) -> bool:
    radius_squared = radius_cells * radius_cells
    for next_row in range(max(0, row - radius_cells), min(height, row + radius_cells + 1)):
        for next_column in range(max(0, column - radius_cells), min(width, column + radius_cells + 1)):
            if (next_column - column) ** 2 + (next_row - row) ** 2 > radius_squared:
                continue
            if data[next_row * width + next_column] >= 65:
                return True
    return False


def select_frontier_goal(
    data: Iterable[Any],
    *,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
    robot_x: float,
    robot_y: float,
    min_frontier_cells: int,
    obstacle_clearance_m: float,
    min_goal_distance_m: float,
    excluded_goals: Iterable[tuple[float, float]] = (),
    excluded_radius_m: float = 0.75,
) -> dict[str, Any]:
    """Choose a reachable-looking frontier target from a ROS OccupancyGrid.

    This selector deliberately makes no motion command. Nav2 remains responsible
    for global/local planning and final collision checking.
    """
    cells = [int(value) for value in data]
    cell_count = max(0, int(width)) * max(0, int(height))
    if width <= 0 or height <= 0 or resolution <= 0.0 or len(cells) < cell_count:
        return {"goal": None, "candidates": [], "known_cells": 0, "coverage": 0.0}
    cells = cells[:cell_count]
    known_cells = sum(1 for value in cells if value >= 0)
    frontiers = {index for index in range(cell_count) if _is_frontier(cells, width, height, index)}
    radius_cells = max(1, int(math.ceil(max(0.0, obstacle_clearance_m) / resolution)))
    excluded = list(excluded_goals)
    candidates: list[dict[str, Any]] = []
    for group in _clusters(frontiers, width, height):
        if len(group) < max(1, int(min_frontier_cells)):
            continue
        mean_column = sum(index % width for index in group) / len(group)
        mean_row = sum(index // width for index in group) / len(group)
        target_index = min(
            group,
            key=lambda index: (index % width - mean_column) ** 2 + (index // width - mean_row) ** 2,
        )
        row, column = divmod(target_index, width)
        if _occupied_within(cells, width, height, column, row, radius_cells):
            continue
        world_x, world_y = _cell_to_world(
            column,
            row,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
            origin_yaw=origin_yaw,
        )
        distance = math.hypot(world_x - robot_x, world_y - robot_y)
        if distance < max(0.0, min_goal_distance_m):
            continue
        if any(math.hypot(world_x - x, world_y - y) < excluded_radius_m for x, y in excluded):
            continue
        information_gain_m = len(group) * resolution
        score = information_gain_m - 0.35 * distance
        candidates.append({
            "x_m": world_x,
            "y_m": world_y,
            "yaw_rad": math.atan2(world_y - robot_y, world_x - robot_x),
            "distance_m": distance,
            "frontier_cells": len(group),
            "information_gain_m": information_gain_m,
            "score": score,
        })
    candidates.sort(key=lambda item: item["score"], reverse=True)
    return {
        "goal": dict(candidates[0]) if candidates else None,
        "candidates": [dict(item) for item in candidates[:20]],
        "known_cells": known_cells,
        "frontier_cells": len(frontiers),
        "coverage": known_cells / cell_count if cell_count else 0.0,
    }
