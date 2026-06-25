"""Pure geometry helpers for conservative ART path post-processing."""

from dataclasses import dataclass
import math
from typing import Sequence, Tuple

from pct_art_local_navigation.coordinator_logic import (
    MapGeometry,
    Point2,
    distance,
    point_inside_map,
)


@dataclass(frozen=True)
class PathSmoothingConfig:
    enabled: bool = True
    min_point_spacing: float = 0.10
    resample_spacing: float = 0.10
    collinear_angle_threshold: float = 0.08
    iterations: int = 20
    data_weight: float = 0.45
    smooth_weight: float = 0.20
    max_deviation: float = 0.15


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def remove_short_segments(
    points: Sequence[Point2], minimum_spacing: float
) -> list[Point2]:
    if len(points) <= 2:
        return list(points)

    filtered = [points[0]]
    for point in points[1:-1]:
        if distance(filtered[-1], point) >= minimum_spacing:
            filtered.append(point)

    if distance(filtered[-1], points[-1]) > 0.0:
        filtered.append(points[-1])
    elif filtered[-1] != points[-1]:
        filtered[-1] = points[-1]
    return filtered


def remove_collinear_points(
    points: Sequence[Point2], angle_threshold: float
) -> list[Point2]:
    if len(points) <= 2:
        return list(points)

    filtered = [points[0]]
    for index in range(1, len(points) - 1):
        previous = filtered[-1]
        current = points[index]
        following = points[index + 1]
        first_length = distance(previous, current)
        second_length = distance(current, following)
        if first_length <= 0.0 or second_length <= 0.0:
            continue

        first_yaw = math.atan2(current.y - previous.y, current.x - previous.x)
        second_yaw = math.atan2(following.y - current.y, following.x - current.x)
        turn = abs(normalize_angle(second_yaw - first_yaw))
        if turn > angle_threshold:
            filtered.append(current)

    if distance(filtered[-1], points[-1]) > 0.0:
        filtered.append(points[-1])
    elif filtered[-1] != points[-1]:
        filtered[-1] = points[-1]
    return filtered


def closest_point_on_segment(point: Point2, start: Point2, end: Point2) -> Point2:
    dx = end.x - start.x
    dy = end.y - start.y
    length_sq = dx * dx + dy * dy
    if length_sq <= 0.0:
        return start

    ratio = ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_sq
    ratio = max(0.0, min(1.0, ratio))
    return Point2(start.x + ratio * dx, start.y + ratio * dy)


def closest_point_on_path(point: Point2, path: Sequence[Point2]) -> Point2:
    if not path:
        raise ValueError('reference path must not be empty')
    if len(path) == 1:
        return path[0]

    return min(
        (
            closest_point_on_segment(point, path[index], path[index + 1])
            for index in range(len(path) - 1)
        ),
        key=lambda candidate: distance(point, candidate),
    )


def deviation_from_path(point: Point2, reference_path: Sequence[Point2]) -> float:
    return distance(point, closest_point_on_path(point, reference_path))


def maximum_deviation_from_path(
    points: Sequence[Point2], reference_path: Sequence[Point2]
) -> float:
    if not points:
        return 0.0
    return max(deviation_from_path(point, reference_path) for point in points)


def limit_point_deviation(
    point: Point2, reference_path: Sequence[Point2], maximum_deviation: float
) -> Point2:
    closest = closest_point_on_path(point, reference_path)
    current_deviation = distance(point, closest)
    if current_deviation <= maximum_deviation or current_deviation <= 0.0:
        return point

    ratio = maximum_deviation / current_deviation
    return Point2(
        closest.x + (point.x - closest.x) * ratio,
        closest.y + (point.y - closest.y) * ratio,
    )


def smooth_points(
    points: Sequence[Point2],
    iterations: int,
    data_weight: float,
    smooth_weight: float,
    maximum_deviation: float,
    reference_path: Sequence[Point2],
) -> list[Point2]:
    if len(points) <= 2 or iterations <= 0:
        return list(points)

    original = list(points)
    smoothed = list(points)
    for _ in range(iterations):
        updated = smoothed.copy()
        for index in range(1, len(smoothed) - 1):
            updated_point = Point2(
                smoothed[index].x
                + data_weight * (original[index].x - smoothed[index].x)
                + smooth_weight
                * (smoothed[index - 1].x + smoothed[index + 1].x - 2.0 * smoothed[index].x),
                smoothed[index].y
                + data_weight * (original[index].y - smoothed[index].y)
                + smooth_weight
                * (smoothed[index - 1].y + smoothed[index + 1].y - 2.0 * smoothed[index].y),
            )
            updated[index] = limit_point_deviation(
                updated_point, reference_path, maximum_deviation
            )
        smoothed = updated
    return smoothed


def resample_path(points: Sequence[Point2], spacing: float) -> list[Point2]:
    if len(points) < 2:
        return list(points)

    cumulative = [0.0]
    for index in range(1, len(points)):
        cumulative.append(cumulative[-1] + distance(points[index - 1], points[index]))

    total_length = cumulative[-1]
    if total_length <= 0.0:
        return [points[0], points[-1]]

    resampled = [points[0]]
    target_distance = spacing
    segment_index = 1
    while target_distance < total_length:
        while segment_index < len(cumulative) - 1 and cumulative[segment_index] < target_distance:
            segment_index += 1

        start_distance = cumulative[segment_index - 1]
        end_distance = cumulative[segment_index]
        segment_length = end_distance - start_distance
        if segment_length <= 0.0:
            target_distance += spacing
            continue

        ratio = (target_distance - start_distance) / segment_length
        start = points[segment_index - 1]
        end = points[segment_index]
        resampled.append(
            Point2(
                start.x + ratio * (end.x - start.x),
                start.y + ratio * (end.y - start.y),
            )
        )
        target_distance += spacing

    if distance(resampled[-1], points[-1]) > 0.0:
        resampled.append(points[-1])
    return resampled


def smooth_local_path(
    points: Sequence[Point2], config: PathSmoothingConfig
) -> list[Point2]:
    if not config.enabled:
        return list(points)

    cleaned = remove_short_segments(points, config.min_point_spacing)
    cleaned = remove_collinear_points(cleaned, config.collinear_angle_threshold)
    smoothed = smooth_points(
        cleaned,
        config.iterations,
        config.data_weight,
        config.smooth_weight,
        config.max_deviation,
        points,
    )
    resampled = resample_path(smoothed, config.resample_spacing)
    return [
        limit_point_deviation(point, points, config.max_deviation)
        for point in resampled
    ]


def validate_smoothed_path(
    points: Sequence[Point2],
    reference_path: Sequence[Point2],
    geometry: MapGeometry,
    edge_margin: float,
    maximum_deviation: float,
) -> Tuple[bool, str]:
    for index, point in enumerate(points):
        if not point_inside_map(point, geometry, edge_margin):
            return False, f'smoothed path point {index} is outside GridMap margin'

    measured_deviation = maximum_deviation_from_path(points, reference_path)
    if measured_deviation > maximum_deviation + 1e-9:
        return (
            False,
            f'smoothed path deviates {measured_deviation:.2f} m from ART path',
        )
    return True, ''


def total_turning_angle(points: Sequence[Point2]) -> float:
    if len(points) < 3:
        return 0.0

    total = 0.0
    previous_yaw = math.atan2(points[1].y - points[0].y, points[1].x - points[0].x)
    for index in range(2, len(points)):
        yaw = math.atan2(
            points[index].y - points[index - 1].y,
            points[index].x - points[index - 1].x,
        )
        total += abs(normalize_angle(yaw - previous_yaw))
        previous_yaw = yaw
    return total
