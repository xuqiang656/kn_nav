import math

from pct_art_local_navigation.coordinator_logic import MapGeometry, Point2
from pct_art_local_navigation.path_smoothing import (
    PathSmoothingConfig,
    maximum_deviation_from_path,
    remove_collinear_points,
    remove_short_segments,
    resample_path,
    smooth_local_path,
    smooth_points,
    total_turning_angle,
    validate_smoothed_path,
)


def test_short_segment_cleanup_preserves_endpoints():
    points = [
        Point2(0.0, 0.0),
        Point2(0.02, 0.0),
        Point2(0.20, 0.0),
        Point2(1.0, 0.0),
    ]

    filtered = remove_short_segments(points, 0.10)

    assert filtered[0] == points[0]
    assert filtered[-1] == points[-1]
    assert Point2(0.02, 0.0) not in filtered


def test_collinear_cleanup_removes_redundant_points_only():
    points = [
        Point2(0.0, 0.0),
        Point2(1.0, 0.01),
        Point2(2.0, 0.0),
        Point2(2.0, 1.0),
    ]

    filtered = remove_collinear_points(points, 0.08)

    assert Point2(1.0, 0.01) not in filtered
    assert Point2(2.0, 0.0) in filtered
    assert filtered[0] == points[0]
    assert filtered[-1] == points[-1]


def test_zigzag_path_smoothing_reduces_total_turning_angle():
    points = [
        Point2(0.0, 0.0),
        Point2(1.0, 0.50),
        Point2(2.0, -0.50),
        Point2(3.0, 0.50),
        Point2(4.0, 0.0),
    ]

    smoothed = smooth_points(points, 20, 0.45, 0.20, 1.0, points)

    assert smoothed[0] == points[0]
    assert smoothed[-1] == points[-1]
    assert total_turning_angle(smoothed) < total_turning_angle(points)


def test_smoothing_respects_maximum_deviation_from_art_path():
    points = [
        Point2(0.0, 0.0),
        Point2(1.0, 1.0),
        Point2(2.0, -1.0),
        Point2(3.0, 0.0),
    ]
    config = PathSmoothingConfig(
        resample_spacing=0.20,
        iterations=30,
        data_weight=0.20,
        smooth_weight=0.50,
        max_deviation=0.05,
    )

    smoothed = smooth_local_path(points, config)

    assert maximum_deviation_from_path(smoothed, points) <= 0.05 + 1e-9


def test_resample_path_uses_nearly_uniform_spacing():
    points = [Point2(0.0, 0.0), Point2(1.0, 0.0), Point2(1.0, 1.0)]

    resampled = resample_path(points, 0.25)

    assert resampled[0] == points[0]
    assert resampled[-1] == points[-1]
    for first, second in zip(resampled[:-2], resampled[1:-1]):
        assert math.isclose(math.hypot(second.x - first.x, second.y - first.y), 0.25)


def test_validate_smoothed_path_rejects_map_escape_and_large_deviation():
    reference = [Point2(0.0, 0.0), Point2(1.0, 0.0)]
    geometry = MapGeometry(0.0, 0.0, 0.0, 4.0, 4.0)

    valid, reason = validate_smoothed_path(
        [Point2(0.0, 0.0), Point2(0.5, 0.0), Point2(1.0, 0.0)],
        reference,
        geometry,
        0.5,
        0.10,
    )
    assert valid
    assert reason == ''

    valid, reason = validate_smoothed_path(
        [Point2(0.0, 0.0), Point2(2.0, 0.0)],
        reference,
        geometry,
        0.5,
        0.10,
    )
    assert not valid
    assert 'outside GridMap' in reason

    valid, reason = validate_smoothed_path(
        [Point2(0.0, 0.0), Point2(0.5, 0.2), Point2(1.0, 0.0)],
        reference,
        geometry,
        0.5,
        0.10,
    )
    assert not valid
    assert 'deviates' in reason
