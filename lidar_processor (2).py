"""Process 2-D LiDAR scans for gaps, tunnel centering, and bottlenecks."""

from dataclasses import dataclass
import math
from statistics import median
from typing import List, Optional, Sequence, Tuple, TYPE_CHECKING

from config import (
    BOTTLENECK_EDGE_BAND,
    BOTTLENECK_LOOKAHEAD_DISTANCE,
    BOTTLENECK_MAX_OPENING_WIDTH,
    BOTTLENECK_MAX_RAY_ANGLE_DEG,
    BOTTLENECK_MIN_EDGE_SAMPLES,
    BOTTLENECK_MIN_FORWARD_DISTANCE,
    BOTTLENECK_MIN_OPENING_WIDTH,
    BOTTLENECK_MIN_RAY_ANGLE_DEG,
    BOTTLENECK_SIDE_DEPTH_TOLERANCE,
    FRONT_SECTOR_DEG,
    GAP_DEPTH_MARGIN,
    GAP_EDGE_CLEARANCE,
    GAP_MAX_SEARCH_DEG,
    GAP_MIN_DEPTH,
    GAP_MIN_WIDTH,
    GOAL_CLEAR_DISTANCE,
    ONE_SIDED_GAP_MAX_ANGLE_DEG,
    SIDE_SECTOR_DEG,
    TARGET_LIDAR_ANGLE_DEG,
)

if TYPE_CHECKING:
    from lidar_interface import LidarScanData


@dataclass(frozen=True)
class LidarResult:
    """Processed LiDAR features consumed by the agent and controller."""

    path_angle: float
    center_error: float
    free_width: float
    front_distance: float
    goal_blocked: bool
    gap_found: bool = False
    left_distance: float = math.inf
    right_distance: float = math.inf
    gap_width: float = math.inf
    gap_active: bool = False
    gap_lateral_error: float = 0.0
    left_wall_seen: bool = False
    right_wall_seen: bool = False
    bottleneck_found: bool = False
    bottleneck_distance: float = math.inf
    bottleneck_width: float = math.inf
    bottleneck_angle: float = 0.0
    bottleneck_lateral_error: float = 0.0


def angle_difference(first: float, second: float) -> float:
    """Return the shortest signed angle ``first - second``."""

    return math.atan2(math.sin(first - second), math.cos(first - second))


def sanitize_ranges(scan: "LidarScanData") -> List[float]:
    """Replace no-hit values with range_max while preserving ray indices."""

    clean = []
    for value in scan.ranges:
        if not math.isfinite(value):
            clean.append(scan.range_max)
        else:
            clean.append(max(scan.range_min, min(float(value), scan.range_max)))
    return clean


def ray_angles(scan: "LidarScanData") -> List[float]:
    return [
        scan.angle_min + index * scan.angle_step
        for index in range(len(scan.ranges))
    ]


def sector_values(
    ranges: Sequence[float],
    angles: Sequence[float],
    center: float,
    half_width: float,
) -> List[float]:
    return [
        distance
        for distance, angle in zip(ranges, angles)
        if abs(angle_difference(angle, center)) <= half_width
    ]


def find_gap(
    scan: "LidarScanData",
    ranges: Sequence[float],
    angles: Sequence[float],
    target_angle: float,
    front_distance: float,
) -> Optional[Tuple[float, float]]:
    """Return ``(angle, width)`` for the nearest bounded opening.

    Fully bounded openings always have priority.  A one-sided opening is used
    only when it remains close to the forward direction; this covers a real
    entrance whose far edge is outside the LiDAR view without selecting the
    distant outside end of a wall.
    """

    step = abs(scan.angle_step)

    # Only search in front of the vehicle.  Rays beyond 90 degrees point
    # sideways/backwards and cannot be used as an entrance toward the goal.
    search_limit = min(
        math.radians(GAP_MAX_SEARCH_DEG),
        math.radians(85.0),
    )
    search_indices = sorted(
        (
            index
            for index, angle in enumerate(angles)
            if abs(angle_difference(angle, target_angle)) <= search_limit
        ),
        key=lambda index: angles[index],
    )
    if not search_indices:
        return None

    # A flat wall does not have the same range at every scan angle.  If its
    # perpendicular distance is d, the expected range is d / cos(angle).
    # Comparing every ray with ``front_distance + margin`` therefore creates
    # false openings at oblique angles and can also miss a real opening whose
    # ray hits a tunnel wall shortly behind the entrance.
    open_by_index = {}
    for index in search_indices:
        deviation = abs(angle_difference(angles[index], target_angle))
        cosine = math.cos(deviation)
        if cosine <= 0.0:
            open_by_index[index] = False
            continue

        expected_front_wall = front_distance / cosine
        required_depth = max(
            GAP_MIN_DEPTH,
            expected_front_wall + GAP_DEPTH_MARGIN,
        )
        open_by_index[index] = ranges[index] >= min(
            scan.range_max,
            required_depth,
        )

    runs = []
    run_start = None
    for position, index in enumerate(search_indices):
        is_open = open_by_index[index]
        if is_open and run_start is None:
            run_start = position
        if run_start is not None and (
            not is_open or position == len(search_indices) - 1
        ):
            run_end = position if is_open else position - 1
            runs.append((run_start, run_end))
            run_start = None

    wall_depth = max(front_distance, scan.range_min)
    edge_margin_angle = math.atan2(GAP_EDGE_CLEARANCE, wall_depth)
    bounded_candidates = []
    one_sided_candidates = []

    for start_position, end_position in runs:
        start_index = search_indices[start_position]
        end_index = search_indices[end_position]
        start_angle = angles[start_index]
        end_angle = angles[end_index]
        # Estimate width in the plane of the front wall.  The old chord-based
        # formula used an arbitrary depth threshold and underestimated or
        # overestimated openings as the UAV approached the wall.
        start_deviation = angle_difference(start_angle, target_angle)
        end_deviation = angle_difference(end_angle, target_angle)
        start_edge = wall_depth * math.tan(start_deviation - 0.5 * step)
        end_edge = wall_depth * math.tan(end_deviation + 0.5 * step)
        physical_width = abs(end_edge - start_edge)
        # Reject unrealistic gaps (e.g. open sky/outside space detected as entrance)
        if physical_width > 5.0:
            continue
        if physical_width < GAP_MIN_WIDTH:
            continue

        closed_before = (
            start_position > 0
            and not open_by_index[search_indices[start_position - 1]]
        )
        closed_after = (
            end_position < len(search_indices) - 1
            and not open_by_index[search_indices[end_position + 1]]
        )

        if closed_before and closed_after:
            centre_edge = 0.5 * (start_edge + end_edge)
            candidate_angle = target_angle + math.atan2(
                centre_edge,
                wall_depth,
            )
            deviation = abs(angle_difference(candidate_angle, target_angle))
            bounded_candidates.append(
                (deviation, -physical_width, candidate_angle, physical_width)
            )
            continue

        if closed_before and not closed_after:
            candidate_angle = min(
                end_angle,
                start_angle + edge_margin_angle,
            )
        elif closed_after and not closed_before:
            candidate_angle = max(
                start_angle,
                end_angle - edge_margin_angle,
            )
        else:
            # The entire search sector is open, so there is no wall entrance.
            continue

        deviation = abs(angle_difference(candidate_angle, target_angle))
        if deviation > math.radians(ONE_SIDED_GAP_MAX_ANGLE_DEG):
            continue
        one_sided_candidates.append(
            (deviation, -physical_width, candidate_angle, physical_width)
        )

    candidates = bounded_candidates or one_sided_candidates
    if not candidates:
        return None

    best = min(candidates)
    return best[2], best[3]



def find_forward_bottleneck(
    scan: "LidarScanData",
    ranges: Sequence[float],
    angles: Sequence[float],
    target_angle: float,
) -> Optional[Tuple[float, float, float]]:
    """Return ``(distance, width, centre_angle)`` for a narrowing ahead.

    A 3 m -> 1 m transition produces two shoulder faces on the same forward
    plane.  Rays are projected into forward/lateral coordinates and the two
    inner shoulder edges are used to estimate the future opening.  Ordinary
    three-metre side walls produce an opening near 3 m and are rejected.
    """

    left_points = []
    right_points = []
    min_angle = math.radians(BOTTLENECK_MIN_RAY_ANGLE_DEG)
    max_angle = math.radians(BOTTLENECK_MAX_RAY_ANGLE_DEG)

    for distance, angle in zip(ranges, angles):
        # Sanitized no-hit rays equal range_max and are not physical edges.
        if distance >= 0.98 * scan.range_max:
            continue

        deviation = angle_difference(angle, target_angle)
        absolute_deviation = abs(deviation)
        if not min_angle <= absolute_deviation <= max_angle:
            continue

        forward = distance * math.cos(deviation)
        if not (
            BOTTLENECK_MIN_FORWARD_DISTANCE
            <= forward
            <= BOTTLENECK_LOOKAHEAD_DISTANCE
        ):
            continue

        lateral = distance * math.sin(deviation)
        point = (abs(lateral), forward)
        if lateral > 0.0:
            left_points.append(point)
        elif lateral < 0.0:
            right_points.append(point)

    if not left_points or not right_points:
        return None

    left_inner = min(lateral for lateral, _ in left_points)
    right_inner = min(lateral for lateral, _ in right_points)
    opening_width = left_inner + right_inner
    if not (
        BOTTLENECK_MIN_OPENING_WIDTH
        <= opening_width
        <= BOTTLENECK_MAX_OPENING_WIDTH
    ):
        return None

    left_edge_depths = sorted(
        forward
        for lateral, forward in left_points
        if lateral <= left_inner + BOTTLENECK_EDGE_BAND
    )
    right_edge_depths = sorted(
        forward
        for lateral, forward in right_points
        if lateral <= right_inner + BOTTLENECK_EDGE_BAND
    )
    if (
        len(left_edge_depths) < BOTTLENECK_MIN_EDGE_SAMPLES
        or len(right_edge_depths) < BOTTLENECK_MIN_EDGE_SAMPLES
    ):
        return None

    sample_count = BOTTLENECK_MIN_EDGE_SAMPLES
    left_depth = median(left_edge_depths[:sample_count])
    right_depth = median(right_edge_depths[:sample_count])
    if abs(left_depth - right_depth) > BOTTLENECK_SIDE_DEPTH_TOLERANCE:
        return None

    bottleneck_distance = 0.5 * (left_depth + right_depth)
    centre_lateral = 0.5 * (left_inner - right_inner)
    centre_angle = math.atan2(centre_lateral, bottleneck_distance)
    return bottleneck_distance, opening_width, centre_angle


def process_scan(
    scan: "LidarScanData",
    target_angle: float = math.radians(TARGET_LIDAR_ANGLE_DEG),
) -> LidarResult:
    """Convert one raw LiDAR scan into the features used by control logic."""

    if not scan.ranges:
        raise ValueError("LiDAR scan cannot be empty")
    if scan.angle_step == 0.0:
        raise ValueError("LiDAR angle_step cannot be zero")
    if scan.range_max <= scan.range_min:
        raise ValueError("LiDAR range limits are invalid")

    ranges = sanitize_ranges(scan)
    angles = ray_angles(scan)

    front_values = sector_values(
        ranges,
        angles,
        target_angle,
        math.radians(FRONT_SECTOR_DEG),
    )
    # A narrow median is robust to a single noisy ray and does not mistake
    # the side walls of a narrow tunnel for a wall directly ahead.
    front_distance = median(front_values) if front_values else scan.range_max
    goal_blocked = front_distance < GOAL_CLEAR_DISTANCE

    gap = None
    if goal_blocked:
        gap = find_gap(
            scan,
            ranges,
            angles,
            target_angle,
            front_distance,
        )
    gap_found = gap is not None
    path_angle = gap[0] if gap_found else target_angle
    gap_width = gap[1] if gap_found else math.inf

    projected_bottleneck = find_forward_bottleneck(
        scan,
        ranges,
        angles,
        target_angle,
    )
    narrow_gap_ahead = (
        goal_blocked
        and gap_found
        and BOTTLENECK_MIN_OPENING_WIDTH
        <= gap_width
        <= BOTTLENECK_MAX_OPENING_WIDTH
        and front_distance <= BOTTLENECK_LOOKAHEAD_DISTANCE
    )
    if projected_bottleneck is not None:
        bottleneck = projected_bottleneck
    elif narrow_gap_ahead:
        bottleneck = (front_distance, gap_width, path_angle)
    else:
        bottleneck = None
    bottleneck_found = bottleneck is not None
    bottleneck_distance = bottleneck[0] if bottleneck_found else math.inf
    bottleneck_width = bottleneck[1] if bottleneck_found else math.inf
    bottleneck_angle = bottleneck[2] if bottleneck_found else target_angle
    bottleneck_lateral_error = (
        bottleneck_distance * math.tan(bottleneck_angle)
        if bottleneck_found
        else 0.0
    )

    side_half_width = math.radians(SIDE_SECTOR_DEG)
    left_values = sector_values(
        ranges,
        angles,
        target_angle + math.pi / 2.0,
        side_half_width,
    )
    right_values = sector_values(
        ranges,
        angles,
        target_angle - math.pi / 2.0,
        side_half_width,
    )

    left_distance = median(left_values) if left_values else scan.range_max
    right_distance = median(right_values) if right_values else scan.range_max

    left_wall_seen = left_distance < 0.98 * scan.range_max
    right_wall_seen = right_distance < 0.98 * scan.range_max
    if left_wall_seen and right_wall_seen:
        free_width = left_distance + right_distance
        center_error = 0.5 * (left_distance - right_distance)
    else:
        free_width = 2.0 * scan.range_max
        center_error = 0.0

    return LidarResult(
        path_angle=path_angle,
        center_error=center_error,
        free_width=free_width,
        front_distance=front_distance,
        goal_blocked=goal_blocked,
        gap_found=gap_found,
        left_distance=left_distance,
        right_distance=right_distance,
        gap_width=gap_width,
        gap_active=gap_found,
        gap_lateral_error=(
            front_distance * math.tan(path_angle) if gap_found else 0.0
        ),
        left_wall_seen=left_wall_seen,
        right_wall_seen=right_wall_seen,
        bottleneck_found=bottleneck_found,
        bottleneck_distance=bottleneck_distance,
        bottleneck_width=bottleneck_width,
        bottleneck_angle=bottleneck_angle,
        bottleneck_lateral_error=bottleneck_lateral_error,
    )



