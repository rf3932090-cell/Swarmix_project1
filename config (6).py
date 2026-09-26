"""Simple configuration for the two-UAV Gazebo tunnel scenario."""

# Main loop and mission
CONTROL_PERIOD = 0.1
TAKEOFF_ALTITUDE = 1.6
POSITION_TOLERANCE = 0.4
TARGET_EAST = 40.0
TARGET_X = TARGET_EAST  # Kept for compatibility with the phase-2 files.
TARGET_TOLERANCE = 0.4

# Runtime diagnostics.  One readable snapshot is printed for each UAV at this
# interval and the same output is saved to LOG_FILE.  Set LOG_FILE to None if
# terminal output is enough.
LOG_LEVEL = "INFO"
LOG_PERIOD = 0.5
LOG_FILE = "swarmix.log"

# Horizontal movement
FORWARD_SPEED = 1.0
MAX_VELOCITY = 1.5

# Longitudinal synchronization before the intentional one-metre column.
LONGITUDINAL_SYNC_GAIN = 0.8
SYNC_CATCHUP_MAX_SPEED = 1.0
SYNC_CATCHUP_MAX_PATH_ANGLE_DEG = 15.0

# Altitude control
ALTITUDE_GAIN = 0.8
MAX_ALTITUDE_CORRECTION = 0.5

# LiDAR and gap selection
LIDAR_TIMEOUT = 1.0
FRONT_SECTOR_DEG = 3.0
GOAL_CLEAR_DISTANCE = 6.0
GAP_MIN_DEPTH = 1.5
GAP_MIN_WIDTH = 0.75
GAP_MAX_WIDTH = 4.0
# A ray is considered to pass through an opening when it continues this much
# farther than the distance expected for the front wall.  This must stay small:
# rays through the entrance can hit a tunnel side wall shortly after the gap.
GAP_DEPTH_MARGIN = 0.20
GAP_MAX_SEARCH_DEG = 120.0
# Retained for compatibility with earlier processor versions.  The current
# processor uses it only for a near-forward opening with one visible edge.
GAP_EDGE_CLEARANCE = 0.6
ONE_SIDED_GAP_MAX_ANGLE_DEG = 50.0
GAP_APPROACH_SPEED = 0.7
MAX_GAP_LATERAL_SPEED = 0.35
# When a gap is far to one side, first align laterally while holding East.
# Forward entry fades in only after the path angle becomes safely small.
GAP_ALIGNMENT_FULL_SPEED_DEG = 8.0
GAP_ALIGNMENT_STOP_DEG = 25.0
GAP_RELEASE_WIDTH_MARGIN = 0.35
GAP_RELEASE_SCANS = 3
SIDE_SECTOR_DEG = 10.0

# Abrupt 3 m -> 1 m bottleneck detection.  This is a look-ahead measurement:
# the UAV may still be inside the three-metre corridor while the two shoulder
# faces of the one-metre opening are detected ahead.
BOTTLENECK_LOOKAHEAD_DISTANCE = 4.2
BOTTLENECK_PREPARE_COMPLETE_DISTANCE = 0.8
BOTTLENECK_MIN_FORWARD_DISTANCE = 0.4
BOTTLENECK_MIN_OPENING_WIDTH = 0.55
BOTTLENECK_MAX_OPENING_WIDTH = 1.80
BOTTLENECK_EDGE_BAND = 0.15
BOTTLENECK_SIDE_DEPTH_TOLERANCE = 0.50
BOTTLENECK_MIN_RAY_ANGLE_DEG = 5.0
BOTTLENECK_MAX_RAY_ANGLE_DEG = 75.0
BOTTLENECK_MIN_EDGE_SAMPLES = 2
BOTTLENECK_CONFIRM_SCANS = 3
BOTTLENECK_LOST_SCANS = 5
BOTTLENECK_EXIT_SCANS = 5
BOTTLENECK_EXIT_WIDTH = 2.20
BOTTLENECK_APPROACH_SPEED = 0.60
BOTTLENECK_ALIGNMENT_FULL_SPEED_METERS = 0.10
BOTTLENECK_ALIGNMENT_STOP_METERS = 0.45
COLUMN_RELEASE_TIME = 2.0
COLUMN_TRANSITION_DISTANCE = 4.0
# The UAV faces East, so the LiDAR's zero ray points along the mission path.
# Positive LiDAR angles point to the UAV's left.
TARGET_YAW_DEG = 90.0
TARGET_LIDAR_ANGLE_DEG = 0.0

# Wall avoidance
WALL_INFLUENCE_DISTANCE = 1.0
WALL_STOP_DISTANCE = 0.35
WALL_AVOIDANCE_GAIN = 0.6
DRONE_COLLISION_RADIUS = 0.30
WALL_SAFETY_MARGIN = 0.10
HARD_WALL_CLEARANCE = DRONE_COLLISION_RADIUS + WALL_SAFETY_MARGIN

# Corridor centering and formation
INITIAL_FORMATION_SPACING = 5.0
WALL_CLEARANCE = 0.55
THREE_METER_WIDTH = 3.0
ONE_METER_WIDTH = 1.0
CENTER_GAIN = 0.8
FORMATION_GAIN = 0.8
MAX_CENTER_CORRECTION = 0.30
MAX_FORMATION_CORRECTION = 0.35
MAX_LATERAL_SPEED = 0.35
GAP_LANE_GAIN = 0.8
GAP_ALIGNMENT_FULL_SPEED_METERS = 0.08
GAP_ALIGNMENT_STOP_METERS = 0.20
DISTANCE_TOLERANCE = 0.15
MAX_LONGITUDINAL_ERROR = 0.50   # maximum allowed front-back error
SYNC_DEADZONE = 0.10            # ignore small differences

# Compatibility name used by the phase-2 controller.
DESIRED_DISTANCE = INITIAL_FORMATION_SPACING

# Peer following and optional communication
COMMUNICATION_TIMEOUT = 0.5
PEER_SAFE_DISTANCE = 1.0
# Inside this radius, final collision safety cannot be weakened by lane priority.
PEER_HARD_DISTANCE = 0.65
# During a valid gap alignment, reduce only the conflicting North component
# of soft peer avoidance.
GAP_PEER_AVOIDANCE_WEIGHT = 0.35
PEER_FOLLOW_DISTANCE = 1.5
MIN_COLUMN_SPEED = 0.2
COLUMN_KP = 0.2
# If the two UAVs are closer than this in East when the column transition
# starts, their progress is treated as a tie and the smaller UAV id is used
# only as a deterministic tie-breaker.  Otherwise the physically leading UAV
# becomes the temporary bottleneck leader.
LEADER_SELECTION_MARGIN = 0.15


# Each agent receives only its own PX4 connection and LiDAR topic.
# Replace the topic strings with the exact names shown by `gz topic -l` if the
# sensor model publishes different names.
UAVS = {
    1: {
        "name": "UAV 1",
        "mavsdk_port": 50051,
        "system_address": "udpin://0.0.0.0:14540",

        # PX4 local-frame origin expressed in shared swarm frame
        "origin_north": 0.0,
        "origin_east": 0.0,

        "formation_north": -1.0,
        "formation_east": -8.0,

        "communication_port": 15000,
        "lidar_topic": (
            "/world/default/model/x500_lidar_2d_0/"
            "link/link/sensor/lidar_2d_v2/scan"
        ),
        "formation_side": -1,
    },

    2: {
        "name": "UAV 2",
        "mavsdk_port": 50052,
        "system_address": "udpin://0.0.0.0:14541",

        # UAV2 starts 3 m north of UAV1
        "origin_north": 3.0,
        "origin_east": 0.0,

        "formation_north": 1.0,
        "formation_east": -8.0,

        "communication_port": 15001,
        "lidar_topic": (
            "/world/default/model/x500_lidar_2d_1/"
            "link/link/sensor/lidar_2d_v2/scan"
        ),
        "formation_side": 1,
    },
}



