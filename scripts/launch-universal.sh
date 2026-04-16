#!/bin/bash

# =============================================================================
# launch-universal.sh
#
# Launches one or more Crazyflie drones in either simulation or real mode.
#
# Usage:
#   ./launch-universal.sh <MODE> <DRONE_SPEC> [<DRONE_SPEC> ...]
#
# Arguments:
#   MODE         Either "sim" or "real"
#   DRONE_SPEC   A colon-separated string: <prefix>:<type>[:<extra>]
#
#     prefix     ROS 2 namespace for this drone, e.g. cf1, cf2
#     type       One of: wallfollowing | manual | square |
#                        frontier-exploration | frontier-exploration-swarm |
#                        map-user
#     extra      Optional third field:
#                  - sim drones (non map-user): Gazebo world name
#                    e.g. maze, crazyflie_world, circle-maze
#                    Only one drone needs to specify this — all share a world.
#                  - map-user drones: absolute path to a saved map.yaml
#                    Omit to start with a blank map.
#
# Examples:
#   ./launch-universal.sh sim cf1:frontier-exploration:maze cf2:frontier-exploration
#   ./launch-universal.sh sim cf1:frontier-exploration-swarm:maze cf2:frontier-exploration-swarm
#   ./launch-universal.sh sim cf1:map-user:/home/ryan/maps/room.yaml
#   ./launch-universal.sh real cf1:frontier-exploration cf2:map-user
#   ./launch-universal.sh real cf1:manual
# =============================================================================

# -----------------------------------------------------------------------------
# Paths
# -----------------------------------------------------------------------------

BASE_DIR=$(readlink -f "$(dirname "${BASH_SOURCE[0]}")")
ROS2_WS="$BASE_DIR/../core/crazyflie_mapping_demo/ros2_ws"
BRINGUP_PKG="crazyflie_ros2_multiranger_bringup"

VALID_TYPES=("wallfollowing" "manual" "square" "frontier-exploration" "frontier-exploration-swarm" "map-user" "object-detection")

# -----------------------------------------------------------------------------
# LAUNCH FILE MAPPING
# -----------------------------------------------------------------------------

SHARED_LAUNCH_REAL="shared_real.launch.py"
SHARED_LAUNCH_SIM="shared_sim.launch.py"

declare -A PER_DRONE_LAUNCH_REAL=(
    ["frontier-exploration"]="frontier_exploration_real.launch.py"
    ["frontier-exploration-swarm"]="frontier_exploration_swarm_real.launch.py"
    ["wallfollowing"]="wallfollowing_real.launch.py"
    ["square"]="square_real.launch.py"
    ["map-user"]="map_user_real.launch.py"
    ["manual"]="none"
    ["object-detection"]="ai_real.launch.py"
)

declare -A PER_DRONE_LAUNCH_SIM=(
    ["frontier-exploration"]="frontier_exploration_sim.launch.py"
    ["frontier-exploration-swarm"]="frontier_exploration_swarm_sim.launch.py"
    ["wallfollowing"]="wallfollowing_sim.launch.py"
    ["square"]="square_sim.launch.py"
    ["map-user"]="map_user_sim.launch.py"
    ["manual"]="none"
    ["object-detection"]="ai_sim.launch.py"
)

# -----------------------------------------------------------------------------
# Help text
# -----------------------------------------------------------------------------

display_help() {
    echo ""
    echo "Usage: ./launch-universal.sh <MODE> <DRONE_SPEC> [<DRONE_SPEC> ...]"
    echo ""
    echo "  MODE         sim | real"
    echo "  DRONE_SPEC   <prefix>:<type>[:<extra>]"
    echo ""
    echo "  Types:"
    echo "    wallfollowing              Follow walls autonomously"
    echo "    manual                     Keyboard teleoperation"
    echo "    square                     Fly a square pattern"
    echo "    frontier-exploration       Single-drone autonomous exploration"
    echo "    frontier-exploration-swarm Multi-drone coordinated exploration"
    echo "                               (uses centralised goal assigner)"
    echo "    map-user                   Navigate using a pre-built or blank map"
    echo ""
    echo "  Third field (optional):"
    echo "    sim mode:            Gazebo world name (e.g. maze, crazyflie_world)"
    echo "                         Only specify on one drone — all drones share the same world."
    echo "    map-user (any mode): Absolute path to map.yaml"
    echo "                         Omit to start with a blank map."
    echo ""
    echo "Examples:"
    echo "  ./launch-universal.sh sim cf1:frontier-exploration:maze cf2:frontier-exploration"
    echo "  ./launch-universal.sh sim cf1:frontier-exploration-swarm:maze cf2:frontier-exploration-swarm"
    echo "  ./launch-universal.sh sim cf1:map-user:/home/ryan/maps/room.yaml"
    echo "  ./launch-universal.sh real cf1:frontier-exploration cf2:map-user"
    echo "  ./launch-universal.sh real cf1:manual"
    echo ""
    exit 1
}

# -----------------------------------------------------------------------------
# Step 1: Validate the mode argument
# -----------------------------------------------------------------------------

MODE=$1
shift

if [[ "$MODE" != "sim" && "$MODE" != "real" ]]; then
    echo "ERROR: MODE must be 'sim' or 'real'. Got: '$MODE'"
    display_help
fi

if [[ $# -eq 0 ]]; then
    echo "ERROR: At least one DRONE_SPEC is required."
    display_help
fi

# -----------------------------------------------------------------------------
# Step 2: Parse drone specs into parallel arrays
# -----------------------------------------------------------------------------

declare -a PREFIXES
declare -a TYPES
declare -a MAPS
WORLD=""
MAP_FILE=""
SWARM_ACTIVE="false"   # set true if any drone uses frontier-exploration-swarm

for spec in "$@"; do
    IFS=':' read -r -a parts <<< "$spec"

    prefix="${parts[0]}"
    type="${parts[1]}"
    extra="${parts[2]:-}"

    if [[ -z "$prefix" || -z "$type" ]]; then
        echo "ERROR: Invalid drone spec '$spec'. Expected <prefix>:<type>[:<extra>]"
        display_help
    fi

    # Validate type
    type_is_valid=0
    for valid_type in "${VALID_TYPES[@]}"; do
        [[ "$type" == "$valid_type" ]] && type_is_valid=1 && break
    done
    if [[ $type_is_valid -eq 0 ]]; then
        echo "ERROR: Unknown type '$type' in spec '$spec'."
        echo "       Valid types: ${VALID_TYPES[*]}"
        display_help
    fi

    PREFIXES+=("/$prefix")
    TYPES+=("$type")
    MAPS+=("$extra")

    # Check if this drone uses the swarm explorer
    if [[ "$type" == "frontier-exploration-swarm" ]]; then
        SWARM_ACTIVE="true"
    fi

    # Extract Gazebo world name from third field when applicable
    if [[ -n "$extra" && "$extra" != /* && "$type" != "map-user" && -z "$WORLD" ]]; then
        WORLD="$extra"
    fi

    # Extract map file path from third field when applicable (any drone type with an absolute path)
    if [[ -n "$extra" && "$extra" == /* && -z "$MAP_FILE" ]]; then
        MAP_FILE="$extra"
    fi
done

DRONE_COUNT=${#PREFIXES[@]}

# Build the prefixes list string: [/cf1,/cf2]
PREFIXES_LIST=$(IFS=','; echo "[${PREFIXES[*]}]")

# Build the swarm-only prefixes list for the goal assigner: [/cf1,/cf2]
# Only includes drones using frontier-exploration-swarm.
SWARM_PREFIXES=()
for i in "${!TYPES[@]}"; do
    if [[ "${TYPES[$i]}" == "frontier-exploration-swarm" ]]; then
        SWARM_PREFIXES+=("${PREFIXES[$i]}")
    fi
done
SWARM_PREFIXES_LIST=$(IFS=','; echo "[${SWARM_PREFIXES[*]}]")

# -----------------------------------------------------------------------------
# Step 3: Source ROS 2 workspace
# -----------------------------------------------------------------------------

source "$ROS2_WS/install/setup.bash"
export GZ_SIM_RESOURCE_PATH="$BASE_DIR/../core/crazyflie_mapping_demo/simulation_ws/crazyflie-simulation/simulator_files/gazebo/"

# -----------------------------------------------------------------------------
# Step 4: Print a launch summary
# -----------------------------------------------------------------------------

echo ""
echo "Launching $DRONE_COUNT drone(s) in $MODE mode:"
for i in "${!PREFIXES[@]}"; do
    extra_info=""
    [[ -n "${MAPS[$i]}" ]] && extra_info=" (${MAPS[$i]})"
    echo "  ${PREFIXES[$i]}  →  ${TYPES[$i]}${extra_info}"
done
[[ -n "$WORLD" ]] && echo "  Gazebo world: $WORLD"
[[ "$SWARM_ACTIVE" == "true" ]] && echo "  Swarm goal assigner: ENABLED (drones: $SWARM_PREFIXES_LIST)"
echo ""

# -----------------------------------------------------------------------------
# Step 5: Launch the shared file
# Passes launch_goal_assigner=true when any swarm drone is present so the
# shared launch file knows to start the goal_assigner node.
# -----------------------------------------------------------------------------

if [[ "$MODE" == "real" ]]; then
    SHARED_FILE="$SHARED_LAUNCH_REAL"
    SHARED_ARGS=(
        "robot_prefixes:=$PREFIXES_LIST"
        "launch_goal_assigner:=$SWARM_ACTIVE"
        "swarm_prefixes:=$SWARM_PREFIXES_LIST"
    )
    [[ -n "$MAP_FILE" ]] && SHARED_ARGS+=("map_file:=$MAP_FILE")
else
    SHARED_FILE="$SHARED_LAUNCH_SIM"
    SHARED_ARGS=(
        "robot_prefixes:=$PREFIXES_LIST"
        "launch_goal_assigner:=$SWARM_ACTIVE"
        "swarm_prefixes:=$SWARM_PREFIXES_LIST"
    )
    [[ -n "$MAP_FILE" ]] && SHARED_ARGS+=("map_file:=$MAP_FILE")
    [[ -n "$WORLD" ]] && SHARED_ARGS+=("world:=$WORLD")
fi

echo "Starting shared nodes ($MODE)..."
ros2 launch "$BRINGUP_PKG" "$SHARED_FILE" "${SHARED_ARGS[@]}" &

# -----------------------------------------------------------------------------
# Step 6: Launch one per-drone file per drone
# -----------------------------------------------------------------------------

for i in "${!PREFIXES[@]}"; do
    prefix="${PREFIXES[$i]}"
    type="${TYPES[$i]}"
    map="${MAPS[$i]}"

    if [[ "$MODE" == "real" ]]; then
        launch_file="${PER_DRONE_LAUNCH_REAL[$type]}"
    else
        launch_file="${PER_DRONE_LAUNCH_SIM[$type]}"
    fi

    # manual: open a teleop terminal instead of a launch file
    if [[ "$type" == "manual" ]]; then
        echo "Opening teleop terminal for $prefix..."
        gnome-terminal --title="Teleop: $prefix" -- bash -c \
            "ros2 run teleop_twist_keyboard teleop_twist_keyboard \
             --ros-args --remap cmd_vel:=${prefix}/cmd_vel; bash"
        continue
    fi

    if [[ "$launch_file" == "none" || -z "$launch_file" ]]; then
        echo "WARNING: No launch file configured for type '$type' in $MODE mode. Skipping $prefix."
        continue
    fi

    DRONE_ARGS=("robot_prefix:=$prefix")
    if [[ -n "$map" ]]; then
        DRONE_ARGS+=("map_file:=$map")
    fi

    echo "Starting $prefix ($type) → $launch_file"
    ros2 launch "$BRINGUP_PKG" "$launch_file" "${DRONE_ARGS[@]}" &
done

# -----------------------------------------------------------------------------
# Step 7: Wait for all background processes to exit
# -----------------------------------------------------------------------------

wait
