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
#                        frontier-exploration | map-user
#     extra      Optional third field:
#                  - sim drones (non map-user): Gazebo world name
#                    e.g. maze, crazyflie_world, circle-maze
#                    Only one drone needs to specify this — all share a world.
#                  - map-user drones: absolute path to a saved map.yaml
#                    Omit to start with a blank map.
#
# Examples:
#   ./launch-universal.sh sim cf1:frontier-exploration:maze cf2:frontier-exploration
#   ./launch-universal.sh sim cf1:map-user:/home/ryan/maps/room.yaml
#   ./launch-universal.sh real cf1:frontier-exploration cf2:map-user
#   ./launch-universal.sh real cf1:manual
# =============================================================================

# -----------------------------------------------------------------------------
# Paths
# -----------------------------------------------------------------------------

BASE_DIR=$(readlink -f "$(dirname "${BASH_SOURCE[0]}")")
ROS2_WS="$BASE_DIR/../core/crazyflie_mapping_demo/ros2_ws"
BRINGUP_PKG="crazyflie_ros2_multiranger_bringup"   # ROS 2 package containing all launch files

VALID_TYPES=("wallfollowing" "manual" "square" "frontier-exploration" "map-user")

# -----------------------------------------------------------------------------
# LAUNCH FILE MAPPING
#
# Maps each drone type to its shared and per-drone launch files.
# Edit this section if you rename a file or add a new drone type.
#
# Format:
#   SHARED_LAUNCH_REAL   — launched once for all drones in real mode
#   SHARED_LAUNCH_SIM    — launched once for all drones in sim mode
#   PER_DRONE_LAUNCH_<TYPE>_REAL  — launched once per drone in real mode
#   PER_DRONE_LAUNCH_<TYPE>_SIM   — launched once per drone in sim mode
#
# Use "none" if a type has no per-drone launch file for that mode
# (e.g. manual — its teleop terminal is opened directly by this script).
# -----------------------------------------------------------------------------

SHARED_LAUNCH_REAL="shared_real.launch.py"
SHARED_LAUNCH_SIM="shared_sim.launch.py"

declare -A PER_DRONE_LAUNCH_REAL=(
    ["frontier-exploration"]="frontier_exploration_real.launch.py"
    ["wallfollowing"]="wallfollowing_real.launch.py"
    ["square"]="square_real.launch.py"
    ["map-user"]="map_user_real.launch.py"
    ["manual"]="none"
)

declare -A PER_DRONE_LAUNCH_SIM=(
    ["frontier-exploration"]="frontier_exploration_sim.launch.py"
    ["wallfollowing"]="wallfollowing_sim.launch.py"
    ["square"]="square_sim.launch.py"
    ["map-user"]="map_user_sim.launch.py"
    ["manual"]="none"
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
    echo "    wallfollowing        Follow walls autonomously"
    echo "    manual               Keyboard teleoperation"
    echo "    square               Fly a square pattern"
    echo "    frontier-exploration Explore unknown space autonomously"
    echo "    map-user             Navigate using a pre-built or blank map"
    echo ""
    echo "  Third field (optional):"
    echo "    sim mode:            Gazebo world name (e.g. maze, crazyflie_world)"
    echo "                         Only specify on one drone — all drones share the same world."
    echo "    map-user (any mode): Absolute path to map.yaml"
    echo "                         Omit to start with a blank map."
    echo ""
    echo "Examples:"
    echo "  ./launch-universal.sh sim cf1:frontier-exploration:maze cf2:frontier-exploration"
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
shift  # Remove MODE so "$@" now contains only drone specs

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
#
# Each spec:  cf1:frontier-exploration:maze
#              ^prefix  ^type           ^extra (world name or map path)
# -----------------------------------------------------------------------------

declare -a PREFIXES
declare -a TYPES
declare -a MAPS
WORLD=""

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

    # Extract the Gazebo world name from the third field when:
    #   - It is non-empty
    #   - It does NOT start with '/' (file paths start with /; world names don't)
    #   - The type is not map-user (map-user uses the third field for a map path)
    #   - We haven't already found a world name
    if [[ -n "$extra" && "$extra" != /* && "$type" != "map-user" && -z "$WORLD" ]]; then
        WORLD="$extra"
    fi
done

DRONE_COUNT=${#PREFIXES[@]}

# Build the prefixes list string for the shared launch file: [/cf1,/cf2]
PREFIXES_LIST=$(IFS=','; echo "[${PREFIXES[*]}]")

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
echo ""

# -----------------------------------------------------------------------------
# Step 5: Launch the shared file
# Starts: crazyflie_server (real) or Gazebo simulator (sim) + shared_mapper + rviz
# -----------------------------------------------------------------------------

if [[ "$MODE" == "real" ]]; then
    SHARED_FILE="$SHARED_LAUNCH_REAL"
    SHARED_ARGS=("robot_prefixes:=$PREFIXES_LIST")
else
    SHARED_FILE="$SHARED_LAUNCH_SIM"
    SHARED_ARGS=("robot_prefixes:=$PREFIXES_LIST")
    [[ -n "$WORLD" ]] && SHARED_ARGS+=("world:=$WORLD")
fi

echo "Starting shared nodes ($MODE)..."
ros2 launch "$BRINGUP_PKG" "$SHARED_FILE" "${SHARED_ARGS[@]}" &

# -----------------------------------------------------------------------------
# Step 6: Launch one per-drone file per drone
# Each drone gets: vel_mux (real only) + its behaviour node(s)
# -----------------------------------------------------------------------------

for i in "${!PREFIXES[@]}"; do
    prefix="${PREFIXES[$i]}"
    type="${TYPES[$i]}"
    map="${MAPS[$i]}"

    # Look up the correct per-drone launch file for this type and mode
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

    # Warn and skip if no launch file is configured for this type+mode
    if [[ "$launch_file" == "none" || -z "$launch_file" ]]; then
        echo "WARNING: No launch file configured for type '$type' in $MODE mode. Skipping $prefix."
        continue
    fi

    # Warn and skip if the launch file is missing from disk
    DRONE_FILE="$launch_file"
    if [[ -z "$DRONE_FILE" ]]; then
        echo "WARNING: No launch file configured for type '$type' in $MODE mode. Skipping $prefix."
        continue
    fi

    # Build per-drone launch arguments
    DRONE_ARGS=("robot_prefix:=$prefix")
    # Only pass map_file if non-empty — ROS 2 rejects empty string argument values
    if [[ -n "$map" ]]; then
        DRONE_ARGS+=("map_file:=$map")
    fi

    echo "Starting $prefix ($type) → $launch_file"
    ros2 launch "$BRINGUP_PKG" "$DRONE_FILE" "${DRONE_ARGS[@]}" &
done

# -----------------------------------------------------------------------------
# Step 7: Wait for all background processes to exit
# -----------------------------------------------------------------------------

wait
