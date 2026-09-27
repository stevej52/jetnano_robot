#!/bin/bash
# Save nvblox's 3D map - the camera's model of everything she has seen since the
# odometry container last started - as a .ply file you can open in MeshLab,
# Blender or Windows' 3D Viewer (2026-09-27, Steve).
#
#   bash $(ros2 pkg prefix jetnano_bringup)/lib/jetnano_bringup/save_3d_map.sh [name]
#
# listen runs it on "Rosie, save the 3D map" and after "Rosie, stop mapping".
# The file is written by nvblox inside the isaac_vo container into the shared
# workspace and linked from ~/maps3d (latest.ply is always the newest). It is in
# colour when the robot runs with color_mesh:=true, grey otherwise. The map
# lives in the odom frame and is lost when the container restarts, so save it
# before shutting down if it matters.
set -u
NAME=${1:-house-$(date +%Y%m%d-%H%M%S)}
CONTAINER=${CUVSLAM_CONTAINER:-isaac_vo}
DIR_IN=/workspaces/isaac_ros-dev/maps3d
DIR_HOST=$HOME/workspaces/isaac_ros-dev/maps3d
mkdir -p "$DIR_HOST" "$HOME/maps3d"
out=$(timeout 90 docker exec "$CONTAINER" bash -c "source /opt/ros/jazzy/setup.bash; \
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-7} FASTRTPS_DEFAULT_PROFILES_FILE=/workspaces/isaac_ros-dev/fastdds_udp_only.xml; \
    ros2 service call /nvblox_node/save_ply nvblox_msgs/srv/FilePath \"{file_path: $DIR_IN/$NAME.ply}\"" 2>&1)
if ! grep -q "success=True" <<<"$out" || [ ! -s "$DIR_HOST/$NAME.ply" ]; then
    echo "save_3d_map: nvblox did not save ($(tail -n 1 <<<"$out"))" >&2
    exit 1
fi
ln -sf "$DIR_HOST/$NAME.ply" "$HOME/maps3d/$NAME.ply"
ln -sfn "$DIR_HOST/$NAME.ply" "$HOME/maps3d/latest.ply"
echo "$HOME/maps3d/$NAME.ply"
