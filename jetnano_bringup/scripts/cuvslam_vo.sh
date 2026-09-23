#!/usr/bin/env bash
# Copyright 2026 stevej52
# Licensed under the Apache License, Version 2.0. See LICENSE.
#
# Run the robot's GPU visual odometry: NVIDIA cuVSLAM (Isaac ROS 4.6) inside
# the isaac_vo container, on the D435's infrared pair, publishing /vo.
#
# odometry.launch.py starts this with vo:=cuvslam. It is a thin wrapper around
# `docker exec` that does the three things a launch file cannot: check that the
# container exists and start it if it is stopped, refuse to run against a
# camera the host driver already owns, and stop the launch inside the container
# when this process is told to stop (a plain docker exec client ignores
# SIGINT, and the nodes would keep running after Ctrl-C).
#
#     cuvslam_vo.sh [infra_profile] [image_jitter_threshold_ms] [base_frame] [nvblox]
#
# Defaults: 640,360,90 (the fastest stable profile measured on 2026-09-23),
# 12 ms, base_link, nvblox=false. With nvblox=true the container runs
# cuvslam_nvblox_d435.launch.py instead: the same odometry plus nvblox 3D
# mapping from the same camera, with the projector alternating between
# frames, so each gets half the frame rate (odometry ~43 Hz instead of 89)
# and Nav2 gains a costmap layer of what the camera sees. Everything about
# the container itself is in ros2_gpu_robot/cuvslam_d435/README.md. On a
# machine without the container (the host PC) this exits with a message:
# use vo:=rtabmap there.

set -u

PROFILE=${1:-640,360,90}
JITTER=${2:-12.0}
BASE_FRAME=${3:-base_link}
NVBLOX=${4:-false}
CONTAINER=${CUVSLAM_CONTAINER:-isaac_vo}
if [ "${NVBLOX}" = "true" ] || [ "${NVBLOX}" = "1" ]; then
    LAUNCH=/workspaces/isaac_ros-dev/cuvslam_nvblox_d435.launch.py
    # the splitter is built into the workspace, not installed from apt
    SOURCE_WS='[ -f /workspaces/isaac_ros-dev/install/setup.bash ] && source /workspaces/isaac_ros-dev/install/setup.bash;'
    [ "${JITTER}" = "12.0" ] && JITTER=50.0   # pairs arrive at half rate with the projector alternating
else
    LAUNCH=/workspaces/isaac_ros-dev/cuvslam_d435_stereo.launch.py
    SOURCE_WS=''
fi
DDS_PROFILE=/workspaces/isaac_ros-dev/fastdds_udp_only.xml
DOMAIN=${ROS_DOMAIN_ID:-0}

say() { echo "cuvslam_vo: $*" >&2; }

if ! command -v docker >/dev/null 2>&1; then
    say "docker is not installed here; GPU visual odometry runs on the Jetson only. Use vo:=rtabmap."
    exit 1
fi

if ! docker ps -q -f "name=^${CONTAINER}$" | grep -q .; then
    if docker ps -aq -f "name=^${CONTAINER}$" | grep -q .; then
        # Right after boot the GPU driver can still be initialising, and the
        # NVIDIA runtime refuses to create the container until it is ready.
        # isaac-vo.service normally has this done already; retry for a while.
        started=0
        for attempt in $(seq 1 8); do
            if [ -e /dev/nvgpu/igpu0 ] && docker start "${CONTAINER}" >/dev/null 2>/tmp/cuvslam_vo_start.err; then
                started=1; break
            fi
            say "container ${CONTAINER} not started yet (attempt ${attempt}/8): $(tail -n 1 /tmp/cuvslam_vo_start.err 2>/dev/null)"
            sleep 5
        done
        if [ "${started}" -ne 1 ]; then
            say "could not start ${CONTAINER}; is the GPU up? (dmesg | grep nvgpu, ros2 run gpu_tools gpu_info). Use vo:=rtabmap meanwhile."
            exit 1
        fi
        sleep 2
    else
        say "container ${CONTAINER} does not exist (see ros2_gpu_robot/cuvslam_d435/README.md). Use vo:=rtabmap."
        exit 1
    fi
fi

# The camera can have one owner. A RealSense driver running on the host (not in
# a container) means sensors.launch.py was started with use_camera:=true.
for pid in $(pgrep -x realsense2_came 2>/dev/null); do
    if ! grep -qE 'docker|containerd' "/proc/${pid}/cgroup" 2>/dev/null; then
        say "a RealSense driver is already running on the host (pid ${pid}); stop it or launch with use_camera:=false"
        exit 1
    fi
done

if docker exec "${CONTAINER}" pgrep -f "${LAUNCH}" >/dev/null 2>&1; then
    say "cuVSLAM is already running in ${CONTAINER}; not starting a second one"
    exit 1
fi

stop() {
    say "stopping cuVSLAM in ${CONTAINER}"
    docker exec "${CONTAINER}" pkill -INT -f "${LAUNCH}" 2>/dev/null
    for _ in $(seq 1 30); do
        docker exec "${CONTAINER}" pgrep -f "${LAUNCH}" >/dev/null 2>&1 || break
        sleep 0.5
    done
    docker exec "${CONTAINER}" pkill -TERM -f 'component_container|realsense2_camera_node' 2>/dev/null
}
trap 'stop; exit 0' INT TERM

say "starting $(basename "${LAUNCH}" .launch.py) in ${CONTAINER}: IR ${PROFILE}, jitter ${JITTER} ms, base_frame ${BASE_FRAME}, nvblox ${NVBLOX}, ROS_DOMAIN_ID ${DOMAIN}"
if [ "${NVBLOX}" = "true" ] || [ "${NVBLOX}" = "1" ]; then
    LAUNCH_ARGS="profile:=${PROFILE} image_jitter_threshold_ms:=${JITTER} base_frame:=${BASE_FRAME}"
else
    LAUNCH_ARGS="infra_profile:=${PROFILE} emitter:=0 image_jitter_threshold_ms:=${JITTER} base_frame:=${BASE_FRAME}"
fi
docker exec -u root \
    -e FASTRTPS_DEFAULT_PROFILES_FILE="${DDS_PROFILE}" -e ROS_DOMAIN_ID="${DOMAIN}" \
    "${CONTAINER}" bash -c "source /opt/ros/jazzy/setup.bash; ${SOURCE_WS} exec ros2 launch ${LAUNCH} ${LAUNCH_ARGS}" &
CHILD=$!
wait "${CHILD}"
status=$?
say "cuVSLAM launch exited with status ${status}"
exit "${status}"
