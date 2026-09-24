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
# 30 ms (100 with nvblox), base_link, nvblox=false. With nvblox=true the container runs
# cuvslam_nvblox_d435.launch.py instead: the same odometry plus nvblox 3D
# mapping from the same camera, with the projector alternating between
# frames, so each gets half the frame rate (odometry ~43 Hz instead of 89)
# and Nav2 gains a costmap layer of what the camera sees. Everything about
# the container itself is in ros2_gpu_robot/cuvslam_d435/README.md. On a
# machine without the container (the host PC) this exits with a message:
# use vo:=rtabmap there.

set -u

PROFILE=${1:-640,360,90}
JITTER=${2:-30.0}
BASE_FRAME=${3:-base_link}
NVBLOX=${4:-false}
CONTAINER=${CUVSLAM_CONTAINER:-isaac_vo}
# The jitter threshold only decides when cuVSLAM logs a "delta above
# threshold" warning; the frame is used either way. The camera drops a frame
# now and then (a quarter of them on a busy day), so a threshold near the
# frame period fills the log: 30 ms at 90 fps, 100 ms with the projector
# alternating (pairs every 22 ms nominal) still flags a real stall.
if [ "${NVBLOX}" = "true" ] || [ "${NVBLOX}" = "1" ]; then
    LAUNCH=/workspaces/isaac_ros-dev/cuvslam_nvblox_d435.launch.py
    # the splitter is built into the workspace, not installed from apt
    SOURCE_WS='[ -f /workspaces/isaac_ros-dev/install/setup.bash ] && source /workspaces/isaac_ros-dev/install/setup.bash;'
    [ "${JITTER}" = "30.0" ] && JITTER=100.0
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

# Everything a launch in the container can leave behind. SIGKILL is fine for
# these: nothing in there has state worth saving, the camera is hardware-reset
# on the next start (initial_reset), and the container's PID namespace keeps
# it away from the host. (The container's PID 1 is `sleep`, which does not
# reap, so a killed process stays as a zombie in `docker exec ps`; harmless.)
LEFTOVERS="${LAUNCH}|component_container|realsense2_camera_node|web_video_server"

launch_running() { docker exec "${CONTAINER}" pgrep -f "${LAUNCH}" >/dev/null 2>&1; }

# A previous launch may still be shutting down (this script is respawned by
# odometry.launch.py when the container's nodes die): give it 20 s. One that
# is still there after that has lost its wrapper - on 2026-09-23 a service
# restart SIGKILLed the wrapper mid-stop and the orphaned launch then blocked
# every respawn with "already running" - so it is killed, not deferred to.
if launch_running; then
    say "a previous launch is still shutting down in ${CONTAINER}; waiting"
    for _ in $(seq 1 20); do
        launch_running || break
        sleep 1
    done
    if launch_running; then
        say "the previous launch is stuck; killing what is left of it"
        docker exec "${CONTAINER}" pkill -KILL -f "${LEFTOVERS}" 2>/dev/null
        sleep 2
    fi
fi

# Called once, on the first INT or TERM. The launch on the host escalates to
# SIGKILL 30 s after its first signal (odometry.launch.py) and systemd stops
# the whole robot 40 s in, so this must be done well inside that: 10 s of
# grace for a clean shutdown, then SIGKILL for whatever is left.
stop() {
    trap '' INT TERM
    say "stopping cuVSLAM in ${CONTAINER}"
    docker exec "${CONTAINER}" pkill -INT -f "${LAUNCH}" 2>/dev/null
    for _ in $(seq 1 20); do
        launch_running || break
        sleep 0.5
    done
    if launch_running; then
        say "the launch did not stop in 10 s; killing what is left of it"
    fi
    docker exec "${CONTAINER}" pkill -KILL -f "${LEFTOVERS}" 2>/dev/null
    # the docker exec client below exits with the launch; do not leave it for
    # systemd to kill
    wait "${CHILD}" 2>/dev/null
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
