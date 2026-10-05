# the simulator's own ROS network: domain 42, this machine only - it can never reach Rosie (domain 7)
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
unset FASTRTPS_DEFAULT_PROFILES_FILE
