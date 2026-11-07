#!/usr/bin/env bash
# Inside the skybeta container — republish /visual_slam/tracking/odometry
# (from cuVSLAM) onto /fmu/in/vehicle_visual_odometry with ENU->NED + FLU->FRD
# frame conversion and PX4 BEST_EFFORT QoS. Source: oakd_vslam_bringup.
set -euo pipefail

# Flight-critical path: the skybeta underlay only, never the team overlay, so a
# team package shadowing an underlay one cannot reach the VIO -> FC pipeline.
# skybeta_env.sh handles the nounset dance the ROS setup scripts need.
SKYBETA_UNDERLAY_ONLY=1
# shellcheck source=container_scripts/skybeta_env.sh
source skybeta_env.sh

echo "[bridge] ros2 launch oakd_vslam_bringup px4_bridge.launch.py"
exec ros2 launch oakd_vslam_bringup px4_bridge.launch.py
