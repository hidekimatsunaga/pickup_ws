#!/bin/bash

ros2 bag record \
    /aruco/image \
    /aruco/markers \
    /aruco/poses \
    /cameraswingmotor/angle \
    /cameraswingmotor/target_angle \
    /chokudomotor/angle \
    /chokudomotor/target_angle \
    /clicked_point \
    /detected_depth_points \
    /goal_pose \
    /initialpose \
    /motor_angles \
    /motor_current_angles \
    /parameter_events \
    /rosout \
    /sensor/pressure \
    /hose/goal_point \
    /hose/neighbor_points \
    /switch \
    /start_grasp \
    /tf \
    /tf_static \
    /relay_switch \
    /sensor/pressure \
    /start_grasp \
    /vacuum_flag \
    --compression-mode file \
    --compression-format zstd \
    -o ~/pickup_ws/bags/record_$(date +%Y%m%d_%H%M%S)
