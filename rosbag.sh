#!/bin/bash

ros2 bag record \
    /camera/camera/color/image_raw \
    /camera/camera/aligned_depth_to_color/image_raw \
    /camera/camera/aligned_depth_to_color/camera_info \
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
    /cmd_vel \
    /drive_odom \
    /drive_vel \
    /joy \
    /joy/set_feedback \
    /odom \
    /om_query0 \
    /om_response0 \
    /om_state0 \
    /parameter_events \
    /rosout \
    /set_position \
    /steer_angle \
    /steer_odom \
    /tf \
    /wheel_odom \
    /robot/state \
    --compression-mode file \
    --compression-format zstd \
    -o ~/pickup_ws/bags/record_$(date +%Y%m%d_%H%M%S)
