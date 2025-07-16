ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py \
  "use_depth_input:=true" \
  "aligned_depth.enable:=true" \
  "enable_sync:=true"