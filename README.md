# pickup_ws
ゴミ拾いロボットの移動部分とホース部分の制御に関わるws。

## ノード一覧
### ArUco認識（Realsense）
- 実行コマンド
	```bash
	./pickup_ws/launch/realsense_aruco.sh
	```

### OpenRBシリアル（9軸モータ）
- 実行コマンド
	```bash
	ros2 run serial_transciever angle_serial_node
	```

### OpenRBシリアル（直動モータ + カメラスイング）
- 実行コマンド
	```bash
	ros2 run serial_transciever chokudo_cameraswing_air_serial_node
	```

### 統合制御ノード
- integrated_control_node
	```bash
	ros2 run serial_transciever integrated_control_node
	```

### ArUco・モーター角度記録
- angle_arucopose_csv
	```bash
	ros2 run serial_transciever angle_arucopose_csv
	```

### リレー制御
- relay_controller
	```bash
	ros2 run serial_transciever relay_controller
	```
- flag_relay_bridge
	```bash
	ros2 run serial_transciever flag_relay_bridge
	```

### hose_control（マニピュレータ制御）
- lookup_table
	```bash
	ros2 run hose_control lookup_table
	```
- feedback_goal_position_node
	```bash
	ros2 run hose_control feedback_goal_position_node
	```
- feedback_motor_publisher
	```bash
	ros2 run hose_control feedback_motor_publisher
	```
- flag_manager
	```bash
	ros2 run hose_control flag_manager
	```

### YOLOで検出した物体位置の固定サービス
- 実行コマンド
	```bash
	ros2 run target_selector target_selector_node_exe
	```

### タスクマネージャ
- task_manager_node
	```bash
	ros2 run task_manager_python task_manager_node
	```
- movement_controller_node
	```bash
	ros2 run task_manager_python movement_controller_node
	```
- manipulator_manager
	```bash
	ros2 run task_manager manipulator_manager
	```
- vacuum_manager_node
	```bash
	ros2 run task_manager vacuum_manager_node
	```

### 物体追尾（カメラスイング + 接近）
- object_chaser_node
	```bash
	ros2 run object_chaser object_chaser_node
	```
- cameraswing_calib_node（カメラスイングキャリブレーション）
	```bash
	ros2 run object_chaser cameraswing_calib_node
	```
- object_chaser_node_differential（微分型物体追尾）
	```bash
	ros2 run object_chaser object_chaser_node_differential
	```
- object_chaser_simple_camera_node（シンプルカメラ物体追尾）
	```bash
	ros2 run object_chaser object_chaser_simple_camera_node
	```

### カメラ座標系とロボット座標系のtf
- 実行コマンド
	```bash
	ros2 run tf2_ros static_transform_publisher 0.410 -0.0484 0.804 0.0 -0.4625 -2.007 base_link camera_color_optical_frame
	```

## 試験・キャリブレーション関連
### PCC試験
- pcc_move_node（PCC移動ノード）
	```bash
	ros2 run pcc_test pcc_move_node
	```
- pcc_visualizer_node（PCC可視化）
	```bash
	ros2 run pcc_test pcc_visualizer_node
	```
- pcc_target_publisher（PCCターゲット発行）
	```bash
	ros2 run pcc_test pcc_target_publisher
	```
- lut_measure_recorder（LUT測定記録）
	```bash
	ros2 run pcc_test lut_measure_recorder
	```
- pcc_error_logger（PCCエラーログ）
	```bash
	ros2 run pcc_test pcc_error_logger
	```
- lut_error_logger（LUTエラーログ）
	```bash
	ros2 run pcc_test lut_error_logger
	```

### テンドンキャリブレーション
- calib_node
	```bash
	ros2 run calib_pkg calib_node
	```

### LUT作成
- slackfree_sweeper（たるみなしスイーパー）
	```bash
	ros2 run create_lut_pkg slackfree_sweeper
	```
- motor10_sweep（モーター10スイープ）
	```bash
	ros2 run create_lut_pkg motor10_sweep
	```

## 手動操作
### キーボード入力でモータ角度司令
- 実行コマンド
	```bash
	ros2 run serial_transciever motor_manual_chokudo_node
	```

#### ホース制御に用いる9つのモーター
- A1: モータ1の角度を +90 度
- B1: モータ1の角度を -90 度
- 1 600: モータ1の角度を 600 度

#### 直動モーター
- CA: 現在の角度 -4000 度（下がる）
- CB: 現在の角度 +4000 度（上がる）

#### カメラスイングモーター
- D+: +40.0 下げる
- D-: -45.0 上げる

### joy入力でモータ角度司令
- 実行コマンド
	```bash
	ros2 run serial_transciever joy_offset_command_node
	```

## motorの初期位置
- 1 257
- 2 265
- 3 190
- 4 91
- 5 16
- 6 15
- 7 70
- 8 87
- 9 36

<!--
- 1 237.92
- 2 239.33
- 3 169.1
- 4 34.63
- 5  5.89
- 6 21.09
- 7 189.58
- 8 95.54
- 9 20.74
-->

## アクティブキャスタの動かし方
### terminal1
- 実行コマンド
	```bash
	ros2 launch launch/picking_drive_launcher.launch.xml
	```

### terminal2
- 実行コマンド
	```bash
	ros2 launch launch/picking_steer_launcher.launch.xml
	```

## joy起動方法
### terminal3
- 実行コマンド
	```bash
	ros2 launch launch/joystic.launch.xml
	``` 