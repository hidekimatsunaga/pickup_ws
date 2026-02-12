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

## Launchファイル
ワークスペースルートの `launch/` ディレクトリに統合launchファイルを配置。

### システム起動

#### realsense_aruco.sh
RealSenseカメラとArUcoマーカー認識を起動。
```bash
./launch/realsense_aruco.sh
```

#### motor_launch.py
モーターシリアル通信ノード（9軸 + 直動・カメラスイング）を起動。
```bash
ros2 launch launch/motor_launch.py
```

#### motor_relay_launch.py
モーターシリアル通信 + リレー制御を起動。
```bash
ros2 launch launch/motor_relay_launch.py
```

#### motor_manual_launch.py
モーターシリアル通信 + 手動制御 + ArUcoログ記録を起動。
```bash
ros2 launch launch/motor_manual_launch.py
```

### マニピュレータ制御

#### lookuptable.py
ターゲット選択 + フィードバック + LUT制御を起動。
```bash
ros2 launch launch/lookuptable.py
```

#### feedback_lookuptable.py
フィードバック制御 + LUT（feedback_2_node使用）を起動。
```bash
ros2 launch launch/feedback_lookuptable.py
```

#### demo_lookuptable.py
デモ用ライン目標 + LUT制御を起動。
```bash
ros2 launch launch/demo_lookuptable.py
```

### タスクマネージャ・物体追尾

#### manager_movement_launch.py
物体追尾 + タスクマネージャ + 移動制御を統合起動。
```bash
ros2 launch launch/manager_movement_launch.py
```

#### manager_movement_differential_launch.py
物体追尾（微分型） + タスクマネージャ + 移動制御を統合起動。
```bash
ros2 launch launch/manager_movement_differential_launch.py
```

#### object_chaser.launch.py
物体追尾ノード単体起動。
```bash
ros2 launch launch/object_chaser.launch.py
```

### デモ統合起動

#### pickup_demo_launcher.py
ゴミ拾いデモ用統合起動（リレー + 掃除機 + マニピュレータ + モーター）。
```bash
ros2 launch launch/pickup_demo_launcher.py
```

#### switch_relay_launch.py
リレー制御のみ起動。
```bash
ros2 launch launch/switch_relay_launch.py
```

### 移動機構

#### picking_drive_launcher.launch.xml
アクティブキャスタ駆動系を起動。
```bash
ros2 launch launch/picking_drive_launcher.launch.xml
```

#### picking_steer_launcher.launch.xml
アクティブキャスタ操舵系を起動。
```bash
ros2 launch launch/picking_steer_launcher.launch.xml
```

#### movement_launcher.launch.xml
移動制御ランチャー。
```bash
ros2 launch launch/movement_launcher.launch.xml
```

#### movement_manual_launcher.launch.xml
手動移動制御ランチャー。
```bash
ros2 launch launch/movement_manual_launcher.launch.xml
```

#### movement_manual_no_y_launcher.launch.xml
手動移動制御（y軸なし）。
```bash
ros2 launch launch/movement_manual_no_y_launcher.launch.xml
```

### センサー

#### sensor_launcher.launch.xml
センサー関連起動。
```bash
ros2 launch launch/sensor_launcher.launch.xml
```

### ジョイスティック

#### joystic.launch.xml
ジョイスティック入力を起動。
```bash
ros2 launch launch/joystic.launch.xml
```

## パッケージ内Launch
各パッケージ内にも個別launchファイルがあります：

### YOLO (yolo_bringup)
```bash
ros2 launch yolo_bringup yolo.launch.py
ros2 launch yolo_bringup yolov8.launch.py
ros2 launch yolo_bringup yolov11.launch.py
```

### ArUco姿勢推定
```bash
ros2 launch aruco_pose_estimation aruco_pose_estimation.launch.py
```

### 物体追尾（パッケージ内）
```bash
ros2 launch object_chaser_cpp object_chaser.launch.py
ros2 launch object_chaser_cpp object_chaser_differential.launch.py
``` 