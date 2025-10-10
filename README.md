# pickup_ws
ゴミ拾いロボットの移動部分とホース部分の制御に関わるws

## arucoマーカーをrealsenseで認識してそのIDと位置姿勢をtopicに流すノード
- ./pickup_ws/launch/realsense_aruco.sh

## シリアル通信でopenrbと送受信するノード（モータ9個の現在角の受信とモータ9個に角度司令を送信）
- ros2 run serial_transciever angle_serial_node 

## シリアル通信でopenrbと送受信するノード（直動モーターとカメラスイングモーターの現在の角度の受信とそれぞれのモーターに角度司令を送信）
- ros2 run serial_transciever chokudo_cameraswing_air_serial_node

## キーボード入力でモータの角度司令できるノード 
- ros2 run serial_transciever motor_manual_chokudo_node 
### ホース制御に用いる9つのモーター
#### A1：モータ1の角度をプラス90度
#### B1：モータ1の角度をマイナス90度
#### 1 600：モーター1の角度を600度 
### 直動モーター　
#### CA：現在の角度-4000度　（下がる）
#### CB：現在の角度+4000度　（上がる）
### カメラスイングモーター
#### D+：+40.0下げる
#### D-：-45.0上げる

## joy入力でモータの角度司令できるノード
- ros2 run serial_transciever joy_offset_command_node

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

- 1 30
- 2 166
- 3 81
- 4 -209
- 5 -215
- 6 -262
- 7 10
- 8 -71
- 9 -273

## アクティブキャスタの動かし方
## terminal1
- ros2 launch launch/picking_drive_launcher.launch.xml
## terminal2
- ros2 launch launch/picking_steer_launcher.launch.xml
# joy起動方法
## terminal3
- ros2 launch launch/joystic.launch.xml 