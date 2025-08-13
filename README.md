# pickup_ws
マニピュレータを動かすためにOpenrbとシリアル通信してros2にtopicに流す

## シリアル通信でopenrbと送受信するノード（モータ9個の現在角の受信とモータ9個に角度司令を送信）
- cd pickup_ws
- source install/setup.bash 
- ros2 run serial_transciever angle_serial_node 

## キーボード入力でモータの角度司令できるノード （A1がモータ1の角度をプラス90度，B1がモータ1の角度をマイナス90度）
- cd pickup_ws
- source install/setup.bash 
- ros2 run serial_transciever angle_serial_manual_node 

## joy入力でモータの角度司令できるノード
- cd pickup_ws
- source install/setup.bash 
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