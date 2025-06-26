# pickup_ws
マニピュレータを動かすためにOpenrbとシリアル通信してros2にtopicに流す

## terminal1 シリアル通信でopenrbと送受信するノード（モータ9個の現在角の受信とモータ9個に角度司令を送信）
- cd pickup_ws
- source install/setup.bash 
- ros2 run serial_transciever angle_serial_node 

## キーボード入力でモータの角度司令できるノード　
## A1がモータ1の角度をプラス90度，B1がモータ1の角度をマイナス90度
## terminal2 
- cd pickup_ws
- source install/setup.bash 
- ros2 run serial_transciever angle_serial_manual_node 

## terminal3 joy入力でモータの角度司令できるノード
- cd pickup_ws
- source install/setup.bash 
- ros2 run serial_transciever joy_offset_command_node
