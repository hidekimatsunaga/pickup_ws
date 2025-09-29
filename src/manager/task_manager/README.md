# task_manager (C++)

`task_manager_node` を提供する最小構成の ROS 2 C++ パッケージです。

## 使い方（クイックスタート）

1) ワークスペースのルートでこのパッケージのみビルド

   ```bash
   colcon build --packages-select task_manager
   ```

2) セットアップを読み込んで起動

   ```bash
   source install/setup.bash
   ros2 launch task_manager task_manager.launch.py
   ```

## パッケージ構成

- `src/task_manager_node.cpp`: シンプルなハートビートを出力するノード本体
- `launch/task_manager.launch.py`: ノード起動用の launch ファイル
- `CMakeLists.txt`, `package.xml`: ビルド・依存関係設定

## 依存関係

- ROS 2（`rclcpp`）
- `ament_cmake`

## ライセンス

Apache-2.0
