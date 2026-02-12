# scripts ディレクトリ

データ分析、可視化、カメラ操作などのユーティリティスクリプト集。

## カメラ・画像関連

### camera_video_recorder.py
ROS2トピックから画像を受信して動画として録画。

```bash
ros2 run <package> camera_video_recorder
```

**パラメータ:**
- `image_topic`: 画像トピック（デフォルト: `/camera/camera/color/image_raw`）
- `output_dir`: 出力ディレクトリ（デフォルト: `~/pickup_ws/videos`）
- `fps`: フレームレート（デフォルト: 30）

### camera_pdf_hozon.py
ROS2トピックから画像を受信してPDFとして保存。

```bash
ros2 run <package> camera_pdf_hozon
```

### depth_saver.py
ROS2の深度画像トピックから深度画像を保存。

```bash
ros2 run <package> depth_saver
```

**パラメータ:**
- `depth_topic`: 深度画像トピック
- `out_dir`: 出力ディレクトリ

### kanshicamera.py
USBカメラの映像を単純に表示する監視用スクリプト。

```bash
python3 scripts/kanshicamera.py
```

### kanshicamera_hozon.py
USBカメラの映像を録画（iPhone12相当の解像度設定対応）。

```bash
python3 scripts/kanshicamera_hozon.py
```

### movie_cut.py
保存済み動画の先頭・末尾をトリミング。

```bash
python3 scripts/movie_cut.py
```

**設定項目（スクリプト内CONFIG）:**
- `TRIM_MINUTES`: 先頭から切る時間（分）
- `TRIM_END_MINUTES`: 末尾から切る時間（分）
- `VIDEO_FILE`: 処理対象ファイル

### point_topic.py
ArUcoマーカーと検出点を画像にオーバーレイ表示。

```bash
ros2 run <package> point_topic
```

## データ分析・CSV処理

### analyze_motor10_linearity.py
ArUcoマーカーIDごとのmotor10と先端位置の線形関係を分析。

```bash
python3 scripts/analyze_motor10_linearity.py <csv_path>
```

**出力:** JSON形式の線形近似パラメータ

### check_outliers.py
ArUco-モーターログから外れ値を検出・除去（残差ベース）。

```bash
python3 scripts/check_outliers.py <csv_path>
```

### kaburisakujo.py
近接データの重複削除（x, y, z座標が閾値以内のデータを1つにまとめる）。

```bash
python3 scripts/kaburisakujo.py
```

**設定項目（スクリプト内）:**
- `input_csv_file`: 入力CSVファイルパス
- `output_csv_file`: 出力CSVファイルパス
- `threshold`: 近接判定閾値

### motor10_sweep_prediction.py
既存CSVの各行について、motor1-9を固定してmotor10を掃引し先端位置を予測。

```bash
python3 scripts/motor10_sweep_prediction.py <csv_path>
```

### update_csv_motor_angles.py
CSVファイルのモーター角度を一括更新。

```bash
python3 scripts/update_csv_motor_angles.py [options]
```

### update_motor_headers.py
キャリブレーション結果をヘッダーファイルに反映。

```bash
python3 scripts/update_motor_headers.py
```

**設定項目（スクリプト内）:**
- `NEW_MOTOR_INIT`: 新しいモーター初期位置配列

## 可視化

### plot_marker_id0.py
指定マーカーIDのデータを3Dプロット・時系列グラフで表示。

```bash
python3 scripts/plot_marker_id0.py <csv_path>
```

### visualize_motor10_sweep.py
motor10掃引データを可視化。

```bash
python3 scripts/visualize_motor10_sweep.py <csv_path>
```

## サブディレクトリ

### pcc/
PCC（Piecewise Constant Curvature）関連スクリプト。

- `adjust_uv_means.py`: UV平均値の調整
- `plot_csv_xyz.py`: CSVの3D座標プロット
- `remove_means_jumps.py`: 平均値ジャンプの除去
- `run_pcc_measure_2d.sh`: PCC測定実行シェルスクリプト

### movement/
移動シミュレーション・可視化スクリプト。

- `cmd_vel_path_visualizer.py`: cmd_velパス可視化
- `movement_simulation.py`: 移動シミュレーション
- `movement_approach.py`: 接近動作シミュレーション
- `holonomic_*.py`: 全方向移動関連
- `diff_turn_go_turn_lateral_1m.py`: 差動駆動回転移動
- その他軌道可視化スクリプト

### leader_follower/
リーダーフォロワー制御関連。

- `leader_follower_visualize.py`: リーダーフォロワー可視化
- `leader_follower_curvature_wave.py`: 曲率波動制御
- `leader_follower_snake_wave.py`: 蛇行波動制御

## 使用例

### ArUcoログから外れ値除去 → 線形関係分析
```bash
# 1. 外れ値除去
python3 scripts/check_outliers.py angle_arucopose_csv/aruco_motor_log_1213_205211.csv

# 2. 線形関係分析
python3 scripts/analyze_motor10_linearity.py angle_arucopose_csv_cleaned/aruco_motor_log_1213_cleaned.csv
```

### カメラ映像録画
```bash
# ROS2トピックから録画
ros2 run <package> camera_video_recorder

# 録画した動画の先頭1分カット
python3 scripts/movie_cut.py
```

## 注意事項

- スクリプト内の設定項目（ファイルパスなど）は環境に合わせて編集してください
- PythonスクリプトはROSワークスペースのルートから実行することを想定
- ROS2ノードとして実行するスクリプトは適切なパッケージに配置する必要があります