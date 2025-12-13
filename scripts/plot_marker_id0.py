#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import os

# --- 正解の値をここに設定してください（オプション）---
correct_x = None
correct_y = None
correct_z = None
# --------------------------------------------------

def plot_marker_id_0(csv_path, marker_id=0):
    """marker_id=0のデータを3Dプロットと時系列グラフで表示"""
    
    # CSV読み込み（ヘッダーなし、1行目スキップ）
    df = pd.read_csv(csv_path, header=None, skiprows=1)
    
    # marker_idは12列目（インデックス11）
    # タイムスタンプは0列目、x, y, zは13-15列目（インデックス12-14）
    timestamp = df.iloc[:, 0].astype(float)
    marker_ids = df.iloc[:, 11].astype(int)
    position = df.iloc[:, 12:15].astype(float)
    position.columns = ['x', 'y', 'z']
    
    # marker_idでフィルタリング
    mask = marker_ids == marker_id
    position_filtered = position[mask].reset_index(drop=True)
    timestamp_filtered = timestamp[mask].reset_index(drop=True)
    
    if len(position_filtered) == 0:
        print(f"Warning: marker_id={marker_id}のデータが見つかりませんでした")
        return
    
    print(f"marker_id={marker_id}のデータ数: {len(position_filtered)}")
    
    # タイムスタンプを相対時間に変換（最初を0とする）
    time_rel = timestamp_filtered - timestamp_filtered.iloc[0]
    
    # プロット
    fig = plt.figure(figsize=(14, 8))
    fig.suptitle(f'ArUco Marker ID={marker_id} (file: {os.path.basename(csv_path)})', fontsize=14)
    
    # 1. 3D位置プロット
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    
    # データポイントを描画
    ax1.scatter(position_filtered['x'].values, position_filtered['y'].values, position_filtered['z'].values, 
               color='blue', s=10, label='Measured Data', alpha=0.7)
    
    # 軌跡を描画
    ax1.plot(position_filtered['x'].values, position_filtered['y'].values, position_filtered['z'].values, 
            color='blue', alpha=0.3, linewidth=0.5)
    
    # ★★★ 正解の値をプロット（設定されている場合）★★★
    if correct_x is not None and correct_y is not None and correct_z is not None:
        ax1.scatter(correct_x, correct_y, correct_z, color='purple', s=150, 
                   marker='*', label='Correct Value', depthshade=False)
    
    # ラベル
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    
    # 原点に矢印（x, y, z軸）を描く
    ax1.quiver(0, 0, 0, 0.1, 0, 0, color='red', linewidth=2, normalize=False, arrow_length_ratio=0.2)
    ax1.quiver(0, 0, 0, 0, 0.1, 0, color='green', linewidth=2, normalize=False, arrow_length_ratio=0.2)
    ax1.quiver(0, 0, 0, 0, 0, 0.1, color='blue', linewidth=2, normalize=False, arrow_length_ratio=0.2)
    
    # 原点に大きな点
    ax1.scatter(0, 0, 0, color='black', s=50, label='Origin', depthshade=False)
    
    # 凡例
    ax1.legend()
    ax1.set_title('3D位置軌跡')
    
    # 2. X vs 時系列
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.plot(time_rel.values, position_filtered['x'].values, 'r-', linewidth=2)
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('X [m]')
    ax2.set_title('X座標の時系列変化')
    ax2.grid(True)
    
    # 3. Y vs 時系列
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.plot(time_rel.values, position_filtered['y'].values, 'g-', linewidth=2)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Y [m]')
    ax3.set_title('Y座標の時系列変化')
    ax3.grid(True)
    
    # 4. Z vs 時系列
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.plot(time_rel.values, position_filtered['z'].values, 'b-', linewidth=2)
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Z [m]')
    ax4.set_title('Z座標の時系列変化')
    ax4.grid(True)
    
    # 5. X-Y平面での軌跡
    ax5 = fig.add_subplot(2, 3, 5)
    scatter = ax5.scatter(position_filtered['x'].values, position_filtered['y'].values, 
                         c=time_rel.values, cmap='viridis', s=20, alpha=0.7)
    ax5.plot(position_filtered['x'].values, position_filtered['y'].values, 'k-', alpha=0.2, linewidth=0.5)
    ax5.set_xlabel('X [m]')
    ax5.set_ylabel('Y [m]')
    ax5.set_title('X-Y平面での軌跡')
    ax5.axis('equal')
    ax5.grid(True)
    plt.colorbar(scatter, ax=ax5, label='Time [s]')
    
    # 6. 統計情報
    ax6 = fig.add_subplot(2, 3, 6)
    ax6.axis('off')
    
    # 移動距離を計算
    dx = np.diff(position_filtered['x'].values)
    dy = np.diff(position_filtered['y'].values)
    dz = np.diff(position_filtered['z'].values)
    distances = np.sqrt(dx**2 + dy**2 + dz**2)
    total_distance = np.sum(distances)
    
    stats_text = f"""
    === 統計情報 ===
    データ点数: {len(position_filtered)}
    時間範囲: {time_rel.iloc[0]:.2f} - {time_rel.iloc[-1]:.2f} [s]
    総時間: {time_rel.iloc[-1]:.2f} [s]
    
    X範囲: {position_filtered['x'].min():.4f} - {position_filtered['x'].max():.4f} [m]
    Y範囲: {position_filtered['y'].min():.4f} - {position_filtered['y'].max():.4f} [m]
    Z範囲: {position_filtered['z'].min():.4f} - {position_filtered['z'].max():.4f} [m]
    
    総移動距離: {total_distance:.4f} [m]
    平均速度: {total_distance/time_rel.iloc[-1]:.4f} [m/s]
    """
    ax6.text(0.1, 0.5, stats_text, fontsize=10, family='monospace',
             verticalalignment='center')
    
    plt.tight_layout()
    
    # 保存
    output_path = csv_path.replace('.csv', f'_marker{marker_id}_plot.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"プロットを保存しました: {output_path}\n")
    
    plt.show()


def main():
    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
    else:
        # デフォルトは最新のファイル
        csv_dir = os.path.expanduser("~/pickup_ws/angle_arucopose_csv/")
        csv_files = [f for f in os.listdir(csv_dir) if f.startswith('aruco_motor_log_') and f.endswith('.csv')]
        if not csv_files:
            print("CSVファイルが見つかりませんでした")
            return
        csv_files.sort(reverse=True)
        csv_path = os.path.join(csv_dir, csv_files[0])
        print(f"最新のファイルを使用: {csv_path}\n")
    
    if not os.path.exists(csv_path):
        print(f"エラー: ファイルが見つかりません: {csv_path}")
        return
    
    # marker_idを指定（デフォルト0）
    marker_id = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    plot_marker_id_0(csv_path, marker_id=marker_id)


if __name__ == '__main__':
    main()
