#!/usr/bin/env python3
"""
motor10掃引データを可視化
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys


def visualize_sweep(csv_path, max_configs=10):
    """
    motor10掃引データを可視化
    max_configs: 表示する設定数（多すぎると見づらい）
    """
    
    print(f"CSVを読み込み中: {csv_path}")
    df = pd.read_csv(csv_path)
    
    print(f"データ行数: {len(df)}\n")
    
    # タイムスタンプでグループ化（同じ元データからの掃引）
    groups = df.groupby('timestamp')
    n_configs = len(groups)
    
    print(f"元の設定数: {n_configs}")
    print(f"表示する設定数: {min(max_configs, n_configs)}\n")
    
    # サンプリング（均等に選ぶ）
    group_keys = list(groups.groups.keys())
    if n_configs > max_configs:
        step = n_configs // max_configs
        selected_keys = group_keys[::step][:max_configs]
    else:
        selected_keys = group_keys
    
    # 3Dプロット
    fig = plt.figure(figsize=(16, 10))
    
    # 3D軌跡
    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_zlabel('Z [m]')
    ax1.set_title(f'Motor10掃引による先端位置の軌跡（3D）\n{len(selected_keys)}個の設定')
    
    # X-Y平面
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.set_xlabel('X [m]')
    ax2.set_ylabel('Y [m]')
    ax2.set_title('X-Y平面')
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect('equal')
    
    # X-Z平面
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.set_xlabel('X [m]')
    ax3.set_ylabel('Z [m]')
    ax3.set_title('X-Z平面')
    ax3.grid(True, alpha=0.3)
    
    # Y-Z平面
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.set_xlabel('Y [m]')
    ax4.set_ylabel('Z [m]')
    ax4.set_title('Y-Z平面')
    ax4.grid(True, alpha=0.3)
    
    # 色マップ
    colors = plt.cm.rainbow(np.linspace(0, 1, len(selected_keys)))
    
    print("プロット中...")
    for idx, (timestamp, color) in enumerate(zip(selected_keys, colors)):
        group_data = groups.get_group(timestamp)
        
        # motor10でソート
        group_data = group_data.sort_values('motor10')
        
        x = group_data['x_predicted'].values
        y = group_data['y_predicted'].values
        z = group_data['z_predicted'].values
        motor10 = group_data['motor10'].values
        
        label = f'Config {idx+1}'
        
        # 3Dプロット
        ax1.plot(x, y, z, color=color, alpha=0.7, linewidth=2, label=label)
        ax1.scatter(x[0], y[0], z[0], color=color, s=100, marker='o', edgecolors='black', linewidth=2)
        ax1.scatter(x[-1], y[-1], z[-1], color=color, s=100, marker='s', edgecolors='black', linewidth=2)
        
        # 2Dプロット
        ax2.plot(x, y, color=color, alpha=0.7, linewidth=2)
        ax2.scatter(x[0], y[0], color=color, s=100, marker='o', edgecolors='black', linewidth=2)
        ax2.scatter(x[-1], y[-1], color=color, s=100, marker='s', edgecolors='black', linewidth=2)
        
        ax3.plot(x, z, color=color, alpha=0.7, linewidth=2)
        ax3.scatter(x[0], z[0], color=color, s=100, marker='o', edgecolors='black', linewidth=2)
        ax3.scatter(x[-1], z[-1], color=color, s=100, marker='s', edgecolors='black', linewidth=2)
        
        ax4.plot(y, z, color=color, alpha=0.7, linewidth=2)
        ax4.scatter(y[0], z[0], color=color, s=100, marker='o', edgecolors='black', linewidth=2)
        ax4.scatter(y[-1], z[-1], color=color, s=100, marker='s', edgecolors='black', linewidth=2)
        
        if idx % 10 == 0:
            print(f"  {idx+1}/{len(selected_keys)} 完了")
    
    # 凡例（3Dのみ、簡略化）
    if len(selected_keys) <= 10:
        ax1.legend(loc='upper left', fontsize=8)
    
    # マーカーの説明
    fig.text(0.5, 0.02, '○: 起点（元のmotor10）, □: 終点（motor10=-10000°付近）', 
             ha='center', fontsize=10)
    
    plt.tight_layout()
    
    # 保存
    output_path = csv_path.replace('.csv', '_visualization.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\n✅ グラフを保存: {output_path}")
    
    plt.show()


def main():
    if len(sys.argv) < 2:
        print("使用方法:")
        print("  python3 visualize_motor10_sweep.py <sweep_csv> [max_configs]")
        print("\n例:")
        print("  python3 visualize_motor10_sweep.py \\")
        print("    ~/pickup_ws/angle_arucopose_csv/aruco_motor_log_1213_203430_motor10_sweep.csv")
        print("\n  python3 visualize_motor10_sweep.py sweep.csv 20  # 20個の設定を表示")
        return
    
    csv_path = sys.argv[1]
    max_configs = int(sys.argv[2]) if len(sys.argv) > 2 else 10
    
    visualize_sweep(csv_path, max_configs)


if __name__ == '__main__':
    main()
