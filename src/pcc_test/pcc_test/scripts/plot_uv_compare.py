#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import os
from datetime import datetime
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def main():
    ap = argparse.ArgumentParser()
    # csv を省略可能にして、指定がなければスクリプト隣の uv_compare.csv を使う
    script_dir = os.path.dirname(os.path.realpath(__file__))
    default_csv = os.path.join(script_dir, '..', 'lut_csv', 'uv_compare_1028_v2.csv')
    default_out = os.path.join(script_dir, '..', '..', 'jpg')

    ap.add_argument('csv', nargs='?', default=default_csv,
                    help=f'uv_compare.csv (default: {default_csv})')
    ap.add_argument('--mm', action='store_true',
                    help='mm単位で描画（既定: m）')
    ap.add_argument('--save', action='store_true',
                    help='画像をファイルへ保存')
    ap.add_argument('--out-dir', default=default_out,
                    help='保存先ディレクトリ (デフォルト: src/pcc_test/jpg)')

    # ↓これを追加：modelのv符号をひっくり返す
    ap.add_argument('--flip-vmodel', action='store_true',
                    help='model側のv座標の符号を反転する')

    args = ap.parse_args()

    if not os.path.exists(args.csv):
        raise FileNotFoundError(f"CSV file not found: {args.csv}")

    df = pd.read_csv(args.csv)

    # 単位変換（必要なら）
    scale = 1000.0 if args.mm else 1.0
    unit = 'mm' if args.mm else 'm'

    # Prepare data arrays
    u_model = df['u_model'].to_numpy() * scale
    v_model = df['v_model'].to_numpy() * scale
    u_meas  = df['u_meas'].to_numpy()  * scale
    v_meas  = df['v_meas'].to_numpy()  * scale

    # ここで反転オプションを適用
    if args.flip_vmodel:
        v_model = -v_model

    # time relative (seconds)
    t = (df['stamp'] - df['stamp'].iloc[0]).to_numpy()

    ex = (df['ex'] * scale).to_numpy()
    ey = (df['ey'] * scale).to_numpy()
    e_norm = (df['e_norm'] * scale).to_numpy()

    # switch columns may be strings or numeric; try converting and allow NaN
    sw_cols = ['7', '8', '9']
    sw_data = {}
    for col in sw_cols:
        if col in df.columns:
            sw_data[col] = pd.to_numeric(df[col], errors='coerce').to_numpy()
        else:
            # missing column -> fill with NaN
            sw_data[col] = np.full_like(t, np.nan, dtype=float)

    # summary statistics for e_norm
    e = e_norm
    mean = np.nanmean(e) if len(e) else float('nan')
    med = np.nanmedian(e) if len(e) else float('nan')
    p95 = np.nanpercentile(e, 95) if len(e) else float('nan')
    p99 = np.nanpercentile(e, 99) if len(e) else float('nan')
    print(f"[summary] mean={mean:.3f}{unit}, median={med:.3f}{unit}, p95={p95:.3f}{unit}, p99={p99:.3f}{unit}")

    # Create a single figure with 3 vertical subplots:
    fig, axes = plt.subplots(3, 1, figsize=(10, 12), constrained_layout=True)

    # 1) Trajectory (u-v)
    ax0 = axes[0]
    ax0.plot(u_model, v_model, '-', label='model (PCC)')
    ax0.plot(u_meas,  v_meas,  '-', label='meas (Aruco)')
    ax0.set_xlabel(f'u [{unit}]'); ax0.set_ylabel(f'v [{unit}]')
    ax0.set_title('Trajectory on base plane')
    ax0.set_aspect('equal', adjustable='datalim')
    ax0.grid(True); ax0.legend()

    # 2) Errors vs time
    ax1 = axes[1]
    ax1.plot(t, ex, label='ex')
    ax1.plot(t, ey, label='ey')
    ax1.plot(t, e_norm, label='||e||')
    ax1.set_xlabel('time [s]'); ax1.set_ylabel(f'error [{unit}]')
    ax1.set_title('Plane error over time')
    ax1.grid(True); ax1.legend()

    # 3) Switch values vs time (7,8,9)
    ax2 = axes[2]
    for col in sw_cols:
        ax2.plot(t, sw_data[col], marker='o', linestyle='-', label=f'switch {col}')
    ax2.set_xlabel('time [s]'); ax2.set_ylabel('switch value')
    ax2.set_title('Switch values over time (indices 7,8,9)')
    ax2.grid(True); ax2.legend()

    # Always save the figures to PNG and do not show interactively
    os.makedirs(args.out_dir, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    base = os.path.splitext(os.path.basename(args.csv))[0]
    outpath = os.path.join(args.out_dir, f'uv_compare_{base}_{ts}.png')
    plt.tight_layout()
    plt.savefig(outpath, dpi=150)
    print(f"Saved plots to {outpath}")

if __name__ == '__main__':
    main()
