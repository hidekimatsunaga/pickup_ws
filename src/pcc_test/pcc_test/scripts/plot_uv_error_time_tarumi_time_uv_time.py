#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import os
from datetime import datetime
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# ===== USER CONFIG (ここだけ編集すれば既定動作を変更できます) =====
DEFAULT_FLIP_VMODEL = False      # True にするとデフォルトで model の v を反転
DEFAULT_SHOW_UV_TIME = True      # True で u/v の時間変化プロットをデフォルト表示
DEFAULT_SAVE = True              # True でデフォルトは画像を保存
DEFAULT_MM = False               # True でデフォルト単位を mm
DEFAULT_OUT_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'jpg')
# ===== USER CONFIG END =====

def main():
    ap = argparse.ArgumentParser()
    script_dir = os.path.dirname(os.path.realpath(__file__))
    default_csv = os.path.join(script_dir, '..', 'lut_csv', 'uv_compare_1029_shifted.csv')
    default_out = DEFAULT_OUT_DIR

    ap.add_argument('csv', nargs='?', default=default_csv,
                    help=f'uv_compare.csv (default: {default_csv})')
    ap.add_argument('-m', '--mm', action='store_true', default=DEFAULT_MM,
                    help='mm単位で描画（既定: m）')
    ap.add_argument('--out-dir', default=default_out,
                    help='保存先ディレクトリ (デフォルト: src/pcc_test/jpg)')
    ap.add_argument('-v', '--flip-vmodel', action='store_true', default=DEFAULT_FLIP_VMODEL,
                    help='model側のv座標の符号を反転する')
    ap.add_argument('-T', '--uv-time', action='store_true', default=DEFAULT_SHOW_UV_TIME,
                    help='u と v の時間変化プロットを表示/保存する')
    ap.add_argument('--no-save', action='store_true', default=False,
                    help='画像保存を無効にする (デフォルトは CONFIG の DEFAULT_SAVE)')

    args = ap.parse_args()

    save_enabled = DEFAULT_SAVE and (not args.no_save)

    if not os.path.exists(args.csv):
        raise FileNotFoundError(f"CSV file not found: {args.csv}")

    df = pd.read_csv(args.csv)

    scale = 1000.0 if args.mm else 1.0
    unit = 'mm' if args.mm else 'm'

    u_model = df['u_model'].to_numpy() * scale
    v_model = df['v_model'].to_numpy() * scale
    u_meas  = df['u_meas'].to_numpy()  * scale
    v_meas  = df['v_meas'].to_numpy()  * scale

    if args.flip_vmodel:
        v_model = -v_model

    t = (df['stamp'] - df['stamp'].iloc[0]).to_numpy()

    ex = (df['ex'] * scale).to_numpy()
    ey = (df['ey'] * scale).to_numpy()
    e_norm = (df['e_norm'] * scale).to_numpy()

    sw_cols = ['7','8','9']
    sw_data = {}
    for col in sw_cols:
        if col in df.columns:
            sw_data[col] = pd.to_numeric(df[col], errors='coerce').to_numpy()
        else:
            sw_data[col] = np.full_like(t, np.nan, dtype=float)

    # summary
    e = e_norm
    mean = np.nanmean(e) if len(e) else float('nan')
    med = np.nanmedian(e) if len(e) else float('nan')
    p95 = np.nanpercentile(e, 95) if len(e) else float('nan')
    p99 = np.nanpercentile(e, 99) if len(e) else float('nan')
    print(f"[summary] mean={mean:.3f}{unit}, median={med:.3f}{unit}, p95={p95:.3f}{unit}, p99={p99:.3f}{unit}")

    # プロット数は uv-time フラグで切り替え
    if args.uv_time:
        nrows = 5
    else:
        nrows = 3

    fig, axes = plt.subplots(nrows, 1, figsize=(10, 4 * nrows), constrained_layout=True)

    # 1) Trajectory (u-v)
    ax0 = axes[0]
    ax0.plot(u_model, v_model, '-o', markersize=3, label='model (PCC)')
    ax0.plot(u_meas,  v_meas,  '-o', markersize=3, label='meas (Aruco)')
    ax0.set_xlabel(f'u [{unit}]'); ax0.set_ylabel(f'v [{unit}]')
    ax0.set_title('Trajectory on base plane')
    ax0.set_aspect('equal', adjustable='datalim')
    ax0.grid(True); ax0.legend()

    idx = 1
    if args.uv_time:
        # 2) u vs time (model & meas)
        ax1 = axes[idx]
        ax1.plot(t, u_model, '-o', markersize=3, label='u_model')
        ax1.plot(t, u_meas,  '-o', markersize=3, label='u_meas')
        ax1.set_xlabel('time [s]'); ax1.set_ylabel(f'u [{unit}]')
        ax1.set_title('u over time')
        ax1.grid(True); ax1.legend()
        idx += 1

        # 3) v vs time (model & meas)
        ax2 = axes[idx]
        ax2.plot(t, v_model, '-o', markersize=3, label='v_model')
        ax2.plot(t, v_meas,  '-o', markersize=3, label='v_meas')
        ax2.set_xlabel('time [s]'); ax2.set_ylabel(f'v [{unit}]')
        ax2.set_title('v over time')
        ax2.grid(True); ax2.legend()
        idx += 1

    # Errors vs time
    ax_err = axes[idx]
    ax_err.plot(t, ex, label='ex')
    ax_err.plot(t, ey, label='ey')
    ax_err.plot(t, e_norm, label='||e||')
    ax_err.set_xlabel('time [s]'); ax_err.set_ylabel(f'error [{unit}]')
    ax_err.set_title('Plane error over time')
    ax_err.grid(True); ax_err.legend()
    idx += 1

    # Switch values vs time (7,8,9)
    ax_sw = axes[idx]
    for col in sw_cols:
        ax_sw.plot(t, sw_data[col], marker='o', linestyle='-', label=f'switch {col}')
    ax_sw.set_xlabel('time [s]'); ax_sw.set_ylabel('switch value')
    ax_sw.set_title('Switch values over time (indices 7,8,9)')
    ax_sw.grid(True); ax_sw.legend()

    os.makedirs(args.out_dir, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    base = os.path.splitext(os.path.basename(args.csv))[0]
    outpath = os.path.join(args.out_dir, f'uv_compare_with_time_{base}_{ts}.png')
    plt.tight_layout()
    if save_enabled:
        plt.savefig(outpath, dpi=150)
        print(f"Saved plots to {outpath}")
    else:
        plt.show()

if __name__ == '__main__':
    main()