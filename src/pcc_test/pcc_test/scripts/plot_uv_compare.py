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
    # default_csv = os.path.join(script_dir, '..', 'lut_csv', 'uv_compare_v2.csv')
    default_csv = os.path.join(script_dir, '..', 'lut_csv', 'uv_compare_v2_means_corrected_20251027_125147.csv')    
    default_out = os.path.join(script_dir, '..', '..', 'jpg')
    ap.add_argument('csv', nargs='?', default=default_csv, help=f'uv_compare.csv (default: {default_csv})')
    ap.add_argument('--mm', action='store_true', help='mm単位で描画（既定: m）')
    ap.add_argument('--save', action='store_true', help='画像をファイルへ保存')
    ap.add_argument('--out-dir', default=default_out, help='保存先ディレクトリ (デフォルト: src/pcc_test/jpg)')
    args = ap.parse_args()

    if not os.path.exists(args.csv):
        raise FileNotFoundError(f"CSV file not found: {args.csv}")

    df = pd.read_csv(args.csv)

    # 単位変換（必要なら）
    scale = 1000.0 if args.mm else 1.0
    unit = 'mm' if args.mm else 'm'

    # 2D 平面（u-v）での軌跡
    plt.figure()
    u_model = df['u_model'].to_numpy() * scale
    v_model = df['v_model'].to_numpy() * scale
    u_meas  = df['u_meas'].to_numpy()  * scale
    v_meas  = df['v_meas'].to_numpy()  * scale
    plt.plot(u_model, v_model, '-', label='model (PCC)')
    plt.plot(u_meas,  v_meas,  '-', label='meas (Aruco)')
    plt.xlabel(f'u [{unit}]'); plt.ylabel(f'v [{unit}]')
    plt.title('Trajectory on base plane')
    plt.axis('equal'); plt.grid(True); plt.legend()

    # 誤差の時系列（pandas.Series を直接渡すと matplotlib が内部で
    # numpy スタイルの多次元インデックスを行いエラーになる場合があるため
    # 明示的に numpy 配列へ変換する）
    t = (df['stamp'] - df['stamp'].iloc[0]).to_numpy()
    ex = (df['ex'] * scale).to_numpy()
    ey = (df['ey'] * scale).to_numpy()
    e_norm = (df['e_norm'] * scale).to_numpy()
    plt.figure()
    plt.plot(t, ex, label='ex')
    plt.plot(t, ey, label='ey')
    plt.plot(t, e_norm, label='||e||')
    plt.xlabel('time [s]'); plt.ylabel(f'error [{unit}]')
    plt.title('Plane error over time')
    plt.grid(True); plt.legend()

    # 誤差ヒストグラム & CDF
    e = df['e_norm'].to_numpy() * scale
    plt.figure()
    plt.subplot(2,1,1)
    plt.hist(e, bins=40)
    plt.xlabel(f'||e|| [{unit}]'); plt.ylabel('count'); plt.title('Error histogram')

    plt.subplot(2,1,2)
    ecdf_x = np.sort(e)
    ecdf_y = np.arange(1, len(e)+1)/len(e)
    plt.plot(ecdf_x, ecdf_y)
    plt.xlabel(f'||e|| [{unit}]'); plt.ylabel('CDF'); plt.grid(True)

    # 要約統計
    mean = np.mean(e); med = np.median(e); p95 = np.percentile(e,95); p99 = np.percentile(e,99)
    print(f"[summary] mean={mean:.3f}{unit}, median={med:.3f}{unit}, p95={p95:.3f}{unit}, p99={p99:.3f}{unit}")

    save_requested = args.save or (not ('DISPLAY' in os.environ or 'WAYLAND_DISPLAY' in os.environ))
    if save_requested:
        os.makedirs(args.out_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        base = os.path.splitext(os.path.basename(args.csv))[0]
        outpath = os.path.join(args.out_dir, f'uv_compare_{base}_{ts}.png')
        plt.tight_layout()
        plt.savefig(outpath, dpi=150)
        print(f"Saved plots to {outpath}")
    else:
        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    main()
