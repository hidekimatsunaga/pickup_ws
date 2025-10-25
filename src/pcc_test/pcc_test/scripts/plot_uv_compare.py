#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv', help='uv_compare.csv')
    ap.add_argument('--mm', action='store_true', help='mm単位で描画（既定: m）')
    args = ap.parse_args()

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

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
