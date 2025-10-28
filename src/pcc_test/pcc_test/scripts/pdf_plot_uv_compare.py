# ...existing code...
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import os
from datetime import datetime
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# ===== USER CONFIG (ここを編集すればデフォルト挙動を変えられます) =====
# ファイルを直接編集して既定動作を変えたい場合はここだけ書き換えてください。
# 例：DEFAULT_FLIP_VMODEL = True にすればスクリプトを実行するだけで v が反転します。
DEFAULT_FLIP_VMODEL = False      # True にするとデフォルトで model の v を反転
DEFAULT_SHOW_UV_TIME = True     # True で u/v の時間変化プロットを有効にする
DEFAULT_SAVE = True             # True でデフォルトは画像を保存（保存を無効にするには --no-save を使う）
DEFAULT_MM = False              # True でデフォルト単位を mm にする
DEFAULT_OUT_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'jpg')
# ===== USER CONFIG END =====

def main():
    ap = argparse.ArgumentParser()
    # csv を省略可能にして、指定がなければスクリプト隣の uv_compare.csv を使う
    script_dir = os.path.dirname(os.path.realpath(__file__))
    default_csv = os.path.join(script_dir, '..', 'lut_csv', 'uv_compare_1029_shifted.csv')
    default_out = DEFAULT_OUT_DIR

    ap.add_argument('csv', nargs='?', default=default_csv,
                    help=f'uv_compare.csv (default: {default_csv})')
    # 単位 mm 指定（-m で短縮）
    ap.add_argument('-m', '--mm', action='store_true', default=DEFAULT_MM,
                    help='mm単位で描画（既定: m）')
    # 保存はデフォルトを CONFIG で制御。保存を無効にするには --no-save を使う
    ap.add_argument('--no-save', action='store_true', default=False,
                    help='画像保存を無効にする (デフォルトは CONFIG の DEFAULT_SAVE)')
    ap.add_argument('--out-dir', default=default_out,
                    help='保存先ディレクトリ (デフォルト: src/pcc_test/jpg)')

    # 短縮オプション -v を追加（model の v を反転）
    ap.add_argument('-v', '--flip-vmodel', action='store_true', default=DEFAULT_FLIP_VMODEL,
                    help='model側のv座標の符号を反転する')
    # u/v の時間変化プロットを有効化する短縮オプション -T
    ap.add_argument('-T', '--uv-time', action='store_true', default=DEFAULT_SHOW_UV_TIME,
                    help='u と v の時間変化プロットを表示/保存する')

    args = ap.parse_args()

    # 保存有無は CONFIG と --no-save を組み合わせて決定
    save_enabled = DEFAULT_SAVE and (not args.no_save)

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

    # プロット数は uv-time フラグで切り替え
    if args.uv_time:
        nrows = 2
    else:
        nrows = 3

    fig, axes = plt.subplots(nrows, 1, figsize=(10, 4 * nrows), constrained_layout=True)

    # 1) Trajectory (u-v)
    ax0 = axes[0]
    ax0.plot(u_model, v_model, '-', label='model (PCC)')
    ax0.plot(u_meas,  v_meas,  '-', label='meas (Aruco)')
    ax0.set_xlabel(f'u [{unit}]'); ax0.set_ylabel(f'v [{unit}]')
    ax0.set_title('Trajectory on base plane')
    ax0.set_aspect('equal', adjustable='datalim')
    ax0.grid(True); ax0.legend()

    idx = 1
    # if args.uv_time:
    #     # u vs time
    #     ax_u = axes[idx]; idx += 1
    #     ax_u.plot(t, u_model, '-o', markersize=3, label='u_model')
    #     ax_u.plot(t, u_meas,  '-o', markersize=3, label='u_meas')
    #     ax_u.set_xlabel('time [s]'); ax_u.set_ylabel(f'u [{unit}]')
    #     ax_u.set_title('u over time'); ax_u.grid(True); ax_u.legend()

    #     # v vs time
    #     ax_v = axes[idx]; idx += 1
    #     ax_v.plot(t, v_model, '-o', markersize=3, label='v_model')
    #     ax_v.plot(t, v_meas,  '-o', markersize=3, label='v_meas')
    #     ax_v.set_xlabel('time [s]'); ax_v.set_ylabel(f'v [{unit}]')
    #     ax_v.set_title('v over time'); ax_v.grid(True); ax_v.legend()

    # Errors vs time
    ax1 = axes[idx]; idx += 1
    # ax1.plot(t, ex, label='ex')
    # ax1.plot(t, ey, label='ey')
    ax1.plot(t, e_norm, label='||e||')
    ax1.set_xlabel('time [s]'); ax1.set_ylabel(f'error [{unit}]')
    ax1.set_title('Plane error over time')
    ax1.grid(True); ax1.legend()

    # # Switch values vs time (7,8,9)
    # ax2 = axes[idx]
    # for col in sw_cols:
    #     ax2.plot(t, sw_data[col], marker='o', linestyle='-', label=f'switch {col}')
    # ax2.set_xlabel('time [s]'); ax2.set_ylabel('switch value')
    # ax2.set_title('Switch values over time (indices 7,8,9)')
    # ax2.grid(True); ax2.legend()

    # 保存有効なら保存、無効なら描画（対話的表示）します
    os.makedirs(args.out_dir, exist_ok=True)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    base = os.path.splitext(os.path.basename(args.csv))[0]
    outpath = os.path.join(args.out_dir, f'uv_compare_{base}_{ts}.pdf')
    plt.tight_layout()
    if save_enabled:
        plt.savefig(outpath)
        print(f"Saved plots to {outpath}")
    else:
        plt.show()

if __name__ == '__main__':
    main()
# ...existing code...