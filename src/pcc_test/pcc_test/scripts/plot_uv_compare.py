# ...existing code...
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import os
from datetime import datetime
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as mtick

# ===== USER CONFIG (ここを編集すればデフォルト挙動を変えられます) =====
# ファイルを直接編集して既定動作を変えたい場合はここだけ書き換えてください。
# 例：DEFAULT_FLIP_VMODEL = True にすればスクリプトを実行するだけで v が反転します。
DEFAULT_FLIP_VMODEL = False      # True にするとデフォルトで model の v を反転
DEFAULT_SHOW_UV_TIME = False     # True で u/v の時間変化プロットを有効にする
DEFAULT_SAVE = True             # True でデフォルトは画像を保存（保存を無効にするには --no-save を使う）
DEFAULT_MM = False              # True でデフォルト単位を mm にする
DEFAULT_OUT_DIR = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..', 'jpg')
# Plot style defaults (数値や凡例の大きさ、デフォルト軸範囲をここで変更できます)
DEFAULT_TITLE_SIZE = 24
DEFAULT_LABEL_SIZE = 15
DEFAULT_TICK_SIZE = 15
DEFAULT_LEGEND_SIZE = 15
DEFAULT_MARKER_SIZE = 4
# DEFAULT_XLIM / DEFAULT_YLIM: None または (min, max) のタプル。None の場合は自動スケール
DEFAULT_XLIM = (0.4, 0.8)
# DEFAULT_XLIM = None
DEFAULT_YLIM = None
# DEFAULT tick steps (None で自動)
DEFAULT_XTICK_STEP = None
DEFAULT_YTICK_STEP = None
DEFAULT_ERR_YTICK_STEP = None
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

    # Plot appearance options
    ap.add_argument('--title-size', type=float, default=DEFAULT_TITLE_SIZE,
                    help=f'title のフォントサイズ (default: {DEFAULT_TITLE_SIZE})')
    ap.add_argument('--label-size', type=float, default=DEFAULT_LABEL_SIZE,
                    help=f'軸ラベルのフォントサイズ (default: {DEFAULT_LABEL_SIZE})')
    ap.add_argument('--tick-size', type=float, default=DEFAULT_TICK_SIZE,
                    help=f'目盛り (数値) のフォントサイズ (default: {DEFAULT_TICK_SIZE})')
    ap.add_argument('--legend-size', type=float, default=DEFAULT_LEGEND_SIZE,
                    help=f'凡例のフォントサイズ (default: {DEFAULT_LEGEND_SIZE})')
    ap.add_argument('--marker-size', type=float, default=DEFAULT_MARKER_SIZE,
                    help=f'マーカサイズ (default: {DEFAULT_MARKER_SIZE})')

    # Axis limits: 文字列 "min,max" 形式で指定。指定しないと自動スケール
    ap.add_argument('--xlim', type=str, default=None,
                    help='x 軸範囲を min,max の形式で指定 (例: --xlim -0.1,0.1)')
    ap.add_argument('--ylim', type=str, default=None,
                    help='y 軸範囲を min,max の形式で指定 (例: --ylim -0.1,0.1)')

    # display unit / tick unit options
    ap.add_argument('--display-unit', choices=['m', 'mm'], default=None,
                    help='軸ラベルに表示する単位を強制 (default: m または --mm に依存)')
    ap.add_argument('--tick-unit', action='store_true', default=False,
                    help='目盛り (tick) の数値に単位サフィックスを付ける (例: 0.4m)')
    # xtick/ytick step を引数で渡さない運用に変更（USER CONFIG を編集して設定してください）

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

    # parse xlim/ylim
    def parse_lim(s, default):
        if s is None:
            return default
        try:
            parts = [float(x) for x in s.split(',')]
            if len(parts) != 2:
                return default
            return (parts[0], parts[1])
        except Exception:
            print(f"Warning: could not parse limit '{s}', ignoring")
            return default

    xlim = parse_lim(args.xlim, DEFAULT_XLIM)
    ylim = parse_lim(args.ylim, DEFAULT_YLIM)

    # If defaults were used (ユーザーが --xlim/--ylim を与えなかった) and defaults exist,
    # scale them according to the selected data scale (args.mm)
    if args.xlim is None and DEFAULT_XLIM is not None:
        xlim = (DEFAULT_XLIM[0] * scale, DEFAULT_XLIM[1] * scale)
    if args.ylim is None and DEFAULT_YLIM is not None:
        ylim = (DEFAULT_YLIM[0] * scale, DEFAULT_YLIM[1] * scale)

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

    # decide what unit to display on axis labels
    display_unit = args.display_unit if args.display_unit is not None else unit

    # プロット数は uv-time フラグで切り替え
    if args.uv_time:
        nrows = 4
    else:
        nrows = 2

    # ★修正点 2: constrained_layout=True を削除
    fig, axes = plt.subplots(nrows, 1, figsize=(10, 4 * nrows))

    # 1) Trajectory (u-v)
    ax0 = axes[0]
    ax0.plot(u_model, v_model, '-', label='model (PCC)', markersize=args.marker_size)
    ax0.plot(u_meas,  v_meas,  '-', label='meas (Aruco)', markersize=args.marker_size)
    ax0.set_xlabel(f'u [{display_unit}]', fontsize=args.label_size); ax0.set_ylabel(f'v [{display_unit}]', fontsize=args.label_size)
    ax0.set_title('Trajectory on base plane', fontsize=args.title_size)

    ax0.grid(True)
    # apply tick label size and legend size
    ax0.tick_params(axis='both', which='major', labelsize=args.tick_size)
    ax0.legend(fontsize=args.legend_size)


    # ★★★ 修正箇所 ★★★
    # set_aspect('equal', adjustable='box') が正しく動作するように、
    # xlim と ylim の *両方* を set_aspect の *前* に明示的に固定します。

    # 1. ユーザーが --xlim/--ylim で指定した値を適用
    if xlim is not None:
        ax0.set_xlim(xlim)
    if ylim is not None:
        ax0.set_ylim(ylim)

    # 2. ユーザーが指定しなかった軸 (xlim や ylim が None のままの軸) は、
    #    データ範囲に基づいて自動計算 (autoscale_view) し、
    #    その値を get_xlim/get_ylim で取得し、
    #    set_xlim/set_ylim で *明示的に固定* します。
    
    # 2a. xlim が None (自動) だった場合、自動計算して固定
    if xlim is None:
        ax0.autoscale_view(scalex=True,scaley=False)
        current_xlim = ax0.get_xlim()
        ax0.set_xlim(current_xlim)
    
    # 2b. ylim が None (自動) だった場合、自動計算して固定
    if ylim is None:
        ax0.autoscale_view(scalex=False,scaley=True)
        current_ylim = ax0.get_ylim()
        ax0.set_ylim(current_ylim)

    # 3. これで xlim と ylim が両方とも固定された状態になりました。
    #    この状態で set_aspect を呼ぶと、matplotlib はデータ範囲 (datalim) を
    #    変更できず、ボックス (box) の形状を調整するしかなくなります。
    ax0.set_aspect('equal', adjustable='box')


    idx = 1
    if args.uv_time:
        # u vs time
        ax_u = axes[idx]; idx += 1
        ax_u.plot(t, u_model, '-o', markersize=3, label='u_model')
        ax_u.plot(t, u_meas,  '-o', markersize=3, label='u_meas')
        ax_u.set_xlabel('time [s]'); ax_u.set_ylabel(f'u [{unit}]')
        ax_u.set_title('u over time'); ax_u.grid(True); ax_u.legend()

        # v vs time
        ax_v = axes[idx]; idx += 1
        ax_v.plot(t, v_model, '-o', markersize=3, label='v_model')
        ax_v.plot(t, v_meas,  '-o', markersize=3, label='v_meas')
        ax_v.set_xlabel('time [s]'); ax_v.set_ylabel(f'v [{unit}]')
        ax_v.set_title('v over time'); ax_v.grid(True); ax_v.legend()

    # Errors vs time
    ax1 = axes[idx]; idx += 1
    # ax1.plot(t, ex, label='ex')
    # ax1.plot(t, ey, label='ey')
    ax1.plot(t, e_norm, label='||e||', markersize=args.marker_size)
    ax1.set_xlabel('time [s]', fontsize=args.label_size); ax1.set_ylabel(f'error [{display_unit}]', fontsize=args.label_size)
    ax1.set_title('Plane error over time', fontsize=args.title_size)
    ax1.grid(True)
    ax1.tick_params(axis='both', which='major', labelsize=args.tick_size)
    ax1.legend(fontsize=args.legend_size)

    # optionally append unit to tick labels for spatial axes (u/v and error)
    if args.tick_unit:

        def unit_formatter_factory(unit_str):
            def fmt(x, pos):
                # choose format: if abs(x) >= 1 -> int-like, else 2 decimals
                if abs(x) >= 1 or abs(x) == 0:
                    s = f"{x:.2f}"
                else:
                    s = f"{x:.3f}"
                return f"{s}{unit_str}"
            return fmt

        uf = mtick.FuncFormatter(unit_formatter_factory(display_unit))
        vf = mtick.FuncFormatter(unit_formatter_factory(display_unit))
        # apply to trajectory axes
        ax0.xaxis.set_major_formatter(uf)
        ax0.yaxis.set_major_formatter(vf)
        # apply to error axis y ticks
        ax1.yaxis.set_major_formatter(mtick.FuncFormatter(unit_formatter_factory(display_unit)))

    # apply tick step locators from USER CONFIG (steps are in the same unit as plotted data)
    xtick_step = DEFAULT_XTICK_STEP * scale if DEFAULT_XTICK_STEP is not None else None
    ytick_step = DEFAULT_YTICK_STEP * scale if DEFAULT_YTICK_STEP is not None else None
    err_ytick_step = DEFAULT_ERR_YTICK_STEP * scale if DEFAULT_ERR_YTICK_STEP is not None else None
    if xtick_step is not None:
        ax0.xaxis.set_major_locator(mtick.MultipleLocator(xtick_step))
    if ytick_step is not None:
        ax0.yaxis.set_major_locator(mtick.MultipleLocator(ytick_step))
    if err_ytick_step is not None:
        ax1.yaxis.set_major_locator(mtick.MultipleLocator(err_ytick_step))

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
    outpath = os.path.join(args.out_dir, f'uv_compare_{base}_{ts}.png')

    # ★修正点 2: ここで tight_layout() を呼ぶ
    plt.tight_layout()

    if save_enabled:
        plt.savefig(outpath, dpi=150)
        print(f"Saved plots to {outpath}")
    else:
        plt.show()

if __name__ == '__main__':
    main()