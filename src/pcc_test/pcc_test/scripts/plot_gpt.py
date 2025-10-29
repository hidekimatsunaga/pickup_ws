#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


def plot_eval(
    df,
    fig_size=(10, 8),
    # ===== デザイン系パラメータ =====
    font_sizes=None,
    line_model_width=2.0,
    line_meas_width=1.5,
    line_err_width=1.6,
    meas_marker='o',
    meas_markersize=3,
    meas_markevery=5,        # 実測マーカー間引き
    grid_alpha=0.3,
    grid_lw=0.5,
    hspace=0.35,
    # ===== 軸や表示レンジ =====
    uv_xlim=(0.50, 0.80),
    uv_ylim=(-0.10, 0.50),
    uv_aspect="auto",        # "equal"にすると幾何を正確に見せる
    force_err_start_at_zero=True,
    use_mm=False,
):
    """
    必要な列:
      stamp, u_model, v_model, u_meas, v_meas, e_norm
      (ex/eyだけのときはe_norm自動計算するフォールバックあり)
    """

    # --- 時間軸: stampから相対時間[s]を作る ---
    if "stamp" in df.columns:
        t_arr = df["stamp"].to_numpy() - df["stamp"].iloc[0]
    else:
        # 念のためfallback (今回のCSVだと使わない想定)
        if "dt" in df.columns:
            t_arr = df["dt"].cumsum().to_numpy() - df["dt"].iloc[0]
        else:
            t_arr = np.arange(len(df), dtype=float)

    # --- 誤差ノルム ---
    if "e_norm" in df.columns:
        err_arr = df["e_norm"].to_numpy()
    else:
        if {"ex", "ey"} <= set(df.columns):
            err_arr = np.sqrt(df["ex"]**2 + df["ey"]**2).to_numpy()
        else:
            du = df["u_model"] - df["u_meas"]
            dv = df["v_model"] - df["v_meas"]
            err_arr = np.sqrt(du**2 + dv**2).to_numpy()

    # --- 軌跡 ---
    u_model = df["u_model"].to_numpy()
    v_model = df["v_model"].to_numpy()
    u_meas  = df["u_meas"].to_numpy()
    v_meas  = df["v_meas"].to_numpy()

    # --- 単位スケール (mm表示したいとき用) ---
    scale = 1000.0 if use_mm else 1.0
    unit_uv  = " [mm]" if use_mm else " [m]"
    unit_err = " [mm]" if use_mm else " [m]"

    u_model_plot = u_model * scale
    v_model_plot = v_model * scale
    u_meas_plot  = u_meas  * scale
    v_meas_plot  = v_meas  * scale
    err_plot     = err_arr * scale

    uv_xlim_scaled = (uv_xlim[0] * scale, uv_xlim[1] * scale) if uv_xlim else None
    uv_ylim_scaled = (uv_ylim[0] * scale, uv_ylim[1] * scale) if uv_ylim else None

    # --- フォント設定 ---
    if font_sizes is None:
        font_sizes = {
            "title": 22,
            "axis_label": 16,
            "tick": 14,
            "legend": 16,
        }

    plt.rcParams.update({
        "axes.titlesize": font_sizes["title"],
        "axes.labelsize": font_sizes["axis_label"],
        "xtick.labelsize": font_sizes["tick"],
        "ytick.labelsize": font_sizes["tick"],
        "legend.fontsize": font_sizes["legend"],
    })

    # --- figure作成 ---
    fig, (ax_top, ax_bot) = plt.subplots(
        2, 1,
        figsize=fig_size,
        gridspec_kw={"height_ratios": [1, 1]},
        sharex=False,
    )

    # ===== 上段: u-v軌跡 =====
    ax_top.plot(
        u_model_plot,
        v_model_plot,
        color="C0",
        linewidth=line_model_width,
        label="model (PCC)",
    )

    ax_top.plot(
        u_meas_plot,
        v_meas_plot,
        color="C1",
        linewidth=line_meas_width,
        marker=meas_marker,
        markersize=meas_markersize,
        markevery=meas_markevery,
        label="meas (Aruco)",
    )

    ax_top.set_xlabel("u" + unit_uv)
    ax_top.set_ylabel("v" + unit_uv)

    if uv_xlim_scaled is not None:
        ax_top.set_xlim(*uv_xlim_scaled)
    if uv_ylim_scaled is not None:
        ax_top.set_ylim(*uv_ylim_scaled)

    ax_top.set_aspect(uv_aspect)

    ax_top.grid(True, linestyle="--", alpha=grid_alpha, linewidth=grid_lw)
    ax_top.legend(loc="best", framealpha=0.9)
    ax_top.set_title("Trajectory on base plane")

    # ===== 下段: 誤差 vs 時間 =====
    ax_bot.plot(
        t_arr,
        err_plot,
        color="C0",
        linewidth=line_err_width,
        label="‖e‖",
    )

    ax_bot.set_xlabel("time [s]")
    ax_bot.set_ylabel("error" + unit_err)

    if force_err_start_at_zero:
        ymin, ymax = ax_bot.get_ylim()
        ax_bot.set_ylim(bottom=0.0, top=ymax)

    ax_bot.grid(True, linestyle="--", alpha=grid_alpha, linewidth=grid_lw)
    ax_bot.legend(loc="best", framealpha=0.9)
    ax_bot.set_title("Plane error over time")

    # レイアウト調整
    fig.tight_layout()
    fig.subplots_adjust(hspace=hspace)

    return fig


def main():
    parser = argparse.ArgumentParser()
    script_dir = os.path.dirname(os.path.realpath(__file__))
    default_csv = os.path.join(
        script_dir,
        '..',
        'lut_csv',
        'uv_compare_1029_shifted.csv'
    )

    parser.add_argument('csv', nargs='?', default=default_csv,
                        help='path to uv_compare_*.csv')
    parser.add_argument('--mm', action='store_true',
                        help='mm単位で描画（デフォはm）')
    parser.add_argument('--save', default=None,
                        help='PNGとして保存したい出力ファイル名 (例: result.png)。指定なしなら画面表示。')
    parser.add_argument('--dpi', type=int, default=300,
                        help='保存時のdpi (デフォ:300)')

    args = parser.parse_args()

    # CSV読み込み
    df = pd.read_csv(args.csv)

    # 図を作成
    fig = plot_eval(
        df,
        use_mm=args.mm,
    )

    # 保存モード or 表示モード
    if args.save is not None:
        # 保存先が相対パスなら、スクリプトを呼んだ場所に書かれる
        fig.savefig(
            args.save,
            dpi=args.dpi,
            bbox_inches='tight',
            pad_inches=0.05,
        )
        plt.close(fig)
        print(f"saved: {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
