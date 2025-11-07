#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Usage:
  python plot_uv_compare.py [csv] [--mm] [--save out.png] [--dpi 300]
                            [--swap-uv] [--paper-view] [--equal]

- データ座標はベース基準の (u,v) とする想定。
- --swap-uv     : CSVのu,vを入れ替えて扱いたいときに使用
- --paper-view  : 論文写真の見え方（v 左＋, u 下＋）で“描画だけ”を変更
- --equal       : u-vを等方表示（アスペクト比1）
"""

import argparse
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def parse_phases(specs):
    """--phase 'LABEL:t0:t1' を [(label, t0, t1), ...] に変換"""
    phases = []
    for s in specs:
        # ラベルにコロンが入っても右から2つだけ分割
        label, t0, t1 = s.rsplit(':', 2)
        phases.append((label.strip(), float(t0), float(t1)))
    return phases

def highlight_uv_ranges(
    ax, u, v, ranges,
    base_lw=1.0, base_alpha=0.5,
    styles=None
):
    """
    u,v: 1D配列（モデルでも実測でもOK）
    ranges: [(i0,i1), ...]  0-based, i1は「含まない」終端
    styles: 各区間の見た目（lw, alpha, lsなど）を辞書で指定
    """
    u = np.asarray(u); v = np.asarray(v)

    # 全体は薄く一本
    ax.plot(u, v, lw=base_lw, alpha=base_alpha, zorder=1)

    # 区間ごとのデフォルト（濃さ・太さを段階的に変える）
    default_styles = [
        dict(lw=1.0, alpha=0.5, ls='--'),   # (i) いちばん目立たせる
        dict(lw=2.4, alpha=0.85, ls='-'),   # (ii) 少し控えめ
        dict(lw=4.0, alpha=1.00, ls='--'),  # (iii) さらに控えめ（破線）
    ]
    if styles is None:
        styles = default_styles
    else:
        # 渡されたstylesをデフォルトに上書きマージ
        merged = []
        for k,(i0,i1) in enumerate(ranges):
            base = default_styles[k % len(default_styles)].copy()
            base.update(styles[k % len(styles)])
            merged.append(base)
        styles = merged

    # 各区間だけ太く重ね描き
    for k, (i0, i1) in enumerate(ranges):
        i0 = max(0, int(i0)); i1 = min(len(u), int(i1))
        if i1 - i0 <= 1:  # 点数が少なすぎたらスキップ
            continue
        ax.plot(u[i0:i1], v[i0:i1], zorder=3, **styles[k % len(styles)])

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
    # ===== 軸や表示レンジ（u軸/x、v軸/y のレンジ指定）=====
    uv_xlim=(0.5, 0.80),
    uv_ylim=(-0.10, 0.50),
    uv_aspect="auto",        # "equal" にすると幾何を正確に見せる
    force_err_start_at_zero=True,
    use_mm=False,
    swap_uv=False,
    paper_view=False,        # 見た目だけ写真向き：x= v(左＋), y= u(下＋)
    phases=None,               # ← 追加: [(" (i) Sec3 only", 15, 55), ...]
    annotate_phases=False,     # ← 追加: True で描画
    phase_alpha=0.15,          # ← 追加: 縦帯の濃さ
):
    """
    必要な列:
      stamp, u_model, v_model, u_meas, v_meas, e_norm
      (ex/eyだけのときはe_norm自動計算のフォールバックあり)
    """

    # --- 時間軸: stampから相対時間[s]を作る ---
    if "stamp" in df.columns:
        t_arr = df["stamp"].to_numpy() - df["stamp"].iloc[0]
    else:
        if "dt" in df.columns:
            t_arr = df["dt"].cumsum().to_numpy() - df["dt"].iloc[0]
        else:
            t_arr = np.arange(len(df), dtype=float)

    # --- 軌跡データの選択（必要ならu/v入替） ---
    if swap_uv:
        u_model = df["v_model"].to_numpy()
        v_model = df["u_model"].to_numpy()
        u_meas  = df["v_meas"].to_numpy()
        v_meas  = df["u_meas"].to_numpy()
    else:
        u_model = df["u_model"].to_numpy()
        v_model = df["v_model"].to_numpy()
        u_meas  = df["u_meas"].to_numpy()
        v_meas  = df["v_meas"].to_numpy()

    # --- 誤差ノルム ---
    if "e_norm" in df.columns:
        err_arr = df["e_norm"].to_numpy()
    elif {"ex", "ey"} <= set(df.columns):
        err_arr = np.sqrt(df["ex"]**2 + df["ey"]**2).to_numpy()
    else:
        du = u_model - u_meas
        dv = v_model - v_meas
        err_arr = np.sqrt(du**2 + dv**2)

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
        font_sizes = {"title": 22, "axis_label": 16, "tick": 14, "legend": 16}

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
    # デフォルト（標準表示）: x=u, y=v
    x_model = u_model_plot
    y_model = v_model_plot
    x_meas  = u_meas_plot
    y_meas  = v_meas_plot
    x_label = "u" + unit_uv
    y_label = "v" + unit_uv
    xlim_to_apply = uv_xlim_scaled
    ylim_to_apply = uv_ylim_scaled
    x_invert = False
    y_invert = False

    # 論文写真の見え方で描くだけ変える: x=v(左+), y=u(下+)
    if paper_view:
        x_model, y_model = v_model_plot, u_model_plot
        x_meas,  y_meas  = v_meas_plot,  u_meas_plot
        x_label = "v (left +)" + unit_uv
        y_label = "u (down +)" + unit_uv
        # 範囲指定は「uの範囲→y」「vの範囲→x」へ入れ替え
        xlim_to_apply = uv_ylim_scaled
        ylim_to_apply = uv_xlim_scaled
        # 左＋/下＋に見せるために軸を反転
        x_invert = True   # 右がマイナス、左がプラスに見える
        y_invert = True   # 上がマイナス、下がプラスに見える

    ax_top.plot(
        x_model, y_model,
        color="C0", linewidth=line_model_width, label="model (PCC)",
    )

    if annotate_phases and phases:
        # 全体は薄く一発
        ax_top.plot(x_meas, y_meas, color="C1",
                    linewidth=line_meas_width*0.5, alpha=0.25,
                    label="meas (Aruco)")

        # --phase の時間→インデックス区間へ
        idx_ranges = []
        for (label, t0, t1) in phases:
            idx = np.where((t_arr >= t0) & (t_arr < t1))[0]
            if idx.size > 1:
                idx_ranges.append((int(idx[0]), int(idx[-1]) + 1))

        # ★ フェーズ共通のカラーパレット（(i),(ii),(iii)…）
        phase_colors = ['C2', 'C3', 'C4', 'C5']

        # フェーズごとのスタイル（線種/太さは好みで）
        phase_styles = []
        for i in range(len(idx_ranges)):
            c = phase_colors[i % len(phase_colors)]
            style = dict(
                color=c, lw=3.0 if i == 0 else 2.6,
                ls='-' if i != 1 else '--',
                alpha=1.0,
                marker=meas_marker, markersize=meas_markersize,
                markevery=max(1, meas_markevery//2)
            )
            phase_styles.append(style)

        # 区間だけ上書き描画（色は上で決めたものを使用）
        highlight_uv_ranges(ax_top, x_meas, y_meas, idx_ranges,
                            base_lw=0.0, base_alpha=0.0,
                            styles=phase_styles)

    else:
        ax_top.plot(x_meas, y_meas, color="C1", linewidth=line_meas_width,
                    marker=meas_marker, markersize=meas_markersize,
                    markevery=meas_markevery, label="meas (Aruco)")

    # --- ここから追記: インデックスで [i0,i1), [i2,i3) を太く ---
    # ranges = [(50, 120), (180, 230)]  # ← 強調したい区間（例）
    # styles = [
    #     dict(color='C1', lw=3.2, alpha=1.0, ls='-'),    # (i) 太め・実線
    #     dict(color='C1', lw=2.6, alpha=1.0, ls='--'),   # (ii) 少し細め・破線
    #     dict(color='C1', lw=2.0, alpha=1.0, ls='-'),    # (iii) さらに控えめ（点線）
    # ]
    # highlight_uv_ranges(
    #     ax_top, x_meas, y_meas, ranges,
    #     base_lw=0.0, base_alpha=0.0,  # すでに全体を描いているのでベースは描かない
    #     styles=styles
    # )
    ax_top.set_xlabel(x_label)
    ax_top.set_ylabel(y_label)

    if xlim_to_apply is not None:
        ax_top.set_xlim(*xlim_to_apply)
    if ylim_to_apply is not None:
        ax_top.set_ylim(*ylim_to_apply)

    # 軸反転（紙面見え方用）
    if x_invert:
        ax_top.invert_xaxis()
    if y_invert:
        ax_top.invert_yaxis()

    ax_top.set_aspect(uv_aspect)
    ax_top.grid(True, linestyle="--", alpha=grid_alpha, linewidth=grid_lw)
    ax_top.legend(loc="best", framealpha=0.9)
    ax_top.set_title("Trajectory on base plane")

    # 上段の plot 後に追加
    # ax_top.plot(x_model[0], y_model[0], 's', ms=6, color='C0')
    # ax_top.plot(x_meas[0],  y_meas[0],  's', ms=6, color='C1')
    # ax_top.plot(x_model[-1], y_model[-1], '^', ms=6, color='C0')
    # ax_top.plot(x_meas[-1],  y_meas[-1],  '^', ms=6, color='C1')

    # ===== 下段: 誤差 vs 時間 =====
    ax_bot.plot(t_arr, err_plot, color="C0", linewidth=line_err_width, label="‖e‖")
    ax_bot.set_xlabel("time [s]")
    ax_bot.set_ylabel("error" + unit_err)
    if force_err_start_at_zero:
        ymin, ymax = ax_bot.get_ylim()
        ax_bot.set_ylim(bottom=0.0, top=ymax)
    ax_bot.grid(True, linestyle="--", alpha=grid_alpha, linewidth=grid_lw)
    ax_bot.legend(loc="best", framealpha=0.9)
    ax_bot.set_title("Plane error over time")


    # ← ここから追加
    if annotate_phases and phases:
        ymin, ymax = ax_bot.get_ylim()
        phase_colors = ['C2', 'C3', 'C4', 'C5']  # ↑と同じ配列を使う
        boundaries = set()

        for i, (label, t0, t1) in enumerate(phases):
            c = phase_colors[i % len(phase_colors)]
            # 縦帯は同色で薄く
            ax_bot.axvspan(t0, t1, facecolor=c, alpha=phase_alpha,
                           edgecolor='none', zorder=0)
            # フェーズ境界も同色の破線
            for b in (t0, t1):
                ax_bot.axvline(b, color=c, linestyle='--', linewidth=1.2, zorder=1)
                boundaries.add(b)
            # ラベルも同色で枠線だけ色付き
            ax_bot.text((t0 + t1)/2, ymax*0.96, label,
                        ha='center', va='top', fontsize=12, color=c,
                        bbox=dict(boxstyle='round,pad=0.2', fc='white', ec=c, alpha=0.95),
                        zorder=5)

        # フェーズ境界を破線で強調
        for b in sorted(boundaries):
            ax_bot.axvline(b, color='0.4', linestyle='--', linewidth=1.0, zorder=1)
    # mean = np.mean(err_plot); med = np.median(err_plot); p95 = np.percentile(err_plot, 95)
    # ax_bot.text(0.99, 0.98,
    #             f"mean {mean*1000:.1f} mm\nmedian {med*1000:.1f} mm\n95% {p95*1000:.1f} mm",
    #             transform=ax_bot.transAxes, ha='right', va='top', fontsize=12,
    #             bbox=dict(boxstyle='round,pad=0.3', fc='white', ec='0.7'))


    # レイアウト調整
    fig.tight_layout()
    fig.subplots_adjust(hspace=hspace)
    return fig


def main():
    parser = argparse.ArgumentParser()
    script_dir = os.path.dirname(os.path.realpath(__file__))
    default_csv = os.path.join(script_dir, '..', 'lut_csv', 'uv_compare_1029_shifted.csv')

    parser.add_argument('csv', nargs='?', default=default_csv, help='path to uv_compare_*.csv')
    parser.add_argument('--mm', action='store_true', help='mm単位で描画（デフォはm）')
    parser.add_argument('--save', default=None,
                        help='PNGとして保存したい出力ファイル名 (例: result.png)。指定なしなら画面表示。')
    parser.add_argument('--dpi', type=int, default=300, help='保存時のdpi (デフォ:300)')
    parser.add_argument('--swap-uv', action='store_true',
                        help='CSVのu,vを入れ替えて扱う（例: ログが画像座標基準のとき）')
    parser.add_argument('--paper-view', action='store_true',
                        help='写真の見え方に合わせて x=v(左＋), y=u(下＋) で描画（データは不変）')
    parser.add_argument('--equal', action='store_true',
                        help='u-vを等方表示（アスペクト比1）')
    parser.add_argument('--annotate-phases', action='store_true',
                        help='(i)(ii)(iii) の時間区間を図に注記する')
    parser.add_argument('--phase', action='append', default=[], metavar='"LABEL:t0:t1"',
                        help='区間を追加（秒）。例: --phase "(i) Sec3 only:15:55"')


    args = parser.parse_args()

    # CSV読み込み
    df = pd.read_csv(args.csv)

    phases = parse_phases(args.phase) if args.phase else None

    # 図を作成
    fig = plot_eval(
        df,
        use_mm=args.mm,
        swap_uv=args.swap_uv,
        paper_view=args.paper_view,
        uv_aspect=("equal" if args.equal else "auto"),
        phases=phases,
        annotate_phases=args.annotate_phases,
    )

    # 保存モード or 表示モード
    if args.save is not None:
        fig.savefig(args.save, dpi=args.dpi, bbox_inches='tight', pad_inches=0.05)
        plt.close(fig)
        print(f"saved: {args.save}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
