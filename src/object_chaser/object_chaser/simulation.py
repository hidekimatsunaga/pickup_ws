#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
simulation_gate_rgb.py

目的:
  - RGB画像で検出→その画素のDepth参照 という前提なので、最大画角はRGBに依存。
  - 開口(ライン)に「正対」して進入する必要があり、さらに開口中心に対して小さな横合わせが必要、
    という状況で、3輪アクティブキャスタ(vy可)とvy=0(対向二輪相当)を比較する。

出力:
  - gate_rgb_rotation_hist.png   : vy=0 の総旋回量の分布
  - gate_rgb_loss_hist.png       : 目標がFOV外に出た割合(見失い率)の分布(2方式)
  - gate_rgb_loss_cdf.png        : 見失い率のCDF(2方式)
  - gate_rgb_example.gif         : 代表例のアニメーション(2方式並列)

使い方:
  python3 simulation_gate_rgb.py

注:
  - HFOVはRGBの代表値 69.4 deg を使用（切り出し無し前提）。
  - FOV判定は水平のみ（BB中心で追う前提ならまず効く指標）。
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter


# =========================
# Utility
# =========================
def wrap_pi(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2 * np.pi) - np.pi

def interp_angle(a0: float, a1: float, t: float) -> float:
    da = wrap_pi(a1 - a0)
    return float(wrap_pi(a0 + da * t))

def stats(arr: np.ndarray) -> dict:
    return {
        "mean": float(np.mean(arr)),
        "median": float(np.median(arr)),
        "p90": float(np.quantile(arr, 0.90)),
        "p10": float(np.quantile(arr, 0.10)),
    }

def hose_reachable(tx, ty, base_th, R=0.4, fov_deg=180.0):
    # base座標系でのターゲット位置 (tx,ty) とベース向き base_th
    d = np.hypot(tx, ty)
    ang = wrap_pi(np.arctan2(ty, tx) - base_th)
    half = np.deg2rad(fov_deg/2)
    return (d <= R) & (np.abs(ang) <= half)

def min_translation_holonomic(tx, ty, th_face, R=0.4):
    # vyあり：向き th_face 固定で、(dx,dy) を動かして到達させる最小移動量
    # 近似：半径0.4m円の中にターゲットが入るまでの最短距離（向き制約は前方180°なので th_face固定なら常に満たす想定）
    d = np.hypot(tx, ty)
    return np.maximum(0.0, d - R)

def min_translation_vy0(tx, ty, th_face, R=0.4, n_phi=31):
    # vy=0：回転して前進のみ（その後正対に戻す）。ここでは「正対姿勢 th_face での前進」を主とし、
    # 事前回転で到達性が上がるかを角度サンプルで近似。
    # 候補：一時姿勢 th = th_face + phi で前進距離 s を選び、その後正対に戻る
    phis = np.linspace(-np.pi/2, np.pi/2, n_phi)
    best = np.inf
    for phi in phis:
        th = th_face + phi
        # 前進方向ベクトル
        ux, uy = np.cos(th), np.sin(th)
        # 1D最適：ターゲットを前進で引き寄せた後の距離がR以内になる最小s
        # target' = (tx,ty) - s*(ux,uy)
        # ||target'|| <= R を満たす最小 s >=0 を解く（二次）
        a = 1.0
        b = -2.0*(tx*ux + ty*uy)
        c = tx*tx + ty*ty - R*R
        disc = b*b - 4*a*c
        if disc < 0:
            continue
        # 不等式 a s^2 + b s + c <= 0 の解区間
        s1 = (-b - np.sqrt(disc))/(2*a)
        s2 = (-b + np.sqrt(disc))/(2*a)
        # s>=0 で最小の解
        s = s1 if s1 >= 0 else (s2 if s2 >= 0 else None)
        if s is None:
            continue
        # 並進量は前進距離 s（回転は別指標で評価）
        best = min(best, s)
    if np.isinf(best):
        return np.nan
    return best

# =========================
# Simulation core
# =========================
def simulate_gate(
    N: int = 60000,
    hfov_deg: float = 69.4,      # RGB HFOV (D435の代表値)
    r: float = 0.25,             # line手前の待機距離
    b: float = 0.20,             # line奥のターゲット距離
    s: float = 0.60,             # startがlineから離れている距離
    lat_max: float = 0.10,       # 開口中心に対する横ずれ（小さめ）
    start_lat_max: float = 1.0,  # start点の横ずれ（今回は無し固定）
    seed: int = 42,
    n_drive: int = 24,
    n_rot: int = 18,
):
    """
    ライン(開口)に正対が必要:
      - 最終的に line normal 方向 (th_face) へ向いている必要がある
    横ずれ:
      - pre点は line手前 r の位置だが、line接線方向に±lat の横ずれがある
    ターゲット:
      - lineの奥 b に配置（line normal方向）
    初期姿勢:
      - "正対が必要" なので、開始姿勢は th0 = th_face とする
        (※ 開口に正対して近づきたい、という状況のモデル化)

    方式A(vyあり):
      - 向きを維持(th0)したまま pre点へ平行移動（横合わせ＋前進）
      - 追加回転は不要（開始から正対）
    方式B(vy=0):
      - pre点へ行くためにいったん回転(th_to_pre)→直進→正対へ回転(th_face)
      - 回転が増える＝操作ステップが増える

    見失い率:
      - ターゲットが水平FOV外に出ていたフレーム割合
    """

    rng = np.random.default_rng(seed)
    hfov = np.deg2rad(hfov_deg)
    fov_half = hfov / 2.0

    # ライン(開口)の接線方向（ランダム）
    psi = rng.uniform(-np.pi, np.pi, N)
    tx, ty = np.cos(psi), np.sin(psi)
    nx, ny = -ty, tx  # 法線

    # ラインの基準点（多少ばらつき）
    x_line = rng.uniform(-0.3, 0.3, N)
    y_line = rng.uniform(-0.3, 0.3, N)

    # 横ずれ（小さめ）
    lat = rng.uniform(0.0, lat_max, N) * rng.choice([-1.0, 1.0], N)

    # pre点（ライン手前 r、さらに接線方向に横ずれ）
    x_pre = x_line - r * nx + lat * tx
    y_pre = y_line - r * ny + lat * ty

    # start点（ライン手前 r+s、さらに接線方向に少しオフセット）
    start_lat = rng.uniform(-start_lat_max, start_lat_max, N)
    x0 = x_line - (r + s) * nx + start_lat * tx
    y0 = y_line - (r + s) * ny + start_lat * ty

    # target（ライン奥 b、横ずれ無し）
    xt = x_line + b * nx
    yt = y_line + b * ny

    # 正対角（ライン法線方向）
    th_face = np.arctan2(ny, nx)
    th0 = th_face  # 開始時点で正対している（開口に正対して進入したい状況）

    # vy=0がpreへ行くために必要な向き
    th_to_pre = np.arctan2(y_pre - y0, x_pre - x0)

    # 総旋回量（deg）
    rot_A = np.degrees(np.abs(wrap_pi(th_face - th0)))  # ほぼ0
    rot_B = np.degrees(np.abs(wrap_pi(th_to_pre - th0)) + np.abs(wrap_pi(th_face - th_to_pre)))

    # 見失い率（フレームごとの判定）
    def visible(x, y, th):
        ang = np.arctan2(yt - y, xt - x)
        err = wrap_pi(ang - th)
        return np.abs(err) <= fov_half

    # A: drive only (heading fixed = th0)
    outA = np.zeros(N, dtype=np.int32)
    totA = np.zeros(N, dtype=np.int32)
    for i in range(n_drive):
        t = (i + 1) / n_drive
        x = x0 + (x_pre - x0) * t
        y = y0 + (y_pre - y0) * t
        th = th0
        outA += (~visible(x, y, th))
        totA += 1
    lostA = outA / totA

    # B: rotate -> drive -> rotate
    outB = np.zeros(N, dtype=np.int32)
    totB = np.zeros(N, dtype=np.int32)

    # rotate at start: th0 -> th_to_pre
    for i in range(n_rot):
        t = (i + 1) / n_rot
        th = wrap_pi(th0 + wrap_pi(th_to_pre - th0) * t)
        x, y = x0, y0
        outB += (~visible(x, y, th))
        totB += 1

    # drive: to pre with heading th_to_pre
    for i in range(n_drive):
        t = (i + 1) / n_drive
        x = x0 + (x_pre - x0) * t
        y = y0 + (y_pre - y0) * t
        th = th_to_pre
        outB += (~visible(x, y, th))
        totB += 1

    # rotate at pre: th_to_pre -> th_face
    for i in range(n_rot):
        t = (i + 1) / n_rot
        th = wrap_pi(th_to_pre + wrap_pi(th_face - th_to_pre) * t)
        x, y = x_pre, y_pre
        outB += (~visible(x, y, th))
        totB += 1

    lostB = outB / totB

    return rot_A, rot_B, lostA, lostB


def save_plots(rot_A, rot_B, lostA, lostB, prefix="gate_rgb"):
    # rotation hist (vy=0が分かりやすい)
    plt.figure()
    plt.hist(rot_B, bins=40, density=True, alpha=0.75)
    plt.xlabel("Total yaw rotation [deg] (vy=0)")
    plt.ylabel("Density")
    plt.title("Rotation distribution (vy=0, gate constraint)")
    rot_path = f"{prefix}_rotation_hist.png"
    plt.savefig(rot_path, dpi=200, bbox_inches="tight")
    plt.close()

    # loss hist
    plt.figure()
    bins = np.linspace(0, 0.5, 41)
    plt.hist(lostA, bins=bins, density=True, alpha=0.6, label="vy available")
    plt.hist(lostB, bins=bins, density=True, alpha=0.6, label="vy=0")
    plt.xlabel("Fraction of time target is outside FOV")
    plt.ylabel("Density")
    plt.title("Target loss under gate constraint")
    plt.legend()
    loss_hist_path = f"{prefix}_loss_hist.png"
    plt.savefig(loss_hist_path, dpi=200, bbox_inches="tight")
    plt.close()

    # loss cdf
    def cdf(arr):
        xs = np.sort(arr)
        ys = np.linspace(0, 1, len(xs), endpoint=True)
        return xs, ys

    xA, yA = cdf(lostA)
    xB, yB = cdf(lostB)
    plt.figure()
    plt.plot(xA, yA, label="vy available")
    plt.plot(xB, yB, label="vy=0")
    plt.xlabel("Fraction of time target is outside FOV")
    plt.ylabel("CDF")
    plt.title("CDF of target loss")
    plt.legend()
    loss_cdf_path = f"{prefix}_loss_cdf.png"
    plt.savefig(loss_cdf_path, dpi=200, bbox_inches="tight")
    plt.close()

    return rot_path, loss_hist_path, loss_cdf_path


# =========================
# Example GIF (illustration)
# =========================
def make_example_gif(
    hfov_deg: float = 69.4,
    r: float = 0.25,
    b: float = 0.20,
    s: float = 0.60,
    lat: float = 0.10,  # 小さめ横ずれ
    start_lat: float = 1.0,  # start点の横ずれ（今回は無し固定）
    seed: int = 7,
    n_drive: int = 14,
    n_rot: int = 10,
    out_path: str = "gate_rgb_example.gif",
):
    rng = np.random.default_rng(seed)

    # ライン方向を一つだけ取る
    psi = float(rng.uniform(-np.pi, np.pi))
    tx, ty = np.cos(psi), np.sin(psi)
    nx, ny = -ty, tx

    x_line, y_line = 0.0, 0.0
    x_pre, y_pre = x_line - r * nx + lat * tx, y_line - r * ny + lat * ty
    x0, y0 = x_line - (r + s) * nx + start_lat * tx, y_line - (r + s) * ny + start_lat * ty
    xt, yt = x_line + b * nx, y_line + b * ny

    th_face = np.arctan2(ny, nx)
    th0 = th_face
    th_to_pre = np.arctan2(y_pre - y0, x_pre - x0)

    hfov = np.deg2rad(hfov_deg)
    alpha = hfov / 2.0

    def visible(x, y, th):
        ang = np.arctan2(yt - y, xt - x)
        err = wrap_pi(ang - th)
        return np.abs(err) <= alpha

    # line segment for drawing
    seg_L = 0.9
    x1, y1 = x_line - seg_L * tx, y_line - seg_L * ty
    x2, y2 = x_line + seg_L * tx, y_line + seg_L * ty

    # Frames A: translate only (keep heading th0)
    framesA = []
    for i in range(n_drive):
        t = (i + 1) / n_drive
        framesA.append((x0 + (x_pre - x0) * t, y0 + (y_pre - y0) * t, th0))

    # Frames B: rotate -> drive -> rotate
    framesB = []
    for i in range(n_rot):
        t = (i + 1) / n_rot
        framesB.append((x0, y0, interp_angle(th0, th_to_pre, t)))
    for i in range(n_drive):
        t = (i + 1) / n_drive
        framesB.append((x0 + (x_pre - x0) * t, y0 + (y_pre - y0) * t, th_to_pre))
    for i in range(n_rot):
        t = (i + 1) / n_rot
        framesB.append((x_pre, y_pre, interp_angle(th_to_pre, th_face, t)))

    T = max(len(framesA), len(framesB))
    while len(framesA) < T:
        framesA.append(framesA[-1])
    while len(framesB) < T:
        framesB.append(framesB[-1])

    fig, axes = plt.subplots(1, 2, figsize=(9.2, 4.2))
    titles = ["vy available (translate only)", "vy=0 (rotate→drive→rotate)"]
    for ax, title in zip(axes, titles):
        ax.set_aspect("equal", "box")
        ax.set_xlim(-1.4, 1.4)
        ax.set_ylim(-1.4, 1.4)
        ax.set_title(title)
        ax.plot([x1, x2], [y1, y2])  # line
        ax.scatter([xt], [yt], marker="x", s=80)
        ax.scatter([x_line], [y_line], marker="s", s=50)
        ax.scatter([x_pre], [y_pre], marker="o", s=50)
        ax.text(xt, yt, " target", va="bottom")
        ax.text(x_line, y_line, " line", va="bottom")
        ax.text(x_pre, y_pre, " pre", va="bottom")

    robA = axes[0].scatter([x0], [y0], s=60)
    robB = axes[1].scatter([x0], [y0], s=60)
    qA = axes[0].quiver([x0], [y0], [np.cos(th0)], [np.sin(th0)], angles="xy", scale_units="xy", scale=2.6)
    qB = axes[1].quiver([x0], [y0], [np.cos(th0)], [np.sin(th0)], angles="xy", scale_units="xy", scale=2.6)

    fovA1, = axes[0].plot([], [], linestyle="--")
    fovA2, = axes[0].plot([], [], linestyle="--")
    fovB1, = axes[1].plot([], [], linestyle="--")
    fovB2, = axes[1].plot([], [], linestyle="--")

    txtA = axes[0].text(-1.35, 1.25, "", fontsize=10)
    txtB = axes[1].text(-1.35, 1.25, "", fontsize=10)

    def update(k):
        xA, yA, thA = framesA[k]
        xB, yB, thB = framesB[k]

        robA.set_offsets([[xA, yA]])
        qA.set_offsets([[xA, yA]])
        qA.set_UVC([np.cos(thA)], [np.sin(thA)])

        robB.set_offsets([[xB, yB]])
        qB.set_offsets([[xB, yB]])
        qB.set_UVC([np.cos(thB)], [np.sin(thB)])

        L = 0.8
        a1 = thA + alpha
        a2 = thA - alpha
        fovA1.set_data([xA, xA + L * np.cos(a1)], [yA, yA + L * np.sin(a1)])
        fovA2.set_data([xA, xA + L * np.cos(a2)], [yA, yA + L * np.sin(a2)])

        b1 = thB + alpha
        b2 = thB - alpha
        fovB1.set_data([xB, xB + L * np.cos(b1)], [yB, yB + L * np.sin(b1)])
        fovB2.set_data([xB, xB + L * np.cos(b2)], [yB, yB + L * np.sin(b2)])

        visA = visible(xA, yA, thA)
        visB = visible(xB, yB, thB)
        txtA.set_text("target: VISIBLE" if visA else "target: LOST")
        txtB.set_text("target: VISIBLE" if visB else "target: LOST")

        return robA, robB, qA, qB, fovA1, fovA2, fovB1, fovB2, txtA, txtB

    anim = FuncAnimation(fig, update, frames=T, interval=110, blit=False)
    anim.save(out_path, writer=PillowWriter(fps=10))
    plt.close(fig)
    return out_path


def main():
    # --- parameters (ここを調整して比較を作り込めます) ---
    HFOV_RGB = 69.4
    LAT_MAX = 0.10  # 「横ずれ小さくていい」→まず10cm上限
    START_LAT_MAX = 0.10  # start点の横ずれ（今回は無し固定）
    N = 60000

    rot_A, rot_B, lostA, lostB = simulate_gate(
        N=N,
        hfov_deg=HFOV_RGB,
        lat_max=LAT_MAX,
        start_lat_max=START_LAT_MAX,
        seed=42,
    )

    print("[Assumptions]")
    print(f"  RGB HFOV: {HFOV_RGB} deg (640x480, no crop)")
    print(f"  lat_max:  {LAT_MAX} m")
    print(f"  N:        {N}")

    print("\n[Rotation deg]")
    print("  vy available:", stats(rot_A))
    print("  vy=0:        ", stats(rot_B))

    print("\n[Target loss fraction]")
    print("  vy available:", stats(lostA))
    print("  vy=0:        ", stats(lostB))

    rot_path, loss_hist_path, loss_cdf_path = save_plots(rot_A, rot_B, lostA, lostB, prefix="gate_rgb")
    gif_path = make_example_gif(hfov_deg=HFOV_RGB, lat=LAT_MAX, out_path="gate_rgb_example.gif")

    print("\n[Saved]")
    print(" ", rot_path)
    print(" ", loss_hist_path)
    print(" ", loss_cdf_path)
    print(" ", gif_path)


if __name__ == "__main__":
    main()
