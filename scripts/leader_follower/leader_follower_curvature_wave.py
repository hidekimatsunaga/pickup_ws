#!/usr/bin/env python3
"""Leader-follower visualization for a continuum manipulator (signed bend propagation).

ユーザ定義のリーダーフォロワー（先端→根元へ伝播）を、
"曲率が時間遅れで根元側へ伝わる" という形で簡易モデル化します。

ポイント:
- 1本のマニピュレータ（2Dバックボーン）
- 先端が出す「曲げ角」入力 $\theta_{tip}(t)$（符号付き）が、速度 c で根元側へ伝播
    $\theta(s,t) = \theta_{tip}(t - (L-s)/c) \cdot (s/L)$
- $\theta(s,t)$ をそのまま接線角として積分して形状 (x(s), y(s)) を生成
    → 断面ごとに遅れが入るので「追従している感」が出やすい
- 折り返し（符号反転）も自然に伝播
- GIF保存して、その後に表示まで同一スクリプトで完結

物理的に厳密なCosseratロッドではなく、視覚化用のトイモデルです。
"""

from __future__ import annotations

import math
import os
import subprocess
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib import font_manager

try:
    from PIL import Image  # pillow
except Exception as exc:  # pragma: no cover
    raise SystemExit(
        "Pillow が必要です。`pip install pillow matplotlib numpy` を実行してください。"
    ) from exc


@dataclass(frozen=True)
class SimParams:
    # Geometry
    n_points: int = 70
    length: float = 1.0

    # Time
    dt: float = 0.03
    n_steps: int = 60

    # Leader input: tip bend angle (signed)
    theta_amp: float = 1.3  # [rad]  (bigger -> larger swing and curvature)
    theta_w: float = 2.0 * math.pi * 0.22

    # Propagation (tip -> base)
    propagation_speed: float = 0.7  # [length/s], bigger -> faster propagation
    time_smoothing: float = 0.45  # 0..1, larger -> more responsive
    space_smoothing: float = 0.12  # 0..1, smaller -> allows sharper bends

    # Forward motion (entering from off-screen)
    forward_speed: float = 0.25  # [length/s], speed of forward motion
    forward_direction: tuple[float, float] = (1.0, 0.0)  # direction vector (will be normalized)

    # Rendering
    fps: int = 20
    gif_name: str = "leader_follower_curvature_wave.gif"
    dpi: int = 180
    trail_len: int = 140

    # Slide-friendly visuals
    figsize: tuple[float, float] = (5.5, 4.5)
    use_japanese_labels: bool = True
    save_keyframe_png: bool = True
    keyframe_index: int = 0  # overwritten to mid-step if 0
    show_numbers: bool = False


def _set_slide_style(use_japanese: bool) -> None:
    """Apply slide-friendly matplotlib defaults (fonts/linewidths/colors)."""
    plt.rcParams.update(
        {
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "axes.edgecolor": "#333333",
            "axes.labelcolor": "#111111",
            "xtick.color": "#111111",
            "ytick.color": "#111111",
            "text.color": "#111111",
            "axes.titlesize": 15,
            "axes.labelsize": 12,
            "font.size": 12,
            "legend.fontsize": 11,
            "lines.linewidth": 2.8,
        }
    )

    if not use_japanese:
        return

    # Prefer common Japanese fonts if available (fallback-safe).
    candidates = [
        "Noto Sans CJK JP",
        "Noto Sans JP",
        "IPAexGothic",
        "IPAPGothic",
        "TakaoGothic",
        "Yu Gothic",
    ]
    available_names = {f.name for f in font_manager.fontManager.ttflist}
    for name in candidates:
        if name in available_names:
            plt.rcParams["font.family"] = [name, "DejaVu Sans"]
            break


def leader_theta_tip(t: float, p: SimParams) -> float:
    """Signed tip bend angle (leader command)."""
    return p.theta_amp * math.sin(p.theta_w * t)


def _delayed_from_history(hist: np.ndarray, step: int, s: np.ndarray, p: SimParams) -> np.ndarray:
    """Compute delayed signal value at each s using history and fractional delay."""
    L = p.length
    c = max(1e-6, p.propagation_speed)
    dt = p.dt

    # Delay is larger near the base.
    delays = (L - s) / c
    delay_steps = delays / dt

    idx_f = step - delay_steps
    idx0 = np.floor(idx_f).astype(int)
    frac = (idx_f - idx0).astype(float)

    idx0 = np.clip(idx0, 0, len(hist) - 2)
    idx1 = idx0 + 1

    v0 = hist[idx0]
    v1 = hist[idx1]
    return (1.0 - frac) * v0 + frac * v1


def theta_to_backbone(theta: np.ndarray, p: SimParams) -> tuple[np.ndarray, np.ndarray]:
    """Integrate tangent angle into a planar backbone curve (x(s), y(s)), base fixed."""
    n = p.n_points
    ds = p.length / (n - 1)
    x = np.zeros(n, dtype=float)
    y = np.zeros(n, dtype=float)
    x[1:] = np.cumsum(np.cos(theta[:-1])) * ds
    y[1:] = np.cumsum(np.sin(theta[:-1])) * ds
    return x, y


def simulate(p: SimParams) -> dict[str, np.ndarray]:
    n = p.n_points
    s = np.linspace(0.0, p.length, n)

    # Precompute tip angle history
    th_hist = np.zeros(p.n_steps + 2, dtype=float)
    for step in range(p.n_steps + 2):
        th_hist[step] = leader_theta_tip(step * p.dt, p)

    xs = np.zeros((p.n_steps, n), dtype=float)
    ys = np.zeros((p.n_steps, n), dtype=float)
    thetas = np.zeros((p.n_steps, n), dtype=float)

    # Smoothing state
    theta_prev = np.zeros(n, dtype=float)

    # Forward motion offset
    fwd_dir = np.array(p.forward_direction, dtype=float)
    fwd_dir = fwd_dir / (np.linalg.norm(fwd_dir) + 1e-9)

    # Markers (4 points -> 3 sections)
    marker_ids = np.array(
        [
            0,
            int(round((1.0 / 3.0) * (n - 1))),
            int(round((2.0 / 3.0) * (n - 1))),
            n - 1,
        ],
        dtype=int,
    )
    marker_xy = np.zeros((p.n_steps, len(marker_ids), 2), dtype=float)

    for step in range(p.n_steps):
        # Delayed tip angle for each cross-section
        th_delayed = _delayed_from_history(th_hist, step, s, p)

        # Map to a backbone tangent-angle field.
        # Multiply by (s/L) so base stays roughly aligned while the bend propagates.
        theta_raw = th_delayed * (s / p.length)

        # Time smoothing (first-order low-pass)
        a_t = float(np.clip(p.time_smoothing, 0.0, 1.0))
        theta_sm = (1.0 - a_t) * theta_prev + a_t * theta_raw

        # Space smoothing (one Jacobi step on a Laplacian)
        a_s = float(np.clip(p.space_smoothing, 0.0, 1.0))
        theta_new = theta_sm.copy()
        if a_s > 0.0:
            lap = np.zeros_like(theta_new)
            lap[1:-1] = theta_new[0:-2] - 2.0 * theta_new[1:-1] + theta_new[2:]
            lap[0] = theta_new[1] - theta_new[0]
            lap[-1] = theta_new[-2] - theta_new[-1]
            theta_new = theta_new + a_s * lap

        theta_prev = theta_new
        x, y = theta_to_backbone(theta_new, p)

        # Apply forward motion offset
        t_current = step * p.dt
        offset = p.forward_speed * t_current * fwd_dir
        x = x + offset[0]
        y = y + offset[1]

        thetas[step] = theta_new
        xs[step] = x
        ys[step] = y
        marker_xy[step] = np.stack([x[marker_ids], y[marker_ids]], axis=-1)

    return {
        "s": s,
        "xs": xs,
        "ys": ys,
        "thetas": thetas,
        "marker_ids": marker_ids,
        "marker_xy": marker_xy,
    }


def save_gif(traj: dict[str, np.ndarray], p: SimParams, out_path: Path) -> None:
    xs, ys = traj["xs"], traj["ys"]
    marker_ids = traj["marker_ids"]
    marker_xy = traj["marker_xy"]

    _set_slide_style(p.use_japanese_labels)
    fig, ax = plt.subplots(figsize=p.figsize, constrained_layout=True)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(False)
    if p.use_japanese_labels:
        ax.set_title("先端\u2192根元へ曲げが伝播するイメージ")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
    else:
        ax.set_title("Signed bend propagation (tip \u2192 base)")
        ax.set_xlabel("x")
        ax.set_ylabel("y")

    all_x = xs.reshape(-1)
    all_y = ys.reshape(-1)
    pad = 0.3
    # Set fixed view to show entry from off-screen
    # Initial position (step 0) should be partially off-screen on the left
    x_start = float(xs[0].min())
    x_end = float(xs[-1].max())
    y_min = float(all_y.min() - pad)
    y_max = float(all_y.max() + pad)
    
    # Extend left side to show initial off-screen position
    x_min = x_start - 0.3
    x_max = x_end + 0.2
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)

    (body_line,) = ax.plot([], [], color="#2ca02c", label=("マニピュレータ" if p.use_japanese_labels else "Manipulator"))
    (base_dot,) = ax.plot([], [], marker="o", color="#111111", ms=6, label=("根元" if p.use_japanese_labels else "Base"))

    palette = ["#111111", "#9467bd", "#ff7f0e", "#1f77b4", "#8c564b"]
    marker_colors = palette[: len(marker_ids)]
    marker_dots = []
    marker_trails = []
    for c in marker_colors:
        (dot,) = ax.plot([], [], marker="o", color=c, ms=5)
        (trail,) = ax.plot([], [], lw=2.0, color=c, alpha=0.22)
        marker_dots.append(dot)
        marker_trails.append(trail)

    # Annotation: propagation direction
    if p.use_japanese_labels:
        ax.annotate(
            "伝播方向",
            xy=(0.78, 0.08),
            xytext=(0.62, 0.08),
            xycoords="axes fraction",
            textcoords="axes fraction",
            arrowprops={"arrowstyle": "->", "color": "#444444", "lw": 2.0},
            ha="left",
            va="center",
            fontsize=11,
            color="#444444",
        )
    else:
        ax.annotate(
            "Propagation",
            xy=(0.78, 0.08),
            xytext=(0.62, 0.08),
            xycoords="axes fraction",
            textcoords="axes fraction",
            arrowprops={"arrowstyle": "->", "color": "#444444", "lw": 2.0},
            ha="left",
            va="center",
            fontsize=11,
            color="#444444",
        )

    ax.legend(loc="upper left", frameon=True, framealpha=0.9, edgecolor="#dddddd")

    def init():
        body_line.set_data([], [])
        for dot, trail in zip(marker_dots, marker_trails, strict=True):
            dot.set_data([], [])
            trail.set_data([], [])
        artists = [body_line, base_dot]
        artists.extend(marker_dots)
        artists.extend(marker_trails)
        return tuple(artists)

    def update(i: int):
        body_line.set_data(xs[i], ys[i])
        # Update base position (it moves forward)
        base_dot.set_data([xs[i, 0]], [ys[i, 0]])

        trail_start = max(0, i + 1 - p.trail_len)
        for k in range(len(marker_ids)):
            marker_dots[k].set_data([marker_xy[i, k, 0]], [marker_xy[i, k, 1]])
            marker_trails[k].set_data(
                marker_xy[trail_start : i + 1, k, 0],
                marker_xy[trail_start : i + 1, k, 1],
            )

        artists = [body_line, base_dot]
        artists.extend(marker_dots)
        artists.extend(marker_trails)
        return tuple(artists)

    ani = FuncAnimation(
        fig,
        update,
        frames=range(p.n_steps),
        init_func=init,
        interval=1000.0 / p.fps,
        blit=True,
    )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    ani.save(out_path, writer=PillowWriter(fps=p.fps), dpi=p.dpi)

    # Save a keyframe PNG for slides (static insert)
    if p.save_keyframe_png:
        idx = p.keyframe_index
        if idx <= 0 or idx >= p.n_steps:
            idx = p.n_steps // 2
        body_line.set_data(xs[idx], ys[idx])
        trail_start = max(0, idx + 1 - p.trail_len)
        for k in range(len(marker_ids)):
            marker_dots[k].set_data([marker_xy[idx, k, 0]], [marker_xy[idx, k, 1]])
            marker_trails[k].set_data(
                marker_xy[trail_start : idx + 1, k, 0],
                marker_xy[trail_start : idx + 1, k, 1],
            )
        png_path = out_path.with_suffix(".png")
        fig.savefig(png_path, dpi=p.dpi)
        print(f"Keyframe PNG saved: {png_path}")
    plt.close(fig)


def display_saved_gif(gif_path: Path) -> None:
    """Display the saved GIF.

    Preference order:
    1) Jupyter/VS Code notebook: inline via IPython.display
    2) Desktop environment: open via xdg-open
    3) Fallback: play frames in a Matplotlib window (if possible)
    """

    try:
        from IPython import get_ipython  # type: ignore
        from IPython.display import Image as IPyImage  # type: ignore
        from IPython.display import display  # type: ignore

        if get_ipython() is not None:
            display(IPyImage(filename=str(gif_path)))
            return
    except Exception:
        pass

    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        try:
            subprocess.Popen(
                ["xdg-open", str(gif_path)],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            return
        except Exception:
            pass

    im = Image.open(gif_path)
    frames: list[np.ndarray] = []
    try:
        while True:
            frames.append(np.array(im.convert("RGBA")))
            im.seek(im.tell() + 1)
    except EOFError:
        pass

    if not frames:
        raise RuntimeError("GIF のフレーム読み込みに失敗しました")

    fig, ax = plt.subplots(figsize=(6.0, 4.5))
    ax.set_axis_off()
    ax.set_title(f"Saved GIF preview: {gif_path.name}")
    img = ax.imshow(frames[0])

    def update(i: int):
        img.set_data(frames[i])
        return (img,)

    _ = FuncAnimation(fig, update, frames=len(frames), interval=50, blit=True)
    plt.show()


def main() -> None:
    p = SimParams()
    traj = simulate(p)

    out_path = Path(__file__).resolve().parent / p.gif_name
    save_gif(traj, p, out_path)

    print(f"GIF saved: {out_path}")
    display_saved_gif(out_path)


if __name__ == "__main__":
    main()
