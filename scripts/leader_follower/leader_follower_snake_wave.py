#!/usr/bin/env python3
"""蛇みたいな波状の動きをするリーダーフォロワー可視化."""

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
    from PIL import Image
except Exception as exc:
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
    n_steps: int = 80

    # Leader input: tip bend angle (signed)
    # 蛇みたいな波状にするため、高周波数・小振幅
    theta_amp: float = 0.6  # [rad] 先端の曲率を強くする
    theta_w: float = 2.0 * math.pi * 0.45  # 高周波数で細かい波を作る

    # Propagation (tip -> base)
    propagation_speed: float = 0.5  # 遅く -> 中腹が曲がった状態を保つ
    time_smoothing: float = 0.25  # より反応的に
    space_smoothing: float = 0.0  # より小さく -> 各セクションの曲率が大きくなる

    # Forward motion (entering from off-screen)
    forward_speed: float = 0.4  # 前進速度

    # Obstacle
    obstacle_enabled: bool = True  # 障害物のオンオフ
    obstacle_x: float = 0.9  # 障害物のx座標（より先に配置）
    obstacle_y: float = 0.0  # 障害物のy座標（少し下にズラして回避可能に）
    obstacle_radius: float = 0.14  # 障害物の半径

    # Target (garbage)
    target_x: float = 1.2  # ゴミのx座標（障害物の先）
    target_y: float = 0.0  # ゴミのy座標
    target_radius: float = 0.07  # ゴミの半径

    # Rendering
    fps: int = 20
    gif_name: str = "leader_follower_snake_wave.gif"
    dpi: int = 180
    trail_len: int = 140

    # Slide-friendly visuals
    figsize: tuple[float, float] = (5.5, 4.5)
    use_japanese_labels: bool = True
    save_keyframe_png: bool = True
    keyframe_index: int = 0
    show_numbers: bool = False


def _set_slide_style(use_japanese: bool) -> None:
    """Apply slide-friendly matplotlib defaults."""
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


def theta_to_backbone(theta: np.ndarray, p: SimParams, custom_n: int = None) -> tuple[np.ndarray, np.ndarray]:
    """Integrate tangent angle into a planar backbone curve."""
    n = custom_n if custom_n is not None else p.n_points
    ds = p.length / (p.n_points - 1)  # Always use original spacing
    x = np.zeros(n, dtype=float)
    y = np.zeros(n, dtype=float)
    if len(theta) > 0:
        x[1:] = np.cumsum(np.cos(theta[:-1]) if len(theta) > 1 else []) * ds
        y[1:] = np.cumsum(np.sin(theta[:-1]) if len(theta) > 1 else []) * ds
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
    fwd_dir = np.array([1.0, 0.0], dtype=float)

    # Markers
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

    collision_step = None  # 衝突したステップを記録
    reached_target = False  # ゴミに到達したか

    for step in range(p.n_steps):
        # Delayed tip angle for each cross-section
        th_delayed = _delayed_from_history(th_hist, step, s, p)

        # Use delayed angle directly (equal magnitude throughout)
        theta_raw = th_delayed

        # Time smoothing
        a_t = float(np.clip(p.time_smoothing, 0.0, 1.0))
        theta_sm = (1.0 - a_t) * theta_prev + a_t * theta_raw

        # Space smoothing - more aggressive for snake-like smoothness
        a_s = float(np.clip(p.space_smoothing, 0.0, 1.0))
        theta_new = theta_sm.copy()
        if a_s > 0.0:
            # Multiple smoothing iterations for smoother curves
            for _ in range(2):
                lap = np.zeros_like(theta_new)
                lap[1:-1] = theta_new[0:-2] - 2.0 * theta_new[1:-1] + theta_new[2:]
                lap[0] = theta_new[1] - theta_new[0]
                lap[-1] = theta_new[-2] - theta_new[-1]
                theta_new = theta_new + a_s * lap

        theta_prev = theta_new
        x, y = theta_to_backbone(theta_new, p)

        # Apply forward motion offset
        t_current = (step - 40) * p.dt
        offset = p.forward_speed * t_current * fwd_dir
        x = x + offset[0]
        y = y + offset[1]

        # 衝突判定
        if collision_step is None and p.obstacle_enabled:
            for i in range(n):
                dist = math.sqrt((x[i] - p.obstacle_x)**2 + (y[i] - p.obstacle_y)**2)
                if dist < p.obstacle_radius:
                    collision_step = step
                    break

        # ゴミへの到達判定（先端が届いたか）
        if not reached_target:
            tip_dist = math.sqrt((x[-1] - p.target_x)**2 + (y[-1] - p.target_y)**2)
            if tip_dist < p.target_radius:
                reached_target = True

        thetas[step] = theta_new
        xs[step] = x
        ys[step] = y
        marker_xy[step] = np.stack([x[marker_ids], y[marker_ids]], axis=-1)

        # 衝突またはゴミ到達で停止
        if collision_step is not None or reached_target:
            break

    # 衝突またはゴミ到達後、最後のフレームを残りに複製
    if collision_step is not None:
        for step in range(collision_step + 1, p.n_steps):
            xs[step] = xs[collision_step]
            ys[step] = ys[collision_step]
            thetas[step] = thetas[collision_step]
            marker_xy[step] = marker_xy[collision_step]
    elif reached_target:
        # ゴミに到達したステップを見つける
        reached_step = None
        for step in range(p.n_steps):
            if np.any(xs[step]):
                tip_dist = math.sqrt((xs[step, -1] - p.target_x)**2 + (ys[step, -1] - p.target_y)**2)
                if tip_dist < p.target_radius:
                    reached_step = step
                    break
        if reached_step is not None:
            for step in range(reached_step + 1, p.n_steps):
                xs[step] = xs[reached_step]
                ys[step] = ys[reached_step]
                thetas[step] = thetas[reached_step]
                marker_xy[step] = marker_xy[reached_step]

    return {
        "s": s,
        "xs": xs,
        "ys": ys,
        "thetas": thetas,
        "marker_ids": marker_ids,
        "marker_xy": marker_xy,
        "collision_step": collision_step,
        "reached_target": reached_target,
    }


def save_gif(traj: dict[str, np.ndarray], p: SimParams, out_path: Path) -> None:
    xs, ys = traj["xs"], traj["ys"]
    marker_ids = traj["marker_ids"]
    marker_xy = traj["marker_xy"]
    reached_target = traj["reached_target"]

    _set_slide_style(p.use_japanese_labels)
    fig, ax = plt.subplots(figsize=p.figsize, constrained_layout=True)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(False)
    ax.axis("off")  # グラフの軸を非表示

    # Use only later steps to avoid including the root when off-screen
    later_steps = xs[30:, :]
    x_start = float(later_steps.min())
    x_end = float(later_steps.max())
    y_min = float(later_steps.min() - 0.3)
    y_max = float(ys[30:, :].max() + 0.3)

    x_min = x_start + 0.25
    x_max = x_end + 1.0
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)

    (body_line,) = ax.plot([], [], color="#2ca02c", lw=8, label=("マニピュレータ" if p.use_japanese_labels else "Manipulator"))
    (base_dot,) = ax.plot([], [], marker="o", color="#111111", ms=6, label=("根元" if p.use_japanese_labels else "Base"))

    palette = ["#111111", "#9467bd", "#ff7f0e", "#2c4ba0", "#8c564b"]  # 先端を緑色に変更
    marker_colors = palette[: len(marker_ids)]
    marker_dots = []
    marker_trails = []
    for c in marker_colors:
        (dot,) = ax.plot([], [], marker="o", color=c, ms=9.5)
        (trail,) = ax.plot([], [], lw=5.0, color=c, alpha=0.12)
        marker_dots.append(dot)
        marker_trails.append(trail)

    # Annotation
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
    
    # Tip annotation (動的に更新)
    tip_text = ax.text(0, 0, "先端", ha="center", va="bottom",
                       fontsize=9, bbox=dict(boxstyle="round,pad=0.3", facecolor="#2c4ba0", alpha=0.8, edgecolor="none"),
                       color="white", fontweight="bold", zorder=10)

    ax.legend(loc="upper left", frameon=True, framealpha=0.9, edgecolor="#dddddd")

    # Draw obstacle
    if p.obstacle_enabled:
        obstacle_circle = plt.Circle(
            (p.obstacle_x, p.obstacle_y),
            p.obstacle_radius,
            color="#ff0000",
            alpha=0.6,
            zorder=1
        )
        ax.add_patch(obstacle_circle)
        
        # Add text inside obstacle
        obstacle_text = ax.text(
            p.obstacle_x, p.obstacle_y,
            "障害物" if p.use_japanese_labels else "Obstacle",
            ha="center", va="center",
            fontsize=7,
            color="white",
            fontweight="bold",
            zorder=2
        )

    # Draw target (garbage)
    target_circle = plt.Circle(
        (p.target_x, p.target_y),
        p.target_radius,
        color="#ffa500" if not reached_target else "#00ff00",
        alpha=0.7,
        zorder=1
    )
    ax.add_patch(target_circle)
    
    # Add text inside target
    target_text = ax.text(
        p.target_x, p.target_y,
        "ゴミ" if p.use_japanese_labels else "Target",
        ha="center", va="center",
        fontsize=6,
        color="white",
        fontweight="bold",
        zorder=2
    )

    def init():
        body_line.set_data([], [])
        tip_text.set_visible(False)
        for dot, trail in zip(marker_dots, marker_trails, strict=True):
            dot.set_data([], [])
            trail.set_data([], [])
        artists = [body_line, base_dot, tip_text]
        artists.extend(marker_dots)
        artists.extend(marker_trails)
        return tuple(artists)

    def update(i: int):
        body_line.set_data(xs[i], ys[i])
        base_dot.set_data([xs[i, 0]], [ys[i, 0]])

        # Update tip text position to follow the tip
        tip_x, tip_y = xs[i, -1], ys[i, -1]
        tip_text.set_position((tip_x, tip_y + 0.08))  # 先端の少し上に配置
        tip_text.set_visible(True)

        # Check if target is reached at this frame
        tip_dist = math.sqrt((xs[i, -1] - p.target_x)**2 + (ys[i, -1] - p.target_y)**2)
        if tip_dist < p.target_radius:
            target_circle.set_color("#00ff00")  # 到達時は緊色
        
        trail_start = max(0, i + 1 - p.trail_len)
        for k in range(len(marker_ids)):
            marker_dots[k].set_data([marker_xy[i, k, 0]], [marker_xy[i, k, 1]])
            marker_trails[k].set_data(
                marker_xy[trail_start : i + 1, k, 0],
                marker_xy[trail_start : i + 1, k, 1],
            )

        artists = [body_line, base_dot, tip_text]
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

    # Save keyframe PNG
    if p.save_keyframe_png:
        idx = p.keyframe_index
        if idx <= 0 or idx >= p.n_steps:
            idx = p.n_steps // 2
        body_line.set_data(xs[idx], ys[idx])
        tip_text.set_position((xs[idx, -1], ys[idx, -1] + 0.08))
        tip_text.set_visible(True)
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
    """Display the saved GIF."""

    try:
        from IPython import get_ipython
        from IPython.display import Image as IPyImage
        from IPython.display import display

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
