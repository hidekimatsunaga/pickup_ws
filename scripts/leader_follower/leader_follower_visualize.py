#!/usr/bin/env python3
"""Leader-follower visualization (tip-to-base propagation) for a toy continuum manipulator.

ユーザ定義の「リーダーフォロワー」:
- 先端（リーダ）が描く軌跡（目標）に追従する
- その動きが、マニピュレータ途中〜根元側（後続）へ徐々に伝播する

実装:
- 2Dの点列（inextensible chain）で連続体を近似
- 先端は目標点へ引っ張る（簡易制御）
- 隣接点間の距離拘束を Position Based Dynamics(PBD) で少ない反復回数だけ解く
    → 「拘束が全身に一瞬で伝わらない」ため、途中までの伝播っぽい見え方になる

GIFを保存し、その後に同じスクリプト内で表示まで行います。
（Notebookならインライン表示、GUIなら xdg-open、ダメなら Matplotlib で再生）
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

try:
    from PIL import Image  # pillow
except Exception as exc:  # pragma: no cover
    raise SystemExit(
        "Pillow が必要です。`pip install pillow matplotlib numpy` を実行してください。"
    ) from exc


@dataclass(frozen=True)
class SimParams:
    # Geometry
    n_points: int = 55  # points along the backbone including base and tip
    length: float = 1.0

    # Time
    dt: float = 0.03
    n_steps: int = 260

    # Leader (tip) target trajectory: circular arc around the base
    tip_radius: float = 0.92  # should be <= length
    tip_theta_amp: float = 0.55  # [rad]
    tip_theta_w: float = 2.0 * math.pi * 0.22

    # Tip control strength (larger -> tip sticks closer to target)
    tip_gain: float = 16.0

    # Propagation feel (smaller -> motion propagates only partway each step)
    constraint_iters: int = 4
    damping: float = 0.06

    # Rendering / GIF
    fps: int = 20
    gif_name: str = "leader_follower.gif"
    dpi: int = 120
    trail_len: int = 120


def leader_tip_target(t: float, p: SimParams) -> tuple[float, float]:
    """Leader desired tip trajectory.

    Tip follows a circular arc centered at the base (0,0):
    x = r cos(theta), y = r sin(theta)
    """
    r = min(p.tip_radius, p.length)
    theta = p.tip_theta_amp * math.sin(p.tip_theta_w * t)
    x = r * math.cos(theta)
    y = r * math.sin(theta)
    return x, y


def _apply_distance_constraint(
    x: np.ndarray,
    i: int,
    j: int,
    rest_len: float,
    inv_mass: np.ndarray,
) -> None:
    """PBD distance constraint between points i and j."""
    xi = x[i]
    xj = x[j]
    delta = xj - xi
    dist = float(np.linalg.norm(delta))
    if dist < 1e-12:
        return
    w_i = float(inv_mass[i])
    w_j = float(inv_mass[j])
    w_sum = w_i + w_j
    if w_sum <= 0.0:
        return
    # Positive C means too long; push together. Negative means too short; pull apart.
    C = dist - rest_len
    corr = (C / dist) * delta
    x[i] += (w_i / w_sum) * corr
    x[j] -= (w_j / w_sum) * corr


def simulate(p: SimParams) -> dict[str, np.ndarray]:
    """Run simulation and return trajectories.

    Single manipulator. The tip is controlled to follow the target trajectory.
    The rest of the body follows through distance constraints solved with limited iterations,
    producing a visually intuitive "propagation" from tip to base.
    """

    n = p.n_points
    ds = p.length / (n - 1)

    # Positions and velocities
    x = np.zeros((n, 2), dtype=float)
    x[:, 0] = np.linspace(0.0, p.length, n)
    v = np.zeros_like(x)

    inv_mass = np.ones(n, dtype=float)
    inv_mass[0] = 0.0  # base fixed

    xs = np.zeros((p.n_steps, n), dtype=float)
    ys = np.zeros((p.n_steps, n), dtype=float)
    tip_target = np.zeros((p.n_steps, 2), dtype=float)
    tip_pos = np.zeros((p.n_steps, 2), dtype=float)

    # Track a few body points to show delayed following
    # Markers to visualize how motion propagates from tip toward base.
    # 4 markers => 3 "sections" along the body (base .. mid1 .. mid2 .. tip)
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
        t = step * p.dt
        tip_target[step] = np.array(leader_tip_target(t, p), dtype=float)

        # Semi-implicit integration (no external forces besides damping)
        v *= (1.0 - p.damping)
        x_pred = x + p.dt * v

        # Tip control: pull tip toward target (soft constraint)
        x_pred[-1] += (p.tip_gain * p.dt) * (tip_target[step] - x_pred[-1])

        # Enforce base fixed
        x_pred[0] = np.array([0.0, 0.0], dtype=float)

        # PBD constraints (limited iterations -> propagation-like effect)
        for _ in range(max(1, p.constraint_iters)):
            x_pred[0] = np.array([0.0, 0.0], dtype=float)
            for i in range(n - 1):
                _apply_distance_constraint(x_pred, i, i + 1, ds, inv_mass)
            # keep pulling the tip a bit each iteration so it doesn't drift
            x_pred[-1] += (0.35 * p.tip_gain * p.dt) * (tip_target[step] - x_pred[-1])

        # Update velocities and commit
        v = (x_pred - x) / p.dt
        x = x_pred

        xs[step] = x[:, 0]
        ys[step] = x[:, 1]
        tip_pos[step] = x[-1]
        marker_xy[step] = x[marker_ids]

    return {
        "xs": xs,
        "ys": ys,
        "tip_target": tip_target,
        "tip_pos": tip_pos,
        "marker_ids": marker_ids,
        "marker_xy": marker_xy,
        "ds": np.array([ds], dtype=float),
    }


def save_gif(traj: dict[str, np.ndarray], p: SimParams, out_path: Path) -> None:
    """Render and save a GIF."""
    xs, ys = traj["xs"], traj["ys"]
    tip_target = traj["tip_target"]
    tip_pos = traj["tip_pos"]
    marker_ids = traj["marker_ids"]
    marker_xy = traj["marker_xy"]

    fig, ax = plt.subplots(figsize=(6.0, 4.5))
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.set_title("Tip-to-base propagation (single continuum manipulator)")

    # Plot bounds (fixed for stable animation)
    all_x = np.concatenate([xs.reshape(-1), tip_target[:, 0]])
    all_y = np.concatenate([ys.reshape(-1), tip_target[:, 1]])
    pad = 0.15
    ax.set_xlim(float(all_x.min() - pad), float(all_x.max() + pad))
    ax.set_ylim(float(all_y.min() - pad), float(all_y.max() + pad))

    (body_line,) = ax.plot([], [], lw=2.8, color="#2ca02c", label="Manipulator")
    (base_dot,) = ax.plot([0.0], [0.0], marker="o", color="black", ms=5)
    (tip_path_line,) = ax.plot([], [], lw=1.6, color="#1f77b4", alpha=0.35, label="Tip path")
    (tip_dot,) = ax.plot([], [], marker="o", color="#1f77b4", ms=5, label="Tip")
    (tip_target_x,) = ax.plot([], [], marker="x", color="#1f77b4", ms=6, mew=2, label="Tip target")

    palette = ["#000000", "#9467bd", "#ff7f0e", "#1f77b4", "#8c564b"]
    marker_colors = palette[: len(marker_ids)]
    marker_dots = []
    marker_trails = []
    for c in marker_colors:
        (dot,) = ax.plot([], [], marker="o", color=c, ms=4)
        (trail,) = ax.plot([], [], lw=1.2, color=c, alpha=0.25)
        marker_dots.append(dot)
        marker_trails.append(trail)

    time_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top")

    ax.legend(loc="lower right")

    def init():
        body_line.set_data([], [])
        tip_path_line.set_data([], [])
        tip_dot.set_data([], [])
        tip_target_x.set_data([], [])
        for dot, trail in zip(marker_dots, marker_trails, strict=True):
            dot.set_data([], [])
            trail.set_data([], [])
        time_text.set_text("")
        artists = [body_line, tip_path_line, tip_dot, tip_target_x, base_dot, time_text]
        artists.extend(marker_dots)
        artists.extend(marker_trails)
        return tuple(artists)

    def update(i: int):
        body_line.set_data(xs[i], ys[i])

        tip_path_line.set_data(tip_pos[: i + 1, 0], tip_pos[: i + 1, 1])
        tip_dot.set_data([tip_pos[i, 0]], [tip_pos[i, 1]])
        tip_target_x.set_data([tip_target[i, 0]], [tip_target[i, 1]])

        # Trails for a few internal points + tip
        trail_start = max(0, i + 1 - p.trail_len)
        for k in range(len(marker_ids)):
            marker_dots[k].set_data([marker_xy[i, k, 0]], [marker_xy[i, k, 1]])
            marker_trails[k].set_data(
                marker_xy[trail_start : i + 1, k, 0],
                marker_xy[trail_start : i + 1, k, 1],
            )

        tip_e = float(np.linalg.norm(tip_pos[i] - tip_target[i]))
        time_text.set_text(
            f"t={i * p.dt:5.2f} s   tip error={tip_e:.3f}   constraint iters={p.constraint_iters}"
        )

        artists = [body_line, tip_path_line, tip_dot, tip_target_x, base_dot, time_text]
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
    plt.close(fig)


def display_saved_gif(gif_path: Path) -> None:
    """Display the saved GIF.

    Preference order:
    1) Jupyter/VS Code notebook: inline via IPython.display
    2) Desktop environment: open via xdg-open
    3) Fallback: play frames in a Matplotlib window (if possible)
    """

    # 1) Inline display when running in IPython/Jupyter
    try:
        from IPython import get_ipython  # type: ignore
        from IPython.display import Image as IPyImage  # type: ignore
        from IPython.display import display  # type: ignore

        if get_ipython() is not None:
            display(IPyImage(filename=str(gif_path)))
            return
    except Exception:
        pass

    # 2) Try opening via OS viewer (works on typical Linux desktops)
    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        try:
            subprocess.Popen(["xdg-open", str(gif_path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return
        except Exception:
            pass

    # 3) Fallback: Matplotlib playback (may not show on headless)
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
