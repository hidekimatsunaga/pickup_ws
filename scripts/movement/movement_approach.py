"""Simple 2D simulation: approaching a fixed target with two controllers.

This script demonstrates two behaviors:
 - fast_approach: go straight to the target with a constant forward speed
 - cautious_approach: when within `switch_radius`, slow down and use proportional
   control to smoothly stop at `stop_radius` while keeping the target in front.

Outputs:
 - approach_trajectory.png : trajectory plot
 - approach_range.png      : range vs time plot

Designed to be runnable standalone.
"""
import math
import numpy as np
import matplotlib.pyplot as plt

def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

def angle_world(from_xy, to_xy):
    return math.atan2(to_xy[1] - from_xy[1], to_xy[0] - from_xy[0])

def rel_in_robot_frame(px, py, th, target_world):
    dx = target_world[0] - px
    dy = target_world[1] - py
    c = math.cos(th)
    s = math.sin(th)
    bx =  c*dx + s*dy
    by = -s*dx + c*dy
    return bx, by

def sim_approach(control_mode='cautious', dt=0.02, T=8.0):
    """Simulate approach to a fixed target at origin.

    control_mode: 'fast', 'cautious', or 'diff_like' (vy=0)
    Returns: traj, logs where traj shape = (N,3): x,y,th and logs columns: t, range
    """
    N = int(T / dt)
    # fixed target
    target = np.array([0.0, 0.0])

    # initial pose (world)
    x, y = 1.6, -0.8
    th = angle_world((x,y), target)  # face target initially

    traj = np.zeros((N, 3))
    logs = np.zeros((N, 2))  # t, range

    # controller parameters
    v_fast = 0.45            # m/s forward for fast approach
    switch_radius = 0.7     # when to switch into cautious mode
    stop_radius = 0.18      # desired final distance to target
    k_v = 1.6               # proportional speed gain for cautious mode
    k_yaw = 2.0             # yaw gain to face target
    wz_max = 2.0

    # special constraint: when within this distance from the target,
    # the robot is only allowed to approach by moving straight parallel to x-axis
    axis_constrain_distance = 1.0  # [m]

    # diff_like parameters (diff-drive style: rotate-then-go)
    alpha_align = math.radians(6)  # [rad] alignment threshold before moving forward
    go_time_max = 0.9  # [s] max forward drive burst duration

    diff_state = "ALIGN"  # for diff_like mode
    go_timer = 0.0

    for k in range(N):
        t = k * dt
        # current relative in robot frame
        tbx, tby = rel_in_robot_frame(x, y, th, target)
        r = math.hypot(tbx, tby)
        bearing = math.atan2(tby, tbx)

        # Nominal controls
        if control_mode == 'fast':
            # fast approach with vy=0 constraint (diff-drive like)
            # simply drive forward at constant speed while pointing to target
            v = v_fast
            wz = max(-wz_max, min(wz_max, k_yaw * bearing))
            # enforce vy=0: integrate pure unicycle kinematics
            if r < 0.01:
                v = 0.0
                wz = 0.0
            x += v * math.cos(th) * dt
            y += v * math.sin(th) * dt
            th = wrap_pi(th + wz * dt)
        elif control_mode == 'diff_like':
            # diff-drive-like: ALIGN (rotate) -> GO (forward) -> repeat
            # no lateral motion allowed (vy=0)
            if r < 0.01:
                # very close, stop
                v = 0.0
                wz = 0.0
            else:
                if diff_state == "ALIGN":
                    v = 0.0
                    wz = max(-wz_max, min(wz_max, k_yaw * bearing))
                    if abs(bearing) < alpha_align:
                        diff_state = "GO"
                        go_timer = 0.0
                elif diff_state == "GO":
                    # move forward while keeping bearing aligned
                    wz = max(-wz_max, min(wz_max, k_yaw * bearing))
                    if r > switch_radius:
                        v = v_fast
                    else:
                        v = k_v * (r - stop_radius)
                        v = max(0.0, min(v, v_fast * 0.9))
                    go_timer += dt

                    # if misaligned or burst ends, re-align
                    if go_timer >= go_time_max or abs(bearing) > 2.5 * alpha_align:
                        diff_state = "ALIGN"
        else:  # cautious
            # if far, move faster; when within switch_radius, slowly reduce speed
            if r > switch_radius:
                v = v_fast
            else:
                # proportional to error from stop radius
                v = k_v * (r - stop_radius)
                # clamp to non-negative and max
                v = max(0.0, min(v, v_fast * 0.9))

            # always try to face the target while approaching
            wz = max(-wz_max, min(wz_max, k_yaw * bearing))

        # if we're very close and commanded speed is tiny, stop
        if r < 0.01:
            v = 0.0
            wz = 0.0

        # integrate kinematics based on mode
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th = wrap_pi(th + wz * dt)

        traj[k] = [x, y, th]
        logs[k] = [t, r]

    return traj, logs

def make_plots(traj_fast, logs_fast, traj_cautious, logs_cautious, traj_diff=None, logs_diff=None, out_prefix='approach'):
    # trajectory
    plt.figure(figsize=(6,6))
    plt.plot(traj_fast[:,0], traj_fast[:,1], label='fast_approach', color='C0')
    plt.plot(traj_cautious[:,0], traj_cautious[:,1], label='cautious_approach', color='C1')
    if traj_diff is not None:
        plt.plot(traj_diff[:,0], traj_diff[:,1], label='diff_like_approach', color='C2')
    plt.scatter([0.0],[0.0], marker='x', color='k', label='target')
    plt.axis('equal')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.title('Approach trajectories')
    plt.legend()
    traj_path = f'{out_prefix}_trajectory.png'
    plt.savefig(traj_path, dpi=150)
    plt.close()

    # range vs time
    plt.figure()
    plt.plot(logs_fast[:,0], logs_fast[:,1], label='fast_approach', color='C0')
    plt.plot(logs_cautious[:,0], logs_cautious[:,1], label='cautious_approach', color='C1')
    if logs_diff is not None:
        plt.plot(logs_diff[:,0], logs_diff[:,1], label='diff_like_approach', color='C2')
    plt.axhline(0.18, linestyle='--', color='gray', label='stop_radius')
    plt.xlabel('t [s]')
    plt.ylabel('range [m]')
    plt.title('Range to target')
    plt.legend()
    range_path = f'{out_prefix}_range.png'
    plt.savefig(range_path, dpi=150)
    plt.close()

    print(f'Saved: {traj_path}, {range_path}')


def make_gif(traj_fast, traj_cautious, traj_diff=None, out_path='approach.gif', step=3, dt=0.02):
    """Create a simple GIF animation showing all robot approaches as arrows moving."""
    from matplotlib.animation import FuncAnimation, PillowWriter
    import matplotlib.patches as patches

    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    # static elements
    ax.scatter([0.0], [0.0], marker='x', color='k', label='target', s=100, zorder=5)
    ax.plot(traj_fast[:,0], traj_fast[:,1], color='C0', alpha=0.25, linewidth=2, label='fast (vy=0)')
    ax.plot(traj_cautious[:,0], traj_cautious[:,1], color='C1', alpha=0.25, linewidth=2, label='cautious (vy≠0)')
    if traj_diff is not None:
        ax.plot(traj_diff[:,0], traj_diff[:,1], color='C2', alpha=0.25, linewidth=2, label='diff_like (vy=0)')

    pad = 0.6
    xs = [traj_fast[:,0], traj_cautious[:,0], [0.0]]
    ys = [traj_fast[:,1], traj_cautious[:,1], [0.0]]
    if traj_diff is not None:
        xs.append(traj_diff[:,0])
        ys.append(traj_diff[:,1])
    xs = np.concatenate(xs)
    ys = np.concatenate(ys)
    ax.set_xlim(xs.min() - pad, xs.max() + pad)
    ax.set_ylim(ys.min() - pad, ys.max() + pad)
    ax.legend(loc='upper right')

    # dynamic artists: use small triangle for robot heading
    def draw_tri(x, y, th, size=0.08, color='C0'):
        c = math.cos(th)
        s = math.sin(th)
        pts = np.array([[ size, 0.0], [-size*0.6,  size*0.4], [-size*0.6, -size*0.4]])
        Rm = np.array([[c, -s], [s, c]])
        wpts = pts @ Rm.T + np.array([x, y])
        return patches.Polygon(wpts, closed=True, fill=True, color=color, alpha=0.9)

    # initial artists stored in a list so we can remove them each frame
    artists = []
    artists.append(ax.add_patch(draw_tri(traj_fast[0,0], traj_fast[0,1], traj_fast[0,2], color='C0')))
    artists.append(ax.add_patch(draw_tri(traj_cautious[0,0], traj_cautious[0,1], traj_cautious[0,2], color='C1')))
    if traj_diff is not None:
        artists.append(ax.add_patch(draw_tri(traj_diff[0,0], traj_diff[0,1], traj_diff[0,2], color='C2')))
    time_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, va='top', fontsize=12, fontweight='bold')

    min_len = min(len(traj_fast), len(traj_cautious))
    if traj_diff is not None:
        min_len = min(min_len, len(traj_diff))
    frames = list(range(0, min_len, step))

    def update(i):
        k = frames[i]
        # remove previous dynamic artists
        for a in artists:
            try:
                a.remove()
            except Exception:
                pass
        artists.clear()

        xA, yA, thA = traj_fast[k]
        xB, yB, thB = traj_cautious[k]
        artists.append(ax.add_patch(draw_tri(xA, yA, thA, color='C0')))
        artists.append(ax.add_patch(draw_tri(xB, yB, thB, color='C1')))
        if traj_diff is not None:
            xC, yC, thC = traj_diff[k]
            artists.append(ax.add_patch(draw_tri(xC, yC, thC, color='C2')))
        time_text.set_text(f't = {k*dt:5.2f} s')
        return artists + [time_text]

    anim = FuncAnimation(fig, update, frames=len(frames), interval=dt * step * 1000, blit=False)
    try:
        anim.save(out_path, writer=PillowWriter(fps=max(1, int(1 / (dt * step)))))
        print(f'Saved GIF: {out_path}')
    except Exception as e:
        print(f'[WARN] GIF generation failed: {e}')
    plt.close(fig)

if __name__ == '__main__':
    traj_f, logs_f = sim_approach(control_mode='fast', T=8.0)
    traj_c, logs_c = sim_approach(control_mode='cautious', T=8.0)
    traj_d, logs_d = sim_approach(control_mode='diff_like', T=8.0)
    make_plots(traj_f, logs_f, traj_c, logs_c, traj_diff=traj_d, logs_diff=logs_d)
    try:
        make_gif(traj_f, traj_c, traj_diff=traj_d, out_path='approach.gif', step=3, dt=0.02)
    except Exception as e:
        print(f'[WARN] make_gif failed: {e}')
