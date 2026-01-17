# fan_orbit_vs_diff_sim.py
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation, PillowWriter

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def wrap_pi(a):
    return (a + math.pi) % (2*math.pi) - math.pi

def angle_world(from_xy, to_xy):
    """Angle of vector (to_xy - from_xy) in world frame."""
    return math.atan2(to_xy[1] - from_xy[1], to_xy[0] - from_xy[0])

# =========================
# Simulation settings
# =========================
dt = 0.02
T = 14.0
N = int(T / dt)

target = np.array([0.0, 0.0])  # fixed target at origin (world)

# initial pose (world)
x0, y0 = 1.2, -0.6
th0 = math.atan2(target[1] - y0, target[0] - x0)  # face target initially

# sector radius: keep constant (use initial radius)
r0 = math.hypot(x0 - target[0], y0 - target[1])
r_goal = r0

# Fan angle around the target (center angle)
# "60deg" here means the central angle of a sector whose vertex is the target.
fan_deg = 60.0
fan = math.radians(fan_deg)

# NOTE: we still keep a robot-centric "camera" FOV metric (optional) derived from fan
# If you don't need it, we can remove it later.
fov_deg = fan_deg
fov = fan

# =========================
# Controller A: holonomic orbit (fan/orbit)
# =========================
A = {
    "radius_des": 0.60,   # [m] (controller internal; goal radius uses r_goal)
    "omega": 0.35,        # [rad/s] CCW orbit speed (tangential)
    "k_r": 1.2,           # radial gain
    "k_yaw": 1.5,         # yaw gain to keep target in front
    "v_rad_max": 0.30,    # [m/s]
    "v_tan_max": 0.35,    # [m/s]
    "wz_max": 1.2,        # [rad/s]
    "min_range": 0.20,    # [m]
}

# =========================
# Controller B: diff-drive-like turn-go-turn (no lateral)
# =========================
# Goal pose: the sector starts from the initial direction (target -> initial robot)
# and ends at +fan_deg along the same radius.
sector_start = angle_world(target, np.array([x0, y0]))
goal_angle = sector_start + fan
goal = np.array([
    target[0] + r_goal * math.cos(goal_angle),
    target[1] + r_goal * math.sin(goal_angle),
])
# goal orientation: face the target at the end pose
goal_th = math.atan2(target[1] - goal[1], target[0] - goal[0])

B = {
    "radius_des": 0.60,                # [m]
    "k_yaw": 2.0,                      # yaw gain
    "wz_max": 1.4,                     # [rad/s]
    "v_max": 0.30,                     # [m/s]
    "alpha_align": math.radians(6),    # [rad] alignment threshold
    "go_time": 0.9,                    # [s] go burst duration
    "min_range": 0.20,                 # [m]
    "k_v": 1.2,                        # [1/s] forward speed gain to goal distance
    "k_goal_yaw": 2.0,                 # yaw gain for aligning to goal_th
    "yaw_align": math.radians(4),      # [rad] final yaw alignment threshold
}

def rel_in_robot_frame(px, py, th, target_world):
    """target in world -> robot frame (bx forward, by left)"""
    dx = target_world[0] - px
    dy = target_world[1] - py
    c = math.cos(th)
    s = math.sin(th)
    bx =  c*dx + s*dy
    by = -s*dx + c*dy
    return bx, by

def sim_holonomic():
    """Holonomic orbit: v = v_rad*ur + v_tan*ut, yaw keeps target in front."""
    x, y, th = x0, y0, th0
    traj = np.zeros((N, 3))
    logs = []  # (t, range_to_target, alpha_robot, in_view_robot)
    lost_sector_count = 0
    lost_count = 0

    for k in range(N):
        # --- Metric A: robot-centric bearing to target (for reference) ---
        tbx, tby = rel_in_robot_frame(x, y, th, target)
        alpha = math.atan2(tby, tbx)
        in_view = abs(alpha) <= fov/2
        if not in_view:
            lost_count += 1

        # --- Metric B: target-centered sector (what you mean by "fan") ---
        # sector center direction is fixed as: target -> initial robot position
        sector_center = angle_world(target, np.array([x0, y0]))
        ang_tr = angle_world(target, np.array([x, y]))
        sector_err = wrap_pi(ang_tr - sector_center)
        in_sector = abs(sector_err) <= fan/2
        if not in_sector:
            lost_sector_count += 1

        # Control objective: reach the common goal pose (position + orientation)
        gbx, gby = rel_in_robot_frame(x, y, th, goal)
        d_goal = math.hypot(gbx, gby)
        d_tgt = math.hypot(tbx, tby)

        # yaw error to goal orientation
        yaw_err = wrap_pi(goal_th - th)

        # if very close to goal position and orientation, stop
        if d_goal < 0.01 and abs(yaw_err) < math.radians(2.0):
            vx_r = vy_r = wz = 0.0
        else:
            # unit vectors in robot frame towards goal
            urx, ury = (gbx/d_goal, gby/d_goal) if d_goal > 1e-6 else (1.0, 0.0)
            # tangential intentionally set to zero to drive to goal (no orbit)
            utx, uty = -ury, urx

            # proportional radial speed to reduce distance to goal
            v_rad = clamp(A["k_r"] * d_goal, -A["v_rad_max"], A["v_rad_max"])
            v_tan = 0.0

            vx_r = v_rad * urx + v_tan * utx
            vy_r = v_rad * ury + v_tan * uty

            # yaw to reach goal_th
            wz = clamp(A["k_yaw"] * yaw_err, -A["wz_max"], A["wz_max"])

        # integrate in world
        c = math.cos(th)
        s = math.sin(th)
        vx_w = c*vx_r - s*vy_r
        vy_w = s*vx_r + c*vy_r
        x += vx_w * dt
        y += vy_w * dt
        th = wrap_pi(th + wz * dt)

        traj[k] = [x, y, th]
        logs.append((k*dt, d_tgt, alpha, in_view))

    return traj, np.array(logs), lost_count, lost_sector_count

def sim_diff_like():
    """Diff-drive-like: ALIGN (rotate in place) -> GO (forward burst) -> ALIGN ..."""
    x, y, th = x0, y0, th0
    traj = np.zeros((N, 3))
    logs = []
    lost_count = 0

    state = "ALIGN"
    go_timer = 0.0

    met_stop = False

    for k in range(N):
        # Tracking metric (camera FOV): target bearing in robot frame
        tbx, tby = rel_in_robot_frame(x, y, th, target)
        alpha = math.atan2(tby, tbx)
        in_view = abs(alpha) <= fov/2
        if not in_view:
            lost_count += 1

        # Control objective: reach the common goal pose
        gbx, gby = rel_in_robot_frame(x, y, th, goal)
        d = math.hypot(gbx, gby)

        # stop if very close to goal and oriented correctly
        yaw_err = wrap_pi(goal_th - th)
        if d < 0.01 and abs(yaw_err) < B["yaw_align"]:
            v = wz = 0.0
            met_stop = True
        else:
            goal_bearing = math.atan2(gby, gbx)

            # If close enough in position, prioritize final yaw alignment
            if d < 0.08:
                state = "FINAL_ALIGN"

            if state == "ALIGN":
                v = 0.0
                wz = clamp(B["k_yaw"] * goal_bearing, -B["wz_max"], B["wz_max"])
                if abs(goal_bearing) < B["alpha_align"]:
                    state = "GO"
                    go_timer = 0.0

            elif state == "GO":
                # move forward toward goal, while keeping heading roughly toward goal position
                wz = clamp(B["k_yaw"] * goal_bearing, -B["wz_max"], B["wz_max"])
                v = clamp(B["k_v"] * d, 0.0, B["v_max"])
                go_timer += dt

                # if misalignment grows or burst ends, re-align
                if go_timer >= B["go_time"] or abs(goal_bearing) > 2.5 * B["alpha_align"]:
                    state = "ALIGN"

            elif state == "FINAL_ALIGN":
                # once position is almost reached, match goal orientation
                v = 0.0
                wz = clamp(B["k_goal_yaw"] * yaw_err, -B["wz_max"], B["wz_max"])
                if abs(yaw_err) < B["yaw_align"]:
                    # let the outer stop condition catch it next loop
                    state = "DONE"

            else:
                v = wz = 0.0

        # integrate diff-drive kinematics (no lateral)
        x += v * math.cos(th) * dt
        y += v * math.sin(th) * dt
        th = wrap_pi(th + wz * dt)

        traj[k] = [x, y, th]
        logs.append((k*dt, d, alpha, in_view))

    return traj, np.array(logs), lost_count, met_stop

# =========================
# Drawing helpers
# =========================
def draw_robot(ax, x, y, th, color="C0", size=0.08):
    """Draw a simple triangular robot footprint and heading arrow."""
    c = math.cos(th)
    s = math.sin(th)
    # triangle in robot frame
    pts = np.array([
        [ size, 0.0],
        [-size*0.6,  size*0.4],
        [-size*0.6, -size*0.4],
    ])
    Rm = np.array([[c, -s], [s, c]])
    wpts = pts @ Rm.T + np.array([x, y])
    poly = patches.Polygon(wpts, closed=True, fill=False, edgecolor=color, linewidth=2)
    ax.add_patch(poly)
    return poly


def draw_camera_fov(ax, x, y, th, fov_rad, color="C0", length=0.6, alpha=0.6):
    """Draw camera FOV rays (two lines) emanating from robot pose."""
    th1 = th - fov_rad / 2
    th2 = th + fov_rad / 2
    l1, = ax.plot([x, x + length * math.cos(th1)], [y, y + length * math.sin(th1)],
                 color=color, linewidth=1.2, alpha=alpha)
    l2, = ax.plot([x, x + length * math.cos(th2)], [y, y + length * math.sin(th2)],
                 color=color, linewidth=1.2, alpha=alpha)
    return l1, l2


def make_animation(trajA, trajB, out_path="movement.gif", step=3):
    """Create a GIF animation showing both robots and their camera FOV."""
    fig, ax = plt.subplots()

    # static elements
    ax.plot(trajA[:, 0], trajA[:, 1], color="C0", alpha=0.25, label="Holonomic path")
    ax.plot(trajB[:, 0], trajB[:, 1], color="C1", alpha=0.25, label="Diff-like path")
    ax.scatter([target[0]], [target[1]], marker="x", color="k", label="Target")
    ax.scatter([goal[0]], [goal[1]], marker="o", s=70, facecolors="none", edgecolors="k", label="Goal")

    # sector visualization
    R = 1.6
    th1 = sector_start
    th2 = sector_start + fan
    ax.plot([target[0], target[0] + R * math.cos(th1)], [target[1], target[1] + R * math.sin(th1)],
            linestyle="--", color="gray", linewidth=1.0)
    ax.plot([target[0], target[0] + R * math.cos(th2)], [target[1], target[1] + R * math.sin(th2)],
            linestyle="--", color="gray", linewidth=1.0)

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Robot motion + camera FOV")

    # set bounds
    xs = np.concatenate([trajA[:, 0], trajB[:, 0], [target[0], goal[0]]])
    ys = np.concatenate([trajA[:, 1], trajB[:, 1], [target[1], goal[1]]])
    pad = 0.4
    ax.set_xlim(xs.min() - pad, xs.max() + pad)
    ax.set_ylim(ys.min() - pad, ys.max() + pad)

    # dynamic artists
    artists = {
        "robotA": draw_robot(ax, trajA[0, 0], trajA[0, 1], trajA[0, 2], color="C0"),
        "robotB": draw_robot(ax, trajB[0, 0], trajB[0, 1], trajB[0, 2], color="C1"),
        "camA": draw_camera_fov(ax, trajA[0, 0], trajA[0, 1], trajA[0, 2], fov, color="C0"),
        "camB": draw_camera_fov(ax, trajB[0, 0], trajB[0, 1], trajB[0, 2], fov, color="C1"),
    }
    time_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top")

    frames = list(range(0, min(len(trajA), len(trajB)), step))

    def update(i):
        k = frames[i]
        # clear previous patches/lines
        artists["robotA"].remove()
        artists["robotB"].remove()
        for ln in artists["camA"] + artists["camB"]:
            ln.remove()

        xA, yA, thA = trajA[k]
        xB, yB, thB = trajB[k]
        artists["robotA"] = draw_robot(ax, xA, yA, thA, color="C0")
        artists["robotB"] = draw_robot(ax, xB, yB, thB, color="C1")
        artists["camA"] = draw_camera_fov(ax, xA, yA, thA, fov, color="C0")
        artists["camB"] = draw_camera_fov(ax, xB, yB, thB, fov, color="C1")
        time_text.set_text(f"t = {k*dt:5.2f} s")

        return [artists["robotA"], artists["robotB"], *artists["camA"], *artists["camB"], time_text]

    anim = FuncAnimation(fig, update, frames=len(frames), interval=dt * step * 1000, blit=False)
    anim.save(out_path, writer=PillowWriter(fps=max(1, int(1 / (dt * step)))))
    plt.close(fig)
    print(f"Saved animation: {out_path}")

# =========================
# Run
# =========================
trajA, logsA, lostA, lostA_sector = sim_holonomic()
trajB, logsB, lostB, diff_met_stop = sim_diff_like()

# logs columns: t, range, alpha, in_view
tA, rA, aA, vA = logsA[:,0], logsA[:,1], logsA[:,2], logsA[:,3]
tB, rB, aB, vB = logsB[:,0], logsB[:,1], logsB[:,2], logsB[:,3]

print("=== Summary ===")
print(f"Holonomic orbit: lost_ratio = {100*lostA/len(tA):.1f}%")
print(f"Holonomic sector: out_ratio = {100*lostA_sector/len(tA):.1f}%  (target-centered {fan_deg:.0f}deg sector)")
print(f"Diff-like      : lost_ratio = {100*lostB/len(tB):.1f}%")
print(f"Diff-like stop : met_stop_condition={diff_met_stop}")

# Final pose errors against the common goal
def pose_error(final_xyz):
    dx = final_xyz[0] - goal[0]
    dy = final_xyz[1] - goal[1]
    pos_err = math.hypot(dx, dy)
    yaw_err = abs(wrap_pi(final_xyz[2] - goal_th))
    return pos_err, yaw_err

eA_pos, eA_yaw = pose_error(trajA[-1])
eB_pos, eB_yaw = pose_error(trajB[-1])
print(f"Holonomic final: pos_err={eA_pos:.3f} m, yaw_err={math.degrees(eA_yaw):.2f} deg")
print(f"Diff-like final: pos_err={eB_pos:.3f} m, yaw_err={math.degrees(eB_yaw):.2f} deg")

# Create GIF animation (requires matplotlib + pillow)
try:
    make_animation(trajA, trajB, out_path="movement.gif", step=3)
except Exception as e:
    print(f"[WARN] GIF generation skipped: {e}")

# =========================
# Visualization
# =========================
plt.figure()
plt.plot(trajA[:,0], trajA[:,1], label="Holonomic orbit (fan)")
plt.plot(trajB[:,0], trajB[:,1], label="Diff-like (turn-go-turn)")
plt.scatter([target[0]], [target[1]], marker="x", label="Target")

# Draw the target-centered sector (fan) for reference.
# Sector starts from target -> initial robot direction and spans +fan_deg.
R = 1.6
th1 = sector_start
th2 = sector_start + fan
plt.plot([target[0], target[0] + R*math.cos(th1)], [target[1], target[1] + R*math.sin(th1)],
         linestyle="--", color="gray", linewidth=1.2, label=f"Sector start")
plt.plot([target[0], target[0] + R*math.cos(th2)], [target[1], target[1] + R*math.sin(th2)],
         linestyle="--", color="gray", linewidth=1.2, label=f"Sector end (+{fan_deg:.0f}°)")

# Mark goal point
plt.scatter([goal[0]], [goal[1]], marker="o", s=60, facecolors="none", edgecolors="k", label="Goal")

plt.axis("equal")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("2D simulation trajectories")
plt.legend()
plt.show()

plt.figure()
plt.plot(tA, np.degrees(aA), label="Holonomic orbit (fan)")
plt.plot(tB, np.degrees(aB), label="Diff-like (turn-go-turn)")
plt.axhline(fov_deg/2, linestyle="--", label="FOV edge (+)")
plt.axhline(-fov_deg/2, linestyle="--", label="FOV edge (-)")
plt.xlabel("t [s]")
plt.ylabel("alpha [deg] (target bearing in robot frame)")
plt.title("Target bearing vs time (out of view if |alpha| > FOV/2)")
plt.legend()
plt.show()

plt.figure()
plt.plot(tA, rA, label="Holonomic orbit (fan)")
plt.plot(tB, rB, label="Diff-like (turn-go-turn)")
plt.axhline(r_goal, linestyle="--", label="goal radius")
plt.xlabel("t [s]")
plt.ylabel("range [m]")
plt.title("Range to target vs time")
plt.legend()
plt.show()
