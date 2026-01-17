#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
段差(縁石)の前で「段差に正対したまま」停止線へ整列して接近するタスクの簡易2Dシミュレーション。
A: ホロノミック(横移動可) → ヨー角を段差法線に固定したまま、x,yで誤差を潰す
B: 対向2輪相当(y封印)      → v, ωのみ。横誤差を潰すために回頭が必要になりがち

可視化:
- 段差ライン、停止線、ターゲット
- 2方式の軌跡、姿勢矢印
- カメラ視野(FOV)扇形
- ターゲットが視野内かどうかを色で表示
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge
from matplotlib.animation import FuncAnimation

# ----------------------------
# ユーティリティ
# ----------------------------
def wrap_pi(a):
    return (a + np.pi) % (2*np.pi) - np.pi

def rot2(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s],
                     [s,  c]])

def in_fov(xy, yaw, target_xy, fov_deg=70.0, max_range=2.0):
    """
    yaw方向を視線中心とする扇形FOV内にターゲットが入っているか
    """
    v = target_xy - xy
    r = np.linalg.norm(v)
    if r < 1e-9 or r > max_range:
        return False
    ang = wrap_pi(np.arctan2(v[1], v[0]) - yaw)
    return (abs(ang) <= np.deg2rad(fov_deg) / 2.0)

# ----------------------------
# シナリオ設定（ここをいじる）
# ----------------------------
np.random.seed(0)

# 段差(縁石)を x = 0 の直線とする（段差法線は +x）
CURB_X = 0.0

# 段差手前に停止線（安全に近づける限界）を x = -STOP_DIST とする
STOP_DIST = 0.25  # [m] 段差からの手前距離
STOP_X = CURB_X - STOP_DIST

# ターゲット（ゴミ）は段差の向こう側（x>0）にある想定
TARGET = np.array([0.35, 0.25])  # [m] (x,y)

# 初期姿勢（段差から少し離れた位置、yずれあり）
x0 = np.array([-1.00, -0.25])
yaw0 = np.deg2rad(20.0)  # 初期は少し斜めを向いている

# 目標：停止線上でターゲットのyに整列し、段差法線(+x)に正対 (yaw=0)
YAW_DES = 0.0
GOAL = np.array([STOP_X, TARGET[1]])  # 停止線上でyだけ揃える

# シミュレーション刻み
dt = 0.05
T = 20.0
N = int(T/dt)

# 視野(FOV)設定
FOV_DEG = 70.0
FOV_RANGE = 2.0

# ----------------------------
# コントローラ設定
# ----------------------------

# A: ホロノミック（x,y速度を直接出せる）
Kp_xy = 1.6
Kp_yaw = 3.0
V_MAX = 0.6
W_MAX = 2.5

# B: 対向2輪相当（v,ωのみ）
Kp_head = 3.0      # 目標ヘディングへの追従
Kp_yaw2 = 2.5      # 最終的に正対へ戻す
V_MAX2 = 0.5
W_MAX2 = 2.5

# Bで「横ずれを消したいので、いったんターゲットyへ向けて回頭→前進→最後に正対へ戻す」
# そのための2フェーズ判定
Y_ALIGN_THRESH = 0.03  # [m] y誤差がこれ以下なら最終正対フェーズへ
X_THRESH = 0.03        # [m] x誤差がこれ以下なら停止


# ----------------------------
# シミュレーション本体
# ----------------------------
def simulate():
    # 状態: [x, y, yaw]
    A = np.zeros((N, 3))
    B = np.zeros((N, 3))
    A[0, :2], A[0, 2] = x0, yaw0
    B[0, :2], B[0, 2] = x0, yaw0

    A_visible = np.zeros(N, dtype=bool)
    B_visible = np.zeros(N, dtype=bool)

    for k in range(N-1):
        # ---------- A: holonomic ----------
        p = A[k, :2]
        yaw = A[k, 2]
        e = GOAL - p

        # 目標へ向かう平面速度
        v_xy = Kp_xy * e
        v_norm = np.linalg.norm(v_xy)
        if v_norm > V_MAX:
            v_xy = v_xy / v_norm * V_MAX

        # yawは段差に正対を維持
        e_yaw = wrap_pi(YAW_DES - yaw)
        w = np.clip(Kp_yaw * e_yaw, -W_MAX, W_MAX)

        p_next = p + v_xy * dt
        yaw_next = wrap_pi(yaw + w * dt)

        # 停止条件（停止線付近に到達したらxをこれ以上進めない）
        if p_next[0] > GOAL[0]:
            p_next[0] = GOAL[0]

        A[k+1, :2] = p_next
        A[k+1, 2] = yaw_next

        # ---------- B: differential (y sealed) ----------
        p = B[k, :2]
        yaw = B[k, 2]
        e = GOAL - p

        # フェーズ:
        # 1) y誤差が大きい間は「GOALへ向く」ヘディングで横誤差を潰す
        # 2) yが揃ったら「段差正対(yaw=0)」を優先しつつxだけ詰める
        if abs(e[1]) > Y_ALIGN_THRESH:
            # GOAL方向へ向ける
            head_des = np.arctan2(e[1], e[0])
            e_head = wrap_pi(head_des - yaw)
            w = np.clip(Kp_head * e_head, -W_MAX2, W_MAX2)

            # 前進速度は「前向き成分がある時だけ」出す
            v = V_MAX2 * np.cos(e_head)
            v = np.clip(v, 0.0, V_MAX2)
        else:
            # 最終フェーズ: 正対へ戻し、x方向に近づく
            e_yaw = wrap_pi(YAW_DES - yaw)
            w = np.clip(Kp_yaw2 * e_yaw, -W_MAX2, W_MAX2)

            # x誤差が残っていれば前進（正対に近いほど出す）
            ex = (GOAL[0] - p[0])
            v = 0.0
            if ex > X_THRESH:
                v = V_MAX2 * (1.0 - min(abs(e_yaw)/np.deg2rad(60.0), 1.0))
                v = np.clip(v, 0.0, V_MAX2)

        # 状態更新
        p_next = p + np.array([np.cos(yaw), np.sin(yaw)]) * v * dt
        yaw_next = wrap_pi(yaw + w * dt)

        # 停止線を越えない
        if p_next[0] > GOAL[0]:
            p_next[0] = GOAL[0]

        B[k+1, :2] = p_next
        B[k+1, 2] = yaw_next

        # 可視判定
        A_visible[k] = in_fov(A[k, :2], A[k, 2], TARGET, FOV_DEG, FOV_RANGE)
        B_visible[k] = in_fov(B[k, :2], B[k, 2], TARGET, FOV_DEG, FOV_RANGE)

    # 最後の時刻も判定
    A_visible[-1] = in_fov(A[-1, :2], A[-1, 2], TARGET, FOV_DEG, FOV_RANGE)
    B_visible[-1] = in_fov(B[-1, :2], B[-1, 2], TARGET, FOV_DEG, FOV_RANGE)

    return A, B, A_visible, B_visible

A, B, A_vis, B_vis = simulate()

# ----------------------------
# 可視化（アニメーション）
# ----------------------------
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_aspect('equal', adjustable='box')

# 描画範囲
xmin = min(np.min(A[:,0]), np.min(B[:,0]), TARGET[0]) - 0.3
xmax = max(np.max(A[:,0]), np.max(B[:,0]), TARGET[0]) + 0.3
ymin = min(np.min(A[:,1]), np.min(B[:,1]), TARGET[1]) - 0.3
ymax = max(np.max(A[:,1]), np.max(B[:,1]), TARGET[1]) + 0.3
ax.set_xlim(xmin, xmax)
ax.set_ylim(ymin, ymax)

# 段差ライン x=0 と停止線 x=STOP_X
ax.axvline(CURB_X, linewidth=2)
ax.text(CURB_X+0.01, ymax-0.05, "curb (x=0)")
ax.axvline(STOP_X, linestyle='--', linewidth=2)
ax.text(STOP_X+0.01, ymax-0.12, "stop line")

# ターゲット
target_sc = ax.scatter([TARGET[0]], [TARGET[1]], s=80, marker='*')
ax.text(TARGET[0]+0.02, TARGET[1]+0.02, "target")

# 軌跡ライン
(lineA,) = ax.plot([], [], linewidth=2, label="A: holonomic (keep yaw)")
(lineB,) = ax.plot([], [], linewidth=2, label="B: differential (y sealed)")

# 現在位置
ptA = ax.scatter([], [], s=40)
ptB = ax.scatter([], [], s=40)

# 姿勢矢印（線で代用）
(arrowA,) = ax.plot([], [], linewidth=2)
(arrowB,) = ax.plot([], [], linewidth=2)

# FOV扇形（Wedge）
fovA = Wedge((0,0), FOV_RANGE, 0, 0, alpha=0.15)
fovB = Wedge((0,0), FOV_RANGE, 0, 0, alpha=0.15)
ax.add_patch(fovA)
ax.add_patch(fovB)

# 凡例
ax.legend(loc="lower left")

# テキスト（可視状態など）
txt = ax.text(xmin+0.02, ymax-0.2, "", fontsize=10)

def update(k):
    # 軌跡
    lineA.set_data(A[:k+1,0], A[:k+1,1])
    lineB.set_data(B[:k+1,0], B[:k+1,1])

    # 点
    ptA.set_offsets([A[k,0], A[k,1]])
    ptB.set_offsets([B[k,0], B[k,1]])

    # 姿勢矢印
    L = 0.12
    a0 = A[k,:2]
    b0 = B[k,:2]
    arrowA.set_data([a0[0], a0[0]+L*np.cos(A[k,2])],
                    [a0[1], a0[1]+L*np.sin(A[k,2])])
    arrowB.set_data([b0[0], b0[0]+L*np.cos(B[k,2])],
                    [b0[1], b0[1]+L*np.sin(B[k,2])])

    # FOV更新
    def set_fov(wedge, p, yaw):
        wedge.center = (p[0], p[1])
        wedge.r = FOV_RANGE
        th = np.rad2deg(yaw)
        wedge.theta1 = th - FOV_DEG/2.0
        wedge.theta2 = th + FOV_DEG/2.0

    set_fov(fovA, A[k,:2], A[k,2])
    set_fov(fovB, B[k,:2], B[k,2])

    # 可視/不可視でターゲット色を変える（A,B両方見えているかも表示）
    a_ok = A_vis[k]
    b_ok = B_vis[k]
    # 色指定は最小限（見分けのため）
    if a_ok and b_ok:
        target_sc.set_color("green")
        vis_str = "target visible: A=YES, B=YES"
    elif a_ok and (not b_ok):
        target_sc.set_color("orange")
        vis_str = "target visible: A=YES, B=NO"
    elif (not a_ok) and b_ok:
        target_sc.set_color("purple")
        vis_str = "target visible: A=NO, B=YES"
    else:
        target_sc.set_color("red")
        vis_str = "target visible: A=NO, B=NO"

    # 指標（途中経過）
    eyA = GOAL[1] - A[k,1]
    eyB = GOAL[1] - B[k,1]
    yawA = np.rad2deg(wrap_pi(A[k,2]-YAW_DES))
    yawB = np.rad2deg(wrap_pi(B[k,2]-YAW_DES))
    txt.set_text(
        f"t={k*dt:4.2f}s | {vis_str}\n"
        f"A: y_err={eyA:+.3f} m, yaw_err={yawA:+.1f} deg\n"
        f"B: y_err={eyB:+.3f} m, yaw_err={yawB:+.1f} deg"
    )

    return lineA, lineB, ptA, ptB, arrowA, arrowB, fovA, fovB, target_sc, txt

ani = FuncAnimation(fig, update, frames=N, interval=30, blit=False)

plt.title("Approach & align to stop line while facing the curb (yaw fixed to curb normal)")
plt.xlabel("x [m] (curb at x=0)")
plt.ylabel("y [m]")

# そのまま表示（保存したい場合は下のsaveを有効化）
plt.show()

# --- 保存例（必要なら有効化） ---
# ani.save("curb_alignment.gif", writer="pillow", fps=int(1/dt))
# ani.save("curb_alignment.mp4", writer="ffmpeg", fps=int(1/dt))
