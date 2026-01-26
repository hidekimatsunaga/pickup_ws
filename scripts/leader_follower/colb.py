# Colabでそのまま実行できます（概念GIF：FTL=Follow-the-Leader形状生成）
# 目的：最初から長さを持ったマニピュレータが，先端の軌跡を弧長で追従して前進する見た目

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import sys
import os

# =========================
# パラメータ
# =========================
N_POINTS = 35           # 点の数（多いほど滑らか）
SPACING = 0.03          # 点間距離（=弧長間隔）
TOTAL_FRAMES = 240
FPS = 25

SHOW_OBSTACLES = False  # 障害物の円を表示したいなら True

# 先端の「行きたい軌道」（曲がりながら前進の例）
def leader_path(t):
    x = 0.15 + 0.006 * t
    y = 0.14 * np.sin(0.05 * t)
    return np.array([x, y])

# 障害物（円）※表示だけ。必要なら当たり判定の押し返しも後で入れられます
obstacles = [
    (0.70,  0.10, 0.08),
    (1.00, -0.12, 0.09),
    (1.25,  0.05, 0.10),
]

# =========================
# FTLのための軌跡（ポリライン）管理
# =========================
# ポリライン points_path: [p0, p1, ..., pk] （p_kが最新=先端）
# cumlen: p0からの累積弧長
points_path = []

def rebuild_cumlen(path):
    if len(path) == 0:
        return np.array([0.0])
    cum = [0.0]
    for i in range(1, len(path)):
        cum.append(cum[-1] + np.linalg.norm(path[i] - path[i-1]))
    return np.array(cum)

def sample_point_by_s(path, cumlen, s_query):
    """
    ポリライン上で「累積弧長が s_query になる点」を線形補間で返す
    s_query は [0, cumlen[-1]] の範囲を想定
    """
    if s_query <= 0.0:
        return path[0].copy()
    if s_query >= cumlen[-1]:
        return path[-1].copy()

    idx = np.searchsorted(cumlen, s_query)
    # idx-1 と idx の間
    s0, s1 = cumlen[idx-1], cumlen[idx]
    p0, p1 = path[idx-1], path[idx]
    if s1 - s0 < 1e-9:
        return p1.copy()
    a = (s_query - s0) / (s1 - s0)
    return (1 - a) * p0 + a * p1

# =========================
# 初期形状：最初から長さを持った直線マニピュレータ
# =========================
leader0 = leader_path(0)
# 先端から後ろに一直線（xマイナス方向）に初期化
init_points = [leader0 - np.array([SPACING * i, 0.0]) for i in range(N_POINTS)]
# 軌跡ポリラインは「根元側→先端側」の順に並べたいので逆順で入れる
# （path[0]が一番古い=根元側、path[-1]が最新=先端）
points_path = init_points[::-1]
cumlen = rebuild_cumlen(points_path)

# =========================
# 描画準備
# =========================
fig, ax = plt.subplots(figsize=(9.0, 3.2))
ax.set_aspect("equal", adjustable="box")
ax.set_xlim(0.0, 1.75)
ax.set_ylim(-0.35, 0.35)
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_title("Follow-the-Leader (Concept): a manipulator with fixed length moving forward")

if SHOW_OBSTACLES:
    for (ox, oy, r) in obstacles:
        ax.add_patch(plt.Circle((ox, oy), r, fill=False, linewidth=2))

# 目標軌道（薄いガイド）
guide = np.array([leader_path(t) for t in range(TOTAL_FRAMES)])
ax.plot(guide[:, 0], guide[:, 1], linewidth=1, alpha=0.4)

# 軌跡とマニピュレータ描画
traj_line, = ax.plot([], [], linestyle="--", linewidth=1)
mani_line, = ax.plot([], [], linewidth=3)
tip_scatter = ax.scatter([], [], s=55)

def init():
    traj_line.set_data([], [])
    mani_line.set_data([], [])
    tip_scatter.set_offsets(np.zeros((1, 2)))
    return traj_line, mani_line, tip_scatter

# =========================
# 1フレーム更新
# =========================
def step(frame):
    global points_path, cumlen

    # 先端を更新（「目標軌道上の点」に置く：概念なので単純化）
    leader = leader_path(frame)

    # 軌跡ポリラインに先端点を追加
    # 直前の点と同じなら追加しない（ゼロ長回避）
    if np.linalg.norm(leader - points_path[-1]) > 1e-6:
        points_path.append(leader.copy())

    # ポリラインが長くなりすぎたら古い部分を削る（マニピュレータ長+αだけ残す）
    # 必要な弧長： (N_POINTS-1)*SPACING ぶん + 余裕
    required = (N_POINTS - 1) * SPACING
    cumlen = rebuild_cumlen(points_path)

    # 先端の累積弧長
    s_tip = cumlen[-1]

    # 根元側が古すぎるなら削る：s_tip - required より前を捨てる
    s_min = max(0.0, s_tip - required - 0.2)  # 0.2は余裕
    # s_minに対応するインデックスを探してスライス
    k = np.searchsorted(cumlen, s_min)
    if k > 0:
        points_path = points_path[k:]
        cumlen = rebuild_cumlen(points_path)
        s_tip = cumlen[-1]

    # FTL形状生成：
    # i番目（先端からi個後ろ）の点は、先端から弧長 i*SPACING だけ戻った位置
    pts = np.zeros((N_POINTS, 2))
    for i in range(N_POINTS):
        s_query = s_tip - i * SPACING
        s_query = max(0.0, s_query)
        pts[i] = sample_point_by_s(points_path, cumlen, s_query)

    # 先端軌跡（表示用）
    traj = np.array(points_path)
    traj_line.set_data(traj[:, 0], traj[:, 1])

    # マニピュレータ（先端→根元の順に描画）
    mani_line.set_data(pts[:, 0], pts[:, 1])

    # 先端
    tip_scatter.set_offsets(pts[0:1, :])

    return traj_line, mani_line, tip_scatter

anim = FuncAnimation(fig, step, frames=TOTAL_FRAMES, init_func=init, blit=True)

# 出力ディレクトリを確保
output_dir = os.path.dirname(os.path.abspath(__file__))
gif_path = os.path.join(output_dir, "ftl_fixed_length_manipulator.gif")

anim.save(gif_path, writer=PillowWriter(fps=FPS))
plt.close(fig)

print(f"Saved: {gif_path}")
print("Animation completed successfully!")
