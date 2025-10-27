#!/usr/bin/env python3
"""
CSV から x,y,z を抽出して表示・プロットする小さなユーティリティ

使い方:
  python3 scripts/plot_csv_xyz.py \
    --file /path/to/pcc_measure_1024.csv --head 20 --plot

オプション:
  --file    : 読み込むCSVファイル（デフォルトはレポジトリ内の該当ファイル）
  --head N  : 最初の N 行だけ表示（0 なら全部）
  --plot    : matplotlib があれば 3D 散布図を表示

このスクリプトは標準ライブラリの csv を使って安全に動作します。
matplotlib がない場合はプロットをスキップしてデータの要約のみ表示します。
"""

import argparse
import csv
import os
import sys
from collections import defaultdict
from datetime import datetime

DEFAULT_CSV = os.path.join(os.path.dirname(__file__), '..', 'src', 'pcc_test', 'pcc_test', 'lut_csv', 'pcc_measure_1024.csv')
DEFAULT_CSV2 = os.path.join(os.path.dirname(__file__), '..', 'src', 'pcc_test', 'pcc_test', 'lut_csv', 'lut_measure_1024.csv')
DEFAULT_OUT_DIR = os.path.join(os.path.dirname(__file__), '..', 'src', 'pcc_test', 'jpg')


def find_xyz_indices(header):
    """ヘッダ行から x,y,z の列インデックスを探す。小文字/大文字を区別しない。"""
    lookup = {h.strip().lower(): i for i, h in enumerate(header)}
    candidates = ['x', 'y', 'z']
    if all(c in lookup for c in candidates):
        return lookup['x'], lookup['y'], lookup['z']
    # フォールバック: 最初の3列を使う
    if len(header) >= 3:
        return 0, 1, 2
    raise ValueError('CSV ヘッダに x,y,z 列が見つからず、代替も使えません')


def read_columns(file_path, cols):
    """汎用: 指定した列名 (小文字照合) を読み出す。存在しない列は例外を投げる。"""
    with open(file_path, newline='') as f:
        reader = csv.reader(f)
        header = next(reader)
        lookup = {h.strip().lower(): i for i, h in enumerate(header)}
        indices = []
        for c in cols:
            key = c.lower()
            if key not in lookup:
                raise ValueError(f"CSV に列 '{c}' が見つかりません (ヘッダ: {header})")
            indices.append(lookup[key])

        cols_data = [[] for _ in cols]
        rows = []
        for row in reader:
            # skip short rows
            if len(row) <= max(indices):
                continue
            try:
                vals = [float(row[i]) for i in indices]
            except Exception:
                continue
            for j, v in enumerate(vals):
                cols_data[j].append(v)
            rows.append((row, tuple(vals)))
    return header, cols_data, rows


def read_xyz(file_path):
    with open(file_path, newline='') as f:
        reader = csv.reader(f)
        header = next(reader)
        xi, yi, zi = find_xyz_indices(header)
        xs, ys, zs = [], [], []
        rows = []
        for row in reader:
            # 空行や短い行をスキップ
            if len(row) <= max(xi, yi, zi):
                continue
            try:
                x = float(row[xi])
                y = float(row[yi])
                z = float(row[zi])
            except ValueError:
                # 変換できない行はスキップ
                continue
            xs.append(x)
            ys.append(y)
            zs.append(z)
            rows.append((row, (x, y, z)))
    return header, xs, ys, zs, rows


def print_summary(xs, ys, zs, head=10, rows=None):
    total = len(xs)
    print(f"合計データ点: {total}")
    if total == 0:
        return
    import statistics
    def stats(arr):
        return statistics.mean(arr), statistics.pstdev(arr)
    mx, sx = stats(xs)
    my, sy = stats(ys)
    mz, sz = stats(zs)
    print("要約:")
    print(f"  x: mean={mx:.6f}, std={sx:.6f}")
    print(f"  y: mean={my:.6f}, std={sy:.6f}")
    print(f"  z: mean={mz:.6f}, std={sz:.6f}")

    if head > 0 and rows is not None:
        print(f"\n最初の {min(head, total)} 行 (xyz):")
        for i in range(min(head, total)):
            x, y, z = rows[i][1]
            print(f"  {i+1:4d}: x={x:.6f}, y={y:.6f}, z={z:.6f}")


def try_plot(xs, ys, zs, save=False, out_dir='.'):
    """3D plot for x,y,z. When save True or no DISPLAY, saves to out_dir/plot_xyz.png."""
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except Exception:
        print("matplotlib が見つからないためプロットをスキップします。\nインストール: pip install matplotlib")
        return
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(xs, ys, zs, s=6)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_title('CSV: x,y,z')
    plt.tight_layout()
    if save or (not ('DISPLAY' in os.environ or 'WAYLAND_DISPLAY' in os.environ)):
        os.makedirs(out_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        out = os.path.join(out_dir, f'pcc_measure_xyz_{ts}.png')
        fig.savefig(out)
        print(f"Saved XYZ single-file plot to {out}")
    else:
        plt.show()


def try_plot_uv(u_vals, v_vals, save=False, out_dir='.', basename='uv_plot', color=None, label=None):
    """2D u-v plot and optionally save to out_dir/{basename}.png

    If `color` is provided, plot all points with that color and add `label` to the legend.
    Otherwise, if many points, color by index to show progression.
    """
    try:
        import matplotlib.pyplot as plt
    except Exception:
        print("matplotlib が見つからないためプロットをスキップします。\nインストール: pip install matplotlib")
        return
    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(111)
    # If many points, color by index to show progression; otherwise draw line+markers
    n = len(u_vals)
    if n == 0:
        print("UV プロット: データ点がありません。")
        return
    if color is not None:
        # uniform color for this dataset
        ax.scatter(u_vals, v_vals, c=color, s=20, label=(label or basename), alpha=0.9)
        ax.legend(loc='best')
        ax.set_title(f'{basename} (n={n})')
    else:
        if n > 50:
            # color by index for visibility
            cmap = plt.get_cmap('viridis')
            c = list(range(n))
            sc = ax.scatter(u_vals, v_vals, c=c, cmap=cmap, s=10)
            plt.colorbar(sc, ax=ax, label='index')
            ax.set_title(f'{basename} (colored by index, n={n})')
        else:
            ax.plot(u_vals, v_vals, '-o', markersize=4)
            ax.set_title(f'{basename} (n={n})')
    ax.set_xlabel('u'); ax.set_ylabel('v')
    ax.grid(True)
    plt.tight_layout()
    if save or (not ('DISPLAY' in os.environ or 'WAYLAND_DISPLAY' in os.environ)):
        os.makedirs(out_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        out = os.path.join(out_dir, f'{basename}_{ts}.png')
        fig.savefig(out, dpi=150)
        print(f"Saved UV single-file plot to {out}")
    else:
        plt.show()


def main():
    p = argparse.ArgumentParser(description='CSV の x,y,z を表示・プロット / 2ファイル比較')
    p.add_argument('--file', '-f', default=DEFAULT_CSV, help='読み込むCSVファイル (主)')
    p.add_argument('--file2', '-g', default=None, help='比較用のCSVファイル (オプション)。指定なければデフォルトの LUT ファイルを使います。')
    p.add_argument('--head', '-n', type=int, default=20, help='最初の N 行を表示（0 なら表示なし）')
    p.add_argument('--plot', action='store_true', help='matplotlib があれば 3D プロットを表示')
    p.add_argument('--save', action='store_true', help='GUI がない環境ではプロットをファイルに保存する')
    p.add_argument('--out-dir', default=DEFAULT_OUT_DIR, help='プロット保存先ディレクトリ（--save 有効時）。デフォルトは src/pcc_test/')
    args = p.parse_args()

    # file2 デフォルトの扱い: 指定がなければ DEFAULT_CSV2 を使う（添付の lut_measure）
    if args.file2 is None:
        args.file2 = DEFAULT_CSV2

    if not os.path.isfile(args.file):
        print(f"エラー: ファイルが見つかりません: {args.file}")
        sys.exit(2)

    # --- main file ---
    try:
        header1, xs1, ys1, zs1, rows1 = read_xyz(args.file)
    except Exception as e:
        print(f"CSV 読み取り中にエラー (file): {e}")
        sys.exit(1)

    print(f"読み込んだファイル: {args.file}")
    print(f"ヘッダ: {header1}")
    print_summary(xs1, ys1, zs1, head=args.head, rows=rows1)

    # --- second file (比較) ---
    if args.file2:
        if not os.path.isfile(args.file2):
            print(f"比較用ファイルが見つかりません: {args.file2}")
            sys.exit(2)

        # 汎用読み込み: まずヘッダを見て u/v か x/y/z かを判定する
        with open(args.file, newline='') as f1:
            reader1 = csv.reader(f1)
            header1_row = next(reader1)
            hdr1 = [h.strip().lower() for h in header1_row]
        with open(args.file2, newline='') as f2:
            reader2 = csv.reader(f2)
            header2_row = next(reader2)
            hdr2 = [h.strip().lower() for h in header2_row]

        # 優先: u/v が両方にあるなら 2D 比較。そうでなければ x/y/z を試す。
        has_uv1 = ('u' in hdr1 and 'v' in hdr1)
        has_xyz1 = ('x' in hdr1 and 'y' in hdr1 and 'z' in hdr1)
        has_uv2 = ('u' in hdr2 and 'v' in hdr2)
        has_xyz2 = ('x' in hdr2 and 'y' in hdr2 and 'z' in hdr2)

        # 混在ケース: 片方が xyz で片方が uv の場合は、個別にプロットを保存して終了する
        if (has_xyz1 and has_uv2) or (has_uv1 and has_xyz2):
            print("ファイルタイプが異なります (xyz と uv)。個別にプロットを作成して保存します。")
            base1 = os.path.basename(args.file)
            base2 = os.path.basename(args.file2)
            # file1
            if has_xyz1:
                # xs1, ys1, zs1 はすでに読み込まれている
                try_plot(xs1, ys1, zs1, args.save, args.out_dir)
            elif has_uv1:
                _, cols1, _ = read_columns(args.file, ['u', 'v'])
                u1, v1 = cols1[0], cols1[1]
                try_plot_uv(u1, v1, args.save, args.out_dir, 'file1_uv', color='C0', label=base1)

            # file2
            if has_xyz2:
                header2, xs2, ys2, zs2, rows2 = read_xyz(args.file2)
                try_plot(xs2, ys2, zs2, args.save, args.out_dir)
            elif has_uv2:
                _, cols2, _ = read_columns(args.file2, ['u', 'v'])
                u2, v2 = cols2[0], cols2[1]
                try_plot_uv(u2, v2, args.save, args.out_dir, 'file2_uv', color='C1', label=base2)

            # 終了
            return

        if ('u' in hdr1 and 'v' in hdr1) and ('u' in hdr2 and 'v' in hdr2):
            # 2D (u,v) 比較
            _, cols1, rows1_cols = read_columns(args.file, ['u', 'v'])
            _, cols2, rows2_cols = read_columns(args.file2, ['u', 'v'])
            u1, v1 = cols1[0], cols1[1]
            u2, v2 = cols2[0], cols2[1]
            base1 = os.path.basename(args.file)
            base2 = os.path.basename(args.file2)
            print(f"\n比較モード: 2D (u,v)。読み込んだファイル: {args.file} と {args.file2}")
            print_summary(u1, v1, [0]*len(u1), head=args.head, rows=[(r,(u1[i],v1[i],0)) for i,r in enumerate(rows1_cols)])
            print(f"\n比較用ファイルヘッダ: {header2_row}")
            print_summary(u2, v2, [0]*len(u2), head=args.head, rows=[(r,(u2[i],v2[i],0)) for i,r in enumerate(rows2_cols)])

            # 差分
            n = min(len(u1), len(u2))
            if n == 0:
                print("比較するデータ点がありません。")
            else:
                import math, statistics
                diffs = [math.hypot(u1[i]-u2[i], v1[i]-v2[i]) for i in range(n)]
                mean_d = statistics.mean(diffs)
                med_d = statistics.median(diffs)
                try:
                    import numpy as _np
                    p95_d = float(_np.percentile(_np.array(diffs), 95))
                    p99_d = float(_np.percentile(_np.array(diffs), 99))
                except Exception:
                    p95_d = None; p99_d = None
                print(f"\n比較要約 (最初の {n} 点): mean={mean_d:.6f}, median={med_d:.6f}, p95={p95_d}, p99={p99_d}")

                if args.plot:
                    try:
                        import matplotlib.pyplot as plt
                    except Exception:
                        print("matplotlib が見つからないためプロットをスキップします。")
                    else:
                        fig = plt.figure(figsize=(10, 6))
                        ax = fig.add_subplot(121)
                        ax.plot(u1, v1, '-o', markersize=3, color='C0', label=base1, linewidth=1.0, alpha=0.9)
                        ax.plot(u2, v2, '-s', markersize=3, color='C1', label=base2, linewidth=1.0, alpha=0.8)
                        ax.set_xlabel('u'); ax.set_ylabel('v'); ax.set_title('u-v Trajectories')
                        ax.legend(loc='best'); ax.grid(True)

                        ax2 = fig.add_subplot(122)
                        ax2.hist(diffs, bins=40, color='C2')
                        ax2.set_title('Difference histogram (u,v norm)')
                        plt.tight_layout()
                        if args.save or (not ('DISPLAY' in os.environ or 'WAYLAND_DISPLAY' in os.environ)):
                            outname = f"uv_compare_{os.path.splitext(base1)[0]}_vs_{os.path.splitext(base2)[0]}.png"
                            outpath = os.path.join(args.out_dir, outname)
                            plt.savefig(outpath, dpi=150)
                            print(f"Saved UV compare plot to {outpath}")
                        else:
                            plt.show()
        else:
            # fallback to xyz comparison (existing behavior)
            header2, xs2, ys2, zs2, rows2 = read_xyz(args.file2)

            print(f"\n比較用ファイル: {args.file2}")
            print(f"ヘッダ: {header2}")
            print_summary(xs2, ys2, zs2, head=args.head, rows=rows2)

            # 比較: 行数の最小値まで element-wise 差分を取る
            n = min(len(xs1), len(xs2))
            if n == 0:
                print("比較するデータ点がありません。")
            else:
                diffs = []
                import math
                for i in range(n):
                    dx = xs1[i] - xs2[i]
                    dy = ys1[i] - ys2[i]
                    dz = zs1[i] - zs2[i]
                    norm = math.sqrt(dx*dx + dy*dy + dz*dz)
                    diffs.append(norm)

                import statistics
                mean_d = statistics.mean(diffs)
                med_d = statistics.median(diffs)
                p95_d = percentile = None
                try:
                    import numpy as _np
                    p95_d = float(_np.percentile(_np.array(diffs), 95))
                    p99_d = float(_np.percentile(_np.array(diffs), 99))
                except Exception:
                    p95_d = None
                    p99_d = None

                print(f"\n比較要約 (最初の {n} 点): mean={mean_d:.6f}, median={med_d:.6f}, p95={p95_d}, p99={p99_d}")

                if args.plot:
                    # プロット: 両方の散布図と差分ヒストグラム
                    try:
                        import matplotlib.pyplot as plt
                        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
                    except Exception:
                        print("matplotlib が見つからないためプロットをスキップします。")
                    else:
                        fig = plt.figure(figsize=(10, 8))
                        ax = fig.add_subplot(221, projection='3d')
                        ax.scatter(xs1, ys1, zs1, s=6, label=os.path.basename(args.file), c='C0')
                        ax.scatter(xs2, ys2, zs2, s=6, label=os.path.basename(args.file2), alpha=0.6, c='C1')
                        ax.set_title('file1 vs file2 (3D)')
                        ax.legend()

                        ax2 = fig.add_subplot(222)
                        ax2.plot(diffs)
                        ax2.set_title('Difference norm over index')
                        ax2.set_xlabel('index'); ax2.set_ylabel('||delta||')

                        ax3 = fig.add_subplot(223)
                        ax3.hist(diffs, bins=40, color='C2')
                        ax3.set_title('Difference histogram')

                        plt.tight_layout()
                        if args.save or (not ('DISPLAY' in os.environ or 'WAYLAND_DISPLAY' in os.environ)):
                            outname = f"xyz_compare_{os.path.splitext(os.path.basename(args.file))[0]}_vs_{os.path.splitext(os.path.basename(args.file2))[0]}.png"
                            outpath = os.path.join(args.out_dir, outname)
                            plt.savefig(outpath, dpi=150)
                            print(f"Saved XYZ compare plot to {outpath}")
                        else:
                            plt.show()
    else:
        # file2 が指定されていない場合は従来通り単体表示/プロット
        if args.plot:
            try_plot(xs1, ys1, zs1, args.save, args.out_dir)


if __name__ == '__main__':
    main()
