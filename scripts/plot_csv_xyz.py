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

DEFAULT_CSV = os.path.join(os.path.dirname(__file__), '..', 'src', 'pcc_test', 'pcc_test', 'lut_csv', 'pcc_measure_1024.csv')


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


def try_plot(xs, ys, zs):
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
    plt.show()


def main():
    p = argparse.ArgumentParser(description='CSV の x,y,z を表示・プロット')
    p.add_argument('--file', '-f', default=DEFAULT_CSV, help='読み込むCSVファイル')
    p.add_argument('--head', '-n', type=int, default=20, help='最初の N 行を表示（0 なら表示なし）')
    p.add_argument('--plot', action='store_true', help='matplotlib があれば 3D プロットを表示')
    args = p.parse_args()

    if not os.path.isfile(args.file):
        print(f"エラー: ファイルが見つかりません: {args.file}")
        sys.exit(2)

    try:
        header, xs, ys, zs, rows = read_xyz(args.file)
    except Exception as e:
        print(f"CSV 読み取り中にエラー: {e}")
        sys.exit(1)

    print(f"読み込んだファイル: {args.file}")
    print(f"ヘッダ: {header}")
    print_summary(xs, ys, zs, head=args.head, rows=rows)

    if args.plot:
        try_plot(xs, ys, zs)


if __name__ == '__main__':
    main()
