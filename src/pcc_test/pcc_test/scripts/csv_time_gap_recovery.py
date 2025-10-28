#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import numpy as np
import pandas as pd
from datetime import datetime

# ...existing code...
# ===== CONFIG (ここを編集すれば動作を変更できます) =====
# 入力 CSV（フルパスか相対パス）
INPUT_CSV = 'src/pcc_test/pcc_test/lut_csv/uv_compare_1029.csv'
# ギャップ判定閾値 [s]（この値より大きければ詰める対象）
GAP_THRESH = 0.5
# ギャップを詰めたあと残す幅 [s]（0.0 にすると完全に詰める）
TARGET_GAP = 0.0
# 出力ファイル（Noneなら INPUT_CSV+'_shifted.csv'）
OUT_CSV = None
# True にすると入力ファイルを上書き（.bak にリネームして退避）
INPLACE = False
# True で実際に書き出さず Dry-run（処理内容だけ表示）
DRY_RUN = False
# ===== CONFIG END =====

def shift_timestamps(stamps, gap_thresh, target_gap=0.0):
    stamps = stamps.astype(float).copy()
    diffs = np.diff(stamps)
    cumulative_shift = 0.0
    shifts = np.zeros_like(stamps)
    for i, dt in enumerate(diffs):
        if dt > gap_thresh:
            # amount to remove so the gap becomes target_gap
            remove = dt - target_gap
            cumulative_shift += remove
        shifts[i+1] = cumulative_shift
    new_stamps = stamps - shifts
    return new_stamps, shifts

def main():
    csv_path = INPUT_CSV
    gap = GAP_THRESH
    target = TARGET_GAP
    out = OUT_CSV
    inplace = INPLACE
    dry = DRY_RUN

    if not os.path.exists(csv_path):
        raise FileNotFoundError(csv_path)
    df = pd.read_csv(csv_path)
    if 'stamp' not in df.columns:
        raise RuntimeError('stamp column not found')

    stamps = df['stamp'].to_numpy(dtype=float)
    new_stamps, shifts = shift_timestamps(stamps, gap, target)

    # report gaps and total shift
    diffs = np.diff(stamps)
    gap_idxs = np.where(diffs > gap)[0]
    if gap_idxs.size == 0:
        print("no gaps > gap_thresh found")
    else:
        print("found gaps (index i -> i+1):")
        for i in gap_idxs:
            print(f"  idx {i} : {stamps[i]:.6f} -> {stamps[i+1]:.6f}  gap={diffs[i]:.6f}s  shift_after={shifts[i+1]:.6f}s")
    total_shift = shifts[-1] if shifts.size else 0.0
    print(f"total time removed from end: {total_shift:.6f}s")

    if dry:
        return

    df2 = df.copy()
    df2['stamp'] = new_stamps
    # recompute dt if present or add it
    dt_col = np.concatenate(([0.0], np.diff(new_stamps)))
    df2['dt'] = dt_col

    out_path = csv_path if inplace else (out if out else os.path.splitext(csv_path)[0] + '_shifted.csv')
    if inplace:
        bak = csv_path + '.bak.' + datetime.now().strftime('%Y%m%d_%H%M%S')
        os.rename(csv_path, bak)
        print(f"original moved to {bak}")
    os.makedirs(os.path.dirname(out_path) or '.', exist_ok=True)
    df2.to_csv(out_path, index=False, float_format='%.6f')
    print(f"Saved shifted csv: {out_path}")

if __name__ == '__main__':
    main()
# ...existing code...