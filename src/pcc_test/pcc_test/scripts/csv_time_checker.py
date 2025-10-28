#!/usr/bin/env python3
# ...existing code...
import os, sys
import pandas as pd
import numpy as np

CSV = 'src/pcc_test/pcc_test/lut_csv/uv_compare_1029.csv'

def find_gaps(df, gap_thresh=0.5):
    stamps = df['stamp'].to_numpy()
    dt = np.diff(stamps)
    gaps_idx = np.where(dt > gap_thresh)[0]
    gaps = [(int(i), float(stamps[i]), float(stamps[i+1]), float(dt[i])) for i in gaps_idx]
    return gaps

def find_missing_switch_segs(df, cols=('7','8','9')):
    is_missing = df[list(cols)].isnull().any(axis=1)
    # indices where missing
    idx = np.where(is_missing)[0]
    if idx.size==0:
        return []
    # compress into contiguous ranges
    ranges = []
    start = idx[0]
    prev = idx[0]
    for i in idx[1:]:
        if i == prev + 1:
            prev = i
            continue
        ranges.append((int(start), int(prev)))
        start = i; prev = i
    ranges.append((int(start), int(prev)))
    return ranges, is_missing

def main():
    if not os.path.exists(CSV):
        print("CSV not found:", CSV); return
    df = pd.read_csv(CSV)
    print("rows:", len(df), "time range:", df['stamp'].iloc[0], "->", df['stamp'].iloc[-1])

    gaps = find_gaps(df, gap_thresh=0.5)  # 閾値[s] はここで変更
    if gaps:
        print("\nタイムギャップ (>0.5s) found:")
        for i, t0, t1, d in gaps:
            print(f"  idx {i} -> {i+1}: {t0} -> {t1}  gap={d:.3f}s")
    else:
        print("\n大きなタイムギャップは見つかりませんでした（閾値 0.5s）")

    ranges, is_missing = find_missing_switch_segs(df)
    if ranges:
        print("\nswitch(7/8/9) が欠損している連続区間:")
        for s,e in ranges:
            print(f"  rows {s}..{e}  stamps {df['stamp'].iloc[s]} .. {df['stamp'].iloc[e]}  count={e-s+1}")
    else:
        print("\nswitch 欠損はありません")

    # もし詳細行を見たいなら下を有効に
    # print("\nmissing rows sample:")
    # print(df[is_missing].head(20))

if __name__ == '__main__':
    main()
# ...existing code...