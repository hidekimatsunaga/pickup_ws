#!/usr/bin/env python3
"""
Remove rows from a uv-compare CSV where the error magnitude (e_norm) jumps abruptly.

Heuristic (default): compute consecutive differences of e_norm, take the median absolute
difference (MAD) and mark any row where abs(diff) > k * MAD as a jump. Removed rows are
written to a new CSV with suffix `_cleaned_YYYYMMDD_HHMMSS.csv`.

Usage:
  python3 scripts/remove_means_jumps.py /path/to/uv_compare.csv [--k 4.0] [--abs-threshold 0.1]

Options:
  --k: multiplier for MAD (default 4.0)
  --abs-threshold: absolute diff threshold (if set, also remove diffs larger than this)
"""
import csv
import sys
import os
from datetime import datetime
import math


def read_csv_rows(path):
    with open(path, newline='') as f:
        r = csv.reader(f)
        header = next(r)
        rows = [row for row in r]
    return header, rows


def write_csv(path, header, rows):
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def to_float(s):
    try:
        return float(s)
    except Exception:
        return float('nan')


def median(xs):
    xs = sorted(xs)
    n = len(xs)
    if n == 0:
        return 0.0
    mid = n // 2
    if n % 2 == 1:
        return xs[mid]
    return 0.5 * (xs[mid-1] + xs[mid])


def mad(xs):
    med = median(xs)
    return median([abs(x - med) for x in xs])


def detect_jumps(e_norm_list, k=4.0, abs_threshold=None):
    # diffs between consecutive e_norm
    diffs = [abs(e_norm_list[i] - e_norm_list[i-1]) for i in range(1, len(e_norm_list))]
    if not diffs:
        return set()
    m = median(diffs)
    # fallback if median is zero
    if m == 0:
        m = mad(diffs)
    thr = k * (m if m > 0 else 1e-6)
    removed_idx = set()
    for i, d in enumerate(diffs, start=1):
        if d > thr:
            if abs_threshold is not None and d <= abs_threshold:
                continue
            # mark the later row (i) for removal
            removed_idx.add(i)
    return removed_idx


def main():
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('csv', help='uv compare CSV')
    p.add_argument('--k', type=float, default=4.0, help='MAD multiplier')
    p.add_argument('--abs-threshold', type=float, default=None, help='absolute diff threshold')
    p.add_argument('--out', default=None, help='output path (optional)')
    args = p.parse_args()

    header, rows = read_csv_rows(args.csv)
    # find index of e_norm or compute from ex/ey
    low_hdr = [h.strip().lower() for h in header]
    if 'e_norm' in low_hdr:
        en_i = low_hdr.index('e_norm')
        e_norm = [to_float(r[en_i]) for r in rows]
    else:
        # attempt to compute from ex,ey
        if 'ex' in low_hdr and 'ey' in low_hdr:
            ex_i = low_hdr.index('ex')
            ey_i = low_hdr.index('ey')
            e_norm = [math.hypot(to_float(r[ex_i]), to_float(r[ey_i])) for r in rows]
        else:
            print('No e_norm or ex/ey columns found in CSV header; cannot detect jumps.')
            sys.exit(2)

    removed = detect_jumps(e_norm, k=args.k, abs_threshold=args.abs_threshold)
    kept_rows = [r for i, r in enumerate(rows) if i not in removed]

    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    if args.out:
        outpath = args.out
    else:
        base = os.path.splitext(os.path.basename(args.csv))[0]
        outpath = os.path.join(os.path.dirname(args.csv), f"{base}_cleaned_{ts}.csv")

    write_csv(outpath, header, kept_rows)
    print(f"Wrote cleaned CSV to: {outpath}")
    print(f"Original rows: {len(rows)}, removed: {len(removed)}, remaining: {len(kept_rows)}")


if __name__ == '__main__':
    main()
