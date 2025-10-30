#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
特定の行範囲だけ、指定列の変化が小さい連続行を間引くツール

使い方の例:
  python thin_in_range.py data.csv \
    --cols u_meas v_meas \
    --range 1201:2500 \
    --tol 1e-4 \
    -o data_thinned.csv \
    --keep-ends

デフォルト：
  - 列は u_meas v_meas
  - 行範囲は全体（--range未指定時）
  - 1-basedの行番号で指定（人が数える「◯行目」基準）
  - 変化判定は「列ごとの差の最大値 >= tol」で保持（any相当）
"""

import argparse
import os
import re
import numpy as np
import pandas as pd

def parse_range(spec: str, n_rows: int):
    """'start:end' を 1-based 包含で受け取り、0-based 包含に直す"""
    if spec is None:
        return 0, n_rows - 1
    m = re.match(r'^\s*(\d+)\s*:\s*(\d+)\s*$', spec)
    if not m:
        raise ValueError("--range は 'start:end' 形式で指定してください（例 1201:2500）")
    s_1b = int(m.group(1))
    e_1b = int(m.group(2))
    if s_1b < 1 or e_1b < 1:
        raise ValueError("行番号は1以上で指定してください")
    if s_1b > e_1b:
        raise ValueError("range の start は end 以下にしてください")
    # 範囲クリップ
    s = max(0, min(n_rows - 1, s_1b - 1))
    e = max(0, min(n_rows - 1, e_1b - 1))
    return s, e

def thin_in_range(df: pd.DataFrame, cols, start_idx: int, end_idx: int,
                  tol: float, mode: str = "any", keep_ends: bool = False, every: int = 0):
    """
    指定範囲 [start_idx, end_idx] のみ、指定列の変化が小さい連続行を間引き。
    - tol: しきい値
    - mode: "any" => いずれかの列で |Δ|>=tol なら保持、"all" => 全列で |Δ|>=tol なら保持
    - every: >0 なら N行ごとに強制保持（間引きしすぎ防止用、OR条件）
    - keep_ends: True なら範囲の先頭・末尾行は必ず残す
    """
    if len(cols) == 0:
        raise ValueError("少なくとも1列は指定してください（--cols）")
    for c in cols:
        if c not in df.columns:
            raise ValueError(f"指定列が見つかりません: {c}")

    n = len(df)
    keep = np.ones(n, dtype=bool)  # まず全行保持、後で範囲内だけ落とす

    # 範囲外は何もしない
    if start_idx >= n or end_idx < 0 or start_idx > end_idx:
        return df, 0

    start_idx = max(0, start_idx)
    end_idx   = min(n - 1, end_idx)
    idxs = np.arange(start_idx, end_idx + 1, dtype=int)
    if len(idxs) <= 1:
        return df, 0

    # 先頭は保持
    last_keep_idx = idxs[0]
    last_vals = df.loc[last_keep_idx, cols].astype(float).to_numpy()
    dropped = 0

    for j, i in enumerate(idxs[1:], start=1):
        vals = df.loc[i, cols].astype(float).to_numpy()
        delta = np.abs(vals - last_vals)
        changed = (delta.max() >= tol) if mode == "any" else np.all(delta >= tol)

        # every > 0 の場合は N行ごとに強制保持
        if every > 0 and (j % every == 0):
            changed = True

        if changed:
            keep[i] = True
            last_keep_idx = i
            last_vals = vals
        else:
            keep[i] = False
            dropped += 1

    if keep_ends:
        keep[idxs[0]] = True
        keep[idxs[-1]] = True  # 末尾は必ず残す

    return df[keep].reset_index(drop=True), dropped

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", help="入力CSV（ヘッダ付き）")
    ap.add_argument("--cols", nargs="+", default=["u_meas", "v_meas"],
                    help="変化を見る列名（複数可）")
    ap.add_argument("--range", dest="row_range", default=None,
                    help="対象行範囲（1-based 包含）例: 1201:2500")
    ap.add_argument("--tol", type=float, default=1e-4, help="変化しきい値（絶対値）")
    ap.add_argument("--mode", choices=["any", "all"], default="any",
                    help="保持判定：any=いずれかの列が tol 以上, all=全列が tol 以上")
    ap.add_argument("--every", type=int, default=0,
                    help=">0 なら N行ごと強制保持（間引きしすぎ防止）")
    ap.add_argument("--keep-ends", action="store_true",
                    help="範囲の先頭/末尾行を必ず保持")
    ap.add_argument("-o", "--out", default=None, help="出力CSV（未指定なら自動命名）")
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    n_rows = len(df)
    s, e = parse_range(args.row_range, n_rows)

    out_df, dropped = thin_in_range(
        df, cols=args.cols, start_idx=s, end_idx=e,
        tol=args.tol, mode=args.mode, keep_ends=args.keep_ends, every=args.every
    )

    if args.out is None:
        base, ext = os.path.splitext(args.csv)
        args.out = f"{base}_thinned_{s+1}-{e+1}_tol{args.tol:g}.csv"

    out_df.to_csv(args.out, index=False)

    kept = len(out_df)
    print(f"[done] total: {n_rows} rows -> {kept} rows (dropped {dropped} rows) in range {s+1}:{e+1}")
    print(f"saved: {args.out}")

if __name__ == "__main__":
    main()
