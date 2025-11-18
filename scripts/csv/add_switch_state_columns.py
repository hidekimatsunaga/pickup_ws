#!/usr/bin/env python3
"""
Add human-readable switch state columns to a CSV that already has sN columns.

Maps 0 -> 'たるみ' (slack), 1 -> '張り' (taut). Empty values are left blank.

Usage:
  python3 scripts/add_switch_state_columns.py input.csv

Writes: input_with_states_YYYYMMDD_HHMMSS.csv
"""
import csv
import os
import sys
from datetime import datetime


def read_csv(path):
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


def main():
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('input', help='input CSV')
    p.add_argument('--switch-indices', default='7,8,9', help='comma separated 1-based indices to name extra switch cols (default: 7,8,9)')
    args = p.parse_args()

    inp = args.input
    header, rows = read_csv(inp)
    low = [h.strip().lower() for h in header]
    # find all sN columns (e.g., s7, s8, s9)
    s_cols = [(i, header[i]) for i, h in enumerate(low) if h.startswith('s') and h[1:].isdigit()]
    # if header has no sN columns, but rows contain extra columns, assume they are switch cols
    if not s_cols:
        extra = 0
        if rows:
            extra = len(rows[0]) - len(header)
        if extra <= 0:
            print('No sN columns found in header and no extra columns detected; nothing to do.')
            sys.exit(0)
        # parse provided switch indices and map the last `extra` columns to them
        try:
            requested = [int(x) for x in args.switch_indices.split(',')]
        except Exception:
            requested = [7,8,9]
        # take last `extra` entries from requested; if mismatch, generate s1..sN names
        if len(requested) >= extra:
            mapped = requested[-extra:]
        else:
            # fallback: use last `extra` numbers starting at 1
            mapped = list(range(1, extra+1))
        # create synthetic s_cols mapping indices -> names (indices are header indexes for extra cols)
        base_idx = len(header)
        synth = [(base_idx + i, f's{mapped[i]}') for i in range(extra)]
        # append synthetic sN names to header so indices line up
        for (_, name) in synth:
            header.append(name)
        s_cols = synth

    # create new header with additional state columns after each sN
    new_header = []
    for i, h in enumerate(header):
        new_header.append(h)
        for idx, name in s_cols:
            if idx == i:
                # add state column
                new_header.append(f"{name}_state")

    new_rows = []
    for r in rows:
        new_r = []
        for i, val in enumerate(r):
            new_r.append(val)
            for idx, name in s_cols:
                if idx == i:
                    # map 0->たるみ, 1->張り
                    v = val.strip()
                    if v == '0':
                        new_r.append('たるみ')
                    elif v == '1':
                        new_r.append('張り')
                    elif v == '':
                        new_r.append('')
                    else:
                        # unknown numeric -> keep as-is
                        new_r.append(v)
        new_rows.append(new_r)

    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    out = os.path.splitext(inp)[0] + f"_with_states_{ts}.csv"
    write_csv(out, new_header, new_rows)
    print(f"Wrote: {out} (added {len(s_cols)} state columns)")


if __name__ == '__main__':
    main()
