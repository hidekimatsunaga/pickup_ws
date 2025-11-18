#!/usr/bin/env python3
"""
Adjust u_meas/v_meas by offset so that the first row's measured values match the model.

Usage:
  python3 scripts/adjust_uv_means.py /path/to/uv_compare_v2.csv

This writes a new CSV next to the input file with suffix `_means_corrected_YYYYMMDD_HHMMSS.csv`.
"""
import csv
import sys
import os
import math
from datetime import datetime


def adjust_file(infile):
    if not os.path.isfile(infile):
        print(f"File not found: {infile}")
        return 2
    out_dir = os.path.dirname(infile)
    base = os.path.splitext(os.path.basename(infile))[0]
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    outfile = os.path.join(out_dir, f"{base}_means_corrected_{ts}.csv")

    with open(infile, newline='') as inf:
        reader = csv.reader(inf)
        header = next(reader)
        # expected columns: stamp,dt,u_model,v_model,u_meas,v_meas,ex,ey,e_norm
        hdr_map = {h.strip(): i for i, h in enumerate(header)}
        req = ['u_model', 'v_model', 'u_meas', 'v_meas']
        lowmap = {h.strip().lower(): i for i, h in enumerate(header)}
        # find indices case-insensitively
        try:
            ui = lowmap['u_model']
            vi = lowmap['v_model']
            umi = lowmap['u_meas']
            vmi = lowmap['v_meas']
        except KeyError as e:
            print(f"Required column missing in header: {e}")
            return 3

        # read all rows
        rows = [row for row in reader]
        if len(rows) == 0:
            print("No data rows found")
            return 4

        # first row values
        def tof(val):
            try:
                return float(val)
            except Exception:
                return 0.0

        u_model0 = tof(rows[0][ui])
        v_model0 = tof(rows[0][vi])
        u_meas0 = tof(rows[0][umi])
        v_meas0 = tof(rows[0][vmi])

        du = u_model0 - u_meas0
        dv = v_model0 - v_meas0
        print(f"Applying offset du={du:.6f}, dv={dv:.6f} (so first row will match model)")

        # write corrected file
        out_header = header[:]  # keep same header
        with open(outfile, 'w', newline='') as outf:
            writer = csv.writer(outf)
            writer.writerow(out_header)
            for row in rows:
                # make a copy to avoid modifying original list
                r = list(row)
                # ensure indices exist
                try:
                    u_meas = tof(r[umi])
                    v_meas = tof(r[vmi])
                except Exception:
                    writer.writerow(r)
                    continue
                u_meas_c = u_meas + du
                v_meas_c = v_meas + dv
                # replace meas
                r[umi] = f"{u_meas_c:.6f}"
                r[vmi] = f"{v_meas_c:.6f}"
                # recompute ex,ey,e_norm if columns exist
                if 'ex' in lowmap and 'ey' in lowmap:
                    exi = lowmap['ex']
                    eyi = lowmap['ey']
                    # model indices
                    u_model = tof(r[ui])
                    v_model = tof(r[vi])
                    ex = u_model - u_meas_c
                    ey = v_model - v_meas_c
                    r[exi] = f"{ex:.6f}"
                    r[eyi] = f"{ey:.6f}"
                    # e_norm
                    if 'e_norm' in lowmap:
                        eni = lowmap['e_norm']
                        e_norm = math.hypot(ex, ey)
                        r[eni] = f"{e_norm:.6f}"

                writer.writerow(r)

    print(f"Wrote corrected CSV to: {outfile}")
    return 0


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: adjust_uv_means.py <input_csv>")
        sys.exit(2)
    sys.exit(adjust_file(sys.argv[1]))
