import os
import sys
import argparse
import numpy as np

# adjust path so importRosbag module is findable
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'realsense-ros', 'realsense2_camera', 'scripts')))
from importRosbag import importRosbag

def read_csv_stamps(csv_path):
    stamps = []
    with open(csv_path, 'r') as f:
        hdr = f.readline()
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(',')
            try:
                stamps.append(float(parts[0]))
            except:
                pass
    return np.array(stamps)

def scan_bags(bags_dir, csv_min, csv_max, tol=0.5):
    candidates = []
    for root,_,files in os.walk(bags_dir):
        for fn in files:
            if not fn.endswith('.bag'):
                continue
            bag = os.path.join(root, fn)
            try:
                # import everything supported (may be slow)
                data = importRosbag(os.path.abspath(bag), disable_bar=True, log='ERROR')
            except Exception as e:
                print(f"skip {bag}: import error {e}")
                continue
            for topic, topic_dict in data.items():
                # many importers use 'ts' key or 'ts' inside
                ts = None
                if isinstance(topic_dict, dict):
                    if 'ts' in topic_dict:
                        ts = np.array(topic_dict['ts'], dtype=np.float64)
                    elif 'point' in topic_dict and 'ts' in topic_dict:
                        ts = np.array(topic_dict['ts'], dtype=np.float64)
                if ts is None:
                    # try common keys
                    for k in ('ts','time','stamp'):
                        if k in topic_dict:
                            try:
                                ts = np.array(topic_dict[k], dtype=np.float64)
                                break
                            except:
                                pass
                if ts is None or ts.size==0:
                    continue
                tmin, tmax = ts.min(), ts.max()
                # overlap test
                if (tmin <= csv_max + tol) and (tmax >= csv_min - tol):
                    candidates.append((bag, topic, float(tmin), float(tmax)))
    return candidates

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--csv', required=True)
    p.add_argument('--bags', required=True, help='directory with .bag files (e.g. src/bags/)')
    p.add_argument('--tol', type=float, default=0.5, help='seconds tolerance for overlap')
    args = p.parse_args()

    stamps = read_csv_stamps(args.csv)
    if stamps.size==0:
        print("no stamps read from csv")
        return
    csv_min, csv_max = float(stamps.min()), float(stamps.max())
    print("csv time range:", csv_min, csv_max)

    candidates = scan_bags(args.bags, csv_min, csv_max, tol=args.tol)
    if not candidates:
        print("no candidate bags found")
        return
    print("candidates found (bag, topic, tmin, tmax):")
    for c in candidates:
        print(c)

if __name__ == '__main__':
    main()