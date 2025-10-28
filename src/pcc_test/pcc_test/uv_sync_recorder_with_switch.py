#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UV sync recorder that also records selected /switch values.

This extends the behavior of `uv_csv_recorder.py` (pairing model/meas PointStamped)
and appends selected switch entries (by default 7,8,9 -> indices 6,7,8) to each CSV row.
"""
import csv
import os
import math
from collections import deque
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int8MultiArray


def to_sec(msg_time) -> float:
    return float(msg_time.sec) + float(msg_time.nanosec) * 1e-9


def normalize_flip_mode(mode: str) -> str:
    if mode is None:
        return ''
    m = str(mode).strip().lower()
    if m in ('', 'none', 'n'):
        return ''
    if m in ('x', 'u'):
        return 'x'
    if m in ('y', 'v'):
        return 'y'
    if m in ('xy', 'yx', 'both', 'all'):
        return 'xy'
    return ''


def apply_flip(u: float, v: float, mode: str) -> Tuple[float, float]:
    if 'x' in mode:
        u = -u
    if 'y' in mode:
        v = -v
    return u, v


class UVSyncRecorderWithSwitch(Node):
    def __init__(self):
        super().__init__('uv_sync_recorder_with_switch')

        # parameters (defaults mirror uv_csv_recorder)
        self.declare_parameter('csv_path', '/home/matsunaga-h/pickup_ws/src/pcc_test/pcc_test/lut_csv/uv_compare_1028_v2.csv')
        self.declare_parameter('model_topic', '/pcc_tip_on_base_plane')
        self.declare_parameter('meas_topic',  '/meas_tip_on_base_plane')
        self.declare_parameter('max_pair_dt', 0.050)
        self.declare_parameter('buffer_sec',  2.0)
        self.declare_parameter('flip_model', 'none')
        self.declare_parameter('flip_meas',  'none')
        # which switch indices to save (1-based friendly). default [7,8,9]
        self.declare_parameter('switch_indices', [7,8,9])

        self.csv_path    = self.get_parameter('csv_path').value
        self.model_topic = self.get_parameter('model_topic').value
        self.meas_topic  = self.get_parameter('meas_topic').value
        self.max_pair_dt = float(self.get_parameter('max_pair_dt').value)
        self.buffer_sec  = float(self.get_parameter('buffer_sec').value)

        self.flip_model_mode = normalize_flip_mode(self.get_parameter('flip_model').value)
        self.flip_meas_mode  = normalize_flip_mode(self.get_parameter('flip_meas').value)

        # switch indices (convert to zero-based)
        self.switch_idx_1b = list(self.get_parameter('switch_indices').value)
        try:
            self.switch_idx = [int(x)-1 for x in self.switch_idx_1b]
        except Exception:
            self.switch_idx = [6,7,8]

        # buffers
        self.buf_model = deque()
        self.buf_meas  = deque()

        # latest switch message
        self.latest_switch = None  # list or None

        # subscriptions
        self.sub_model = self.create_subscription(PointStamped, self.model_topic, self.cb_model, 50)
        self.sub_meas  = self.create_subscription(PointStamped, self.meas_topic,  self.cb_meas,  50)
        self.sub_switch = self.create_subscription(Int8MultiArray, '/switch', self.cb_switch, 10)

        self._ensure_header()
        self.get_logger().info(
            f"UVSyncRecorderWithSwitch ready. csv='{self.csv_path}', max_pair_dt={self.max_pair_dt}, buffer_sec={self.buffer_sec}, switch_idx={self.switch_idx_1b}"
        )

    def _ensure_header(self):
        need_write = True
        if os.path.exists(self.csv_path) and os.path.getsize(self.csv_path) > 0:
            need_write = False
        if need_write:
            os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
            with open(self.csv_path, 'w', newline='') as f:
                w = csv.writer(f)
                cols = ['stamp','dt','u_model','v_model','u_meas','v_meas','ex','ey','e_norm']
                # add switch column headers using 1-based indices
                # use plain numbers for switch column names (e.g. '7','8','9')
                cols += [str(idx) for idx in self.switch_idx_1b]
                w.writerow(cols)

    # callbacks
    def cb_model(self, msg: PointStamped):
        t = to_sec(msg.header.stamp)
        u = float(msg.point.x)
        v = float(msg.point.y)
        u, v = apply_flip(u, v, self.flip_model_mode)
        self.buf_model.append((t, u, v))
        self._prune_buffers(t)
        self._try_match_and_write()

    def cb_meas(self, msg: PointStamped):
        t = to_sec(msg.header.stamp)
        u = float(msg.point.x)
        v = float(msg.point.y)
        u, v = apply_flip(u, v, self.flip_meas_mode)
        self.buf_meas.append((t, u, v))
        self._prune_buffers(t)
        self._try_match_and_write()

    def cb_switch(self, msg: Int8MultiArray):
        try:
            self.latest_switch = list(msg.data)
        except Exception:
            # use warning so it appears in standard ROS2 logs
            self.get_logger().warning('Invalid /switch message received')

    def _prune_buffers(self, now_t: float):
        while self.buf_model and (now_t - self.buf_model[0][0] > self.buffer_sec):
            self.buf_model.popleft()
        while self.buf_meas and (now_t - self.buf_meas[0][0] > self.buffer_sec):
            self.buf_meas.popleft()

    def _find_best_pair(self) -> Optional[Tuple[Tuple[float,float,float], Tuple[float,float,float]]]:
        if not self.buf_model or not self.buf_meas:
            return None
        t_m, um, vm = self.buf_model[-1]
        best = None
        best_dt = 1e9
        best_idx = -1
        for i, (t_e, ue, ve) in enumerate(self.buf_meas):
            dt = abs(t_m - t_e)
            if dt < best_dt:
                best_dt = dt
                best = ((t_m, um, vm), (t_e, ue, ve))
                best_idx = i
        if best and best_dt <= self.max_pair_dt:
            self.buf_meas.remove(self.buf_meas[best_idx])
            self.buf_model.pop()
            return best
        return None

    def _try_match_and_write(self):
        pair = self._find_best_pair()
        if pair is None:
            return
        (t_m, um, vm), (t_e, ue, ve) = pair
        stamp = 0.5 * (t_m + t_e)
        dt = abs(t_m - t_e)
        ex = ue - um
        ey = ve - vm
        e_norm = math.hypot(ex, ey)

        row = [f"{stamp:.6f}", f"{dt:.6f}", f"{um:.6f}", f"{vm:.6f}", f"{ue:.6f}", f"{ve:.6f}", f"{ex:.6f}", f"{ey:.6f}", f"{e_norm:.6f}"]

        # append switch values: if latest_switch available use selected indices, else blanks
        if self.latest_switch is not None:
            for idx in self.switch_idx:
                try:
                    row.append(str(int(self.latest_switch[idx])))
                except Exception:
                    row.append('')
        else:
            row += [''] * len(self.switch_idx)

        with open(self.csv_path, 'a', newline='') as f:
            w = csv.writer(f)
            w.writerow(row)

        # Log a concise info message including the timestamp, dt (ms), error norm and the switch values written
        try:
            # switch columns follow the first 9 columns (stamp..e_norm)
            switch_vals = row[9:]
        except Exception:
            switch_vals = []
        self.get_logger().info(
            f"wrote row to {self.csv_path}: stamp={stamp:.6f}, dt={dt*1000:.1f} ms, e_norm={e_norm:.6f}, switches={switch_vals}"
        )


def main():
    rclpy.init()
    rclpy.spin(UVSyncRecorderWithSwitch())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
