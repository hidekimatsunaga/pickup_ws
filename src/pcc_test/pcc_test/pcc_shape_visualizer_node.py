#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray

# Import helper functions from the existing module in the same package
from pcc_test.pcc_visualizer_node import (
    pcc_centerline_xy,
    theta_from_motor_deg_constrained,
)


class PCCShapeVisualizer(Node):
    """
    Subscribe to /motor_current_angles and compute PCC centerline (X,Y).
    Publish visualization_msgs/Marker for RViz:
      - LINE_STRIP: predicted centerline in base frame
      - SPHERE: predicted tip
      - SPHERE: measured tip (from /meas_tip_on_base_plane)
    """

    def __init__(self):
        super().__init__('pcc_shape_visualizer')

        # params (mirror the model's defaults to keep behavior consistent)
        self.declare_parameter('L', [0.20, 0.20, 0.20])
        self.declare_parameter('tip_extra', 0.10)
        self.declare_parameter('wire_radius', 0.019)
        self.declare_parameter('pulley_radius', 0.008)
        self.declare_parameter('motor_indices', [6, 7, 8])
        self.declare_parameter('psi0', float(np.pi))
        self.declare_parameter('mirror_x_only', False)
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('frame_id', 'base')
        self.declare_parameter('line_width', 0.01)
        self.declare_parameter('tip_size', 0.03)

        self.L = np.array(self.get_parameter('L').value, float)
        self.tip_extra = float(self.get_parameter('tip_extra').value)
        self.r_wire = float(self.get_parameter('wire_radius').value)
        self.r_pulley = float(self.get_parameter('pulley_radius').value)
        self.motor_idx = list(self.get_parameter('motor_indices').value)
        self.psi0 = float(self.get_parameter('psi0').value)
        self.mirror_x_only = bool(self.get_parameter('mirror_x_only').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.line_width = float(self.get_parameter('line_width').value)
        self.tip_size = float(self.get_parameter('tip_size').value)

        # state
        self.zero_deg = None
        self.last_motor_deg = None
        self.last_centerline = None
        # publishers
        self.pub_line = self.create_publisher(Marker, '/pcc_centerline_marker', 10)
        self.pub_tip = self.create_publisher(Marker, '/pcc_tip_marker', 10)
        self.pub_meas_tip = self.create_publisher(Marker, '/pcc_meas_tip_marker', 10)
        # publish per-point spheres (MarkerArray) to match requested format
        self.declare_parameter('viz_frame', 'camera_color_optical_frame')
        self.declare_parameter('fixed_z', None)
        self.declare_parameter('sphere_scale', 0.01)
        self.viz_frame = self.get_parameter('viz_frame').value
        self.fixed_z = self.get_parameter('fixed_z').value
        self.sphere_scale = float(self.get_parameter('sphere_scale').value)
        self.pub_spheres = self.create_publisher(MarkerArray, '/pcc_centerline_spheres', 10)

        # subscribers
        self.sub_mot = self.create_subscription(Float32MultiArray, '/motor_current_angles', self.cb_motor, 10)
        self.sub_meas = self.create_subscription(PointStamped, '/meas_tip_on_base_plane', self.cb_meas_tip, 10)

        period = 1.0 / max(1e-6, float(self.get_parameter('publish_hz').value))
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info('PCCShapeVisualizer ready. Publish topics: /pcc_centerline_marker, /pcc_tip_marker, /pcc_meas_tip_marker')

    def cb_motor(self, msg: Float32MultiArray):
        arr = np.array(msg.data, float)
        self.last_motor_deg = arr
        if self.zero_deg is None:
            self.zero_deg = arr.copy()
            self.get_logger().info('Captured zero (deg) from /motor_current_angles for visualization.')

    def cb_meas_tip(self, msg: PointStamped):
        # publish measured tip as a sphere marker
        m = Marker()
        m.header = msg.header
        m.ns = 'pcc_meas_tip'
        m.id = 2
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = msg.point.x
        m.pose.position.y = msg.point.y
        m.pose.position.z = msg.point.z
        m.pose.orientation.w = 1.0
        m.scale.x = self.tip_size
        m.scale.y = self.tip_size
        m.scale.z = self.tip_size
        m.color.r = 0.0
        m.color.g = 0.0
        m.color.b = 1.0
        m.color.a = 0.9
        m.lifetime.sec = 0
        m.lifetime.nanosec = 0
        self.pub_meas_tip.publish(m)

    def on_timer(self):
        if self.last_motor_deg is None or self.zero_deg is None:
            return

        # compute theta and centerline
        try:
            theta = theta_from_motor_deg_constrained(self.last_motor_deg, self.zero_deg,
                                                     self.motor_idx, self.r_wire, self.r_pulley)
            X, Y = pcc_centerline_xy(theta, self.L, tip_extra=self.tip_extra, psi0=self.psi0)
        except Exception as e:
            self.get_logger().warn(f'Error computing centerline: {e}')
            return

        # mirror if requested
        if self.mirror_x_only and len(X) > 0:
            X = -X

    # publish LINE_STRIP marker for centerline
        line = Marker()
        line.header.frame_id = self.frame_id
        line.header.stamp = self.get_clock().now().to_msg()
        line.ns = 'pcc_centerline'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = self.line_width
        line.color.r = 1.0
        line.color.g = 0.5
        line.color.b = 0.0
        line.color.a = 0.9
        line.pose.orientation.w = 1.0
        line.points = []
        for xi, yi in zip(X, Y):
            p = Point()
            p.x = float(xi)
            p.y = float(yi)
            p.z = 0.0
            line.points.append(p)
        self.pub_line.publish(line)

        # publish predicted tip as a sphere
        tip = Marker()
        tip.header.frame_id = self.frame_id
        tip.header.stamp = line.header.stamp
        tip.ns = 'pcc_tip'
        tip.id = 1
        tip.type = Marker.SPHERE
        tip.action = Marker.ADD
        tip.pose.position.x = float(X[-1])
        tip.pose.position.y = float(Y[-1])
        tip.pose.position.z = 0.0
        tip.pose.orientation.w = 1.0
        tip.scale.x = self.tip_size
        tip.scale.y = self.tip_size
        tip.scale.z = self.tip_size
        tip.color.r = 1.0
        tip.color.g = 0.0
        tip.color.b = 0.0
        tip.color.a = 0.9
        self.pub_tip.publish(tip)

        # publish per-point SPHERE markers inside a MarkerArray (user requested format)
        m_arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for idx, (xi, yi) in enumerate(zip(X, Y)):
            m = Marker()
            m.header.frame_id = self.viz_frame
            m.header.stamp = stamp
            m.ns = 'pcc_viz'
            m.id = idx + 1  # ids start at 1 to avoid collision with line/tip ids
            # use SPHERE type (2) to match the provided example
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            # position: use provided fixed_z if set, else use 0.0
            z_val = 0.0
            if self.fixed_z is not None:
                try:
                    z_val = float(self.fixed_z)
                except Exception:
                    z_val = 0.0
            m.pose.position.x = float(xi)
            m.pose.position.y = float(yi)
            m.pose.position.z = float(z_val)
            m.pose.orientation.w = 1.0
            m.scale.x = self.sphere_scale
            m.scale.y = self.sphere_scale
            m.scale.z = self.sphere_scale
            m.color.r = 1.0
            m.color.g = 0.2
            m.color.b = 0.2
            m.color.a = 1.0
            m.lifetime.sec = 0
            m.lifetime.nanosec = 0
            m.frame_locked = False
            m_arr.markers.append(m)
        self.pub_spheres.publish(m_arr)


def main():
    rclpy.init()
    node = PCCShapeVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
