import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
import math
import csv
import os


class CmdVelTrajLogger(Node):
    def __init__(self):
        super().__init__('cmd_vel_traj_logger')

        # ---- params ----
        self.declare_parameter('omega_eps', 0.02)
        self.declare_parameter('v_eps', 0.001)
        self.declare_parameter('csv_path', 'cmdvel_traj.csv')
        self.declare_parameter('max_dt', 1.0)  # bag再生停止/飛びの保険

        self.omega_eps = float(self.get_parameter('omega_eps').value)
        self.v_eps = float(self.get_parameter('v_eps').value)
        self.csv_path = str(self.get_parameter('csv_path').value)
        self.max_dt = float(self.get_parameter('max_dt').value)

        # ---- state ----
        self.last_time = None
        self.latest_twist = Twist()

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.goal_xy = None
        self.center_xy = None

        # ---- subs ----
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cb_cmd, 10)
        self.sub_goal = self.create_subscription(PointStamped, '/traj/goal', self.cb_goal, 10)
        self.sub_center = self.create_subscription(PointStamped, '/traj/center', self.cb_center, 10)

        # ---- csv ----
        os.makedirs(os.path.dirname(self.csv_path) or ".", exist_ok=True)
        self.f = open(self.csv_path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow([
            't', 'dt',
            'vx_base', 'vy_base', 'omega',
            'x', 'y', 'yaw',
            'goal_x', 'goal_y',
            'center_x', 'center_y'
        ])
        self.f.flush()

        # timer (記録周期：cmd_velが来なくても0速度で更新はしない。来たら積分する方式)
        self.timer = self.create_timer(0.02, self.on_timer)  # 50Hz

        self.get_logger().info(f'Logging to {self.csv_path}')
        self.get_logger().info('Tip: ros2 bag play ... --clock && ros2 run ... --ros-args -p use_sim_time:=true')

    def cb_cmd(self, msg: Twist):
        self.latest_twist = msg

    def cb_goal(self, msg: PointStamped):
        self.goal_xy = (msg.point.x, msg.point.y)

    def cb_center(self, msg: PointStamped):
        self.center_xy = (msg.point.x, msg.point.y)

    def on_timer(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_time is None:
            self.last_time = now
            return

        dt = now - self.last_time
        self.last_time = now

        if dt <= 0.0 or dt > self.max_dt:
            return

        vbx = self.latest_twist.linear.x
        vby = self.latest_twist.linear.y
        omega = self.latest_twist.angular.z

        # small command deadzone
        if abs(vbx) < self.v_eps:
            vbx = 0.0
        if abs(vby) < self.v_eps:
            vby = 0.0
        if abs(omega) < self.omega_eps:
            omega = 0.0

        # 積分（base速度→worldへ回転）
        self.yaw += omega * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        vwx = c * vbx - s * vby
        vwy = s * vbx + c * vby

        self.x += vwx * dt
        self.y += vwy * dt

        gx, gy = (self.goal_xy if self.goal_xy is not None else (float('nan'), float('nan')))
        cx, cy = (self.center_xy if self.center_xy is not None else (float('nan'), float('nan')))

        self.w.writerow([now, dt, vbx, vby, omega, self.x, self.y, self.yaw, gx, gy, cx, cy])
        # bag再生だとバッファ溜め過ぎると怖いので適度にflush
        self.f.flush()

    def destroy_node(self):
        try:
            self.f.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = CmdVelTrajLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
