import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class MotorAnglePublisher(Node):
    def __init__(self):
        super().__init__('motor_angle_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/motor_angles', 10)
        self.get_logger().info('Press key [a, b, c, e, f, g, h, p] + Enter to publish preset angles.')
        self.key_to_angles = {
            "a": [257, 265, 190, 91, 16, 15, 70, 87, 36],
            "p": [334, 749, 1142, 335, 678, 1167, 668, 327, 201],
            "b": [168, 237, 178, 10, 16, -85, 42, 471, 527],
            "h": [168, 366, 852, 1, 185, 1062, 659, 846, 207],
            "c": [168, 237, 178, 10, 16, -85, 668, 471, 527],
            "e": [168, 237, 852, 1, 57, 1062, 659, 846, 207],
            "f": [342, 743, 1012, 327, 224, 1213, 659, 846, 207],
            "g": [342, 1051, 1017, 328, 389, 1062, 662, 849, 207]
        }
        self.run()

    def run(self):
        try:
            while rclpy.ok():
                key = input('Enter key: ').strip().lower()
                if key in self.key_to_angles:
                    msg = Float32MultiArray()
                    msg.data = [float(x) for x in self.key_to_angles[key]]  # ← 修正ポイント
                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Published angles for key "{key}": {msg.data}')
                else:
                    self.get_logger().warn(f'Invalid key "{key}". Try again.')
        except KeyboardInterrupt:
            self.get_logger().info('Publisher stopped by user.')

def main(args=None):
    rclpy.init(args=args)
    node = MotorAnglePublisher()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
