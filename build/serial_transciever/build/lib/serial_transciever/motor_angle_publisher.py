import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class MotorAnglePublisher(Node):
    def __init__(self):
        super().__init__('motor_angle_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/motor_angles', 10)
        self.get_logger().info('Press key [a, b, c, e, f, g, h, p] + Enter to publish preset angles.')
        self.key_to_angles = {
            'a': [227, 99, 109, 300, 231, 277, 60, 16, 309],
            'p': [304, 583, 1061, 544, 893, 1429, 658, 256, 474],
            'b': [138, 71, 97, 219, 231, 177, 32, 400, 800],
            'h': [138, 200, 771, 210, 400, 1324, 649, 775, 480],
            'c': [138, 71, 97, 219, 231, 177, 658, 400, 800],
            'e': [138, 71, 771, 210, 272, 1324, 649, 775, 480],
            'f': [312, 577, 931, 536, 439, 1475, 649, 775, 480],
            'g': [312, 885, 936, 537, 604, 1324, 652, 778, 480],
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
