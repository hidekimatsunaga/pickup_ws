# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray
# import serial

# class AngleSerialNode(Node):
#     def __init__(self):
#         super().__init__('angle_serial_node')
#         self.subscription = self.create_subscription(
#             Float32MultiArray,
#             '/motor_angles',
#             self.listener_callback,
#             10)
        
#         try:
#             self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
#             self.get_logger().info('Serial connection established.')
#         except serial.SerialException:
#             self.get_logger().error('Failed to open serial port.')
#             self.ser = None

#     def listener_callback(self, msg):
#         if self.ser and self.ser.is_open:
#             if len(msg.data) != 9:
#                 self.get_logger().warn('Received data is not 9 elements.')
#                 return

#             data_str = ','.join([f'{angle:.2f}' for angle in msg.data]) + '\n'
#             self.ser.write(data_str.encode('utf-8'))
#             self.ser.flush()
#             self.get_logger().info(f'Sent: {data_str.strip()}')

#             response = self.ser.readline().decode('utf-8').strip()
#             if response.startswith("OK;POS:"):
#                 angles_str = response[7:]
#                 try:
#                     angles = [float(x) for x in angles_str.split(',')]
#                     self.get_logger().info(f'Current angles: {angles}')
#                 except ValueError:
#                     self.get_logger().warn(f'Failed to parse angles from response: {response}')
#             elif response:
#                 self.get_logger().info(f'Response from OpenRB: {response}')
#             else:
#                 self.get_logger().warn('No response from OpenRB.')
#         else:
#             self.get_logger().warn('Serial port is not open.')

# def main(args=None):
#     rclpy.init(args=args)
#     node = AngleSerialNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         if node.ser and node.ser.is_open:
#             node.ser.close()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

class AngleSerialNode(Node):
    def __init__(self):
        super().__init__('angle_serial_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/motor_angles',
            self.listener_callback,
            10)
        
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info('Serial connection established.')
        except serial.SerialException:
            self.get_logger().error('Failed to open serial port.')
            self.ser = None

    def listener_callback(self, msg):
        if self.ser and self.ser.is_open:
            if len(msg.data) != 9:
                self.get_logger().warn('Received data is not 9 elements.')
                return

            # 9個の角度情報をカンマ区切りで送信（末尾に改行）
            data_str = ','.join([f'{angle:.2f}' for angle in msg.data]) + '\n'
            self.ser.write(data_str.encode('utf-8'))
            self.get_logger().info(f'Sent: {data_str.strip()}')
        else:
            self.get_logger().warn('Serial port is not open.')
        response = self.ser.readline().decode('utf-8').strip()
        if response:
            self.get_logger().info(f'Response from OpenRB: {response}')
        else:
            self.get_logger().warn('No response from OpenRB.')

        self.get_logger().info(f'Sent: {data_str.strip()}')


def main(args=None):
    rclpy.init(args=args)
    node = AngleSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()