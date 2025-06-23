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

         # Publisher: 現在角度の配信
        self.angle_pub = self.create_publisher(
            Float32MultiArray,
            '/motor_current_angles',
            10
        )

        # Subscriber: 角度情報の受信
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/motor_angles',
            self.listener_callback,
            10
        )

        # タイマー：0.2秒ごとに現在角度を取得
        self.timer = self.create_timer(0.2, self.read_motor_angles)
        
        try:
            self.ser = serial.Serial('/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_D642773C5055344E312E3120FF0A251E-if00', 115200, timeout=1)
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
        
    # def read_motor_angles(self):
    #     if self.ser and self.ser.is_open:
    #         try:# 現在の角度を取得するコマンドを送信
    #             self.ser.write(b'read\n')
    #             response = self.ser.readline().decode('utf-8').strip()
    #             if response:
    #                 values = [float(x) for x in response.split(',')]
    #                 if len(values) == 9:
    #                     msg = Float32MultiArray()
    #                     msg.data = values
    #                     self.angle_pub.publish(msg)
    #                     self.get_logger().info(f'Published current angles: {msg.data}')
    #                 else:
    #                     self.get_logger().warn(f'Response length mismatch: got {len(values)} values.')
    #             else:
    #                 self.get_logger().warn('No response from OpenRB.')
    #         except Exception as e:
    #             self.get_logger().error(f'Error reading from serial: {str(e)}')
    #     else:
    #         self.get_logger().warn('Serial port is not open.')
    def read_motor_angles(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b'read\n')
                lines = []
                while self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8').strip()
                    if line and not line.upper().startswith('OK') and not line.startswith('//'):
                        lines.append(line)

                for response in lines:
                    try:
                        values = [float(x) for x in response.split(',') if x.strip() != '']
                        if len(values) == 9:
                            self.current_angles = values
                            msg = Float32MultiArray()
                            msg.data = values
                            self.angle_pub.publish(msg)
                            self.get_logger().info(f'Current angles: {values}')
                            break  # 最初に正しく読めた行だけ処理
                    except ValueError:
                        continue  # float変換に失敗した行は無視
            except Exception as e:
                self.get_logger().error(f'Serial read error: {str(e)}')
        else:
            self.get_logger().warn('Serial port not open')


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