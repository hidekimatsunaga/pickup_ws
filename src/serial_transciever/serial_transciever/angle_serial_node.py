import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8MultiArray
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
        self.switch_pub = self.create_publisher(
            Int8MultiArray, 
            '/switch', 
            10
        )

        # Subscriber: 角度情報の受信
        qos = rclpy.qos.QoSProfile(depth=50, reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE)
        self.subscription = self.create_subscription(Float32MultiArray,
                                            '/motor_angles',
                                            self.listener_callback,
                                            qos
    )

        # タイマー：0.2秒ごとに現在角度を取得
        self.timer = self.create_timer(0.2, self.read_motor_angles)
        
        try:
            self.ser = serial.Serial('/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_D642773C5055344E312E3120FF0A251E-if00', 115200, timeout=1)
            self.get_logger().info('Serial connection established.')
        except serial.SerialException:
            self.get_logger().error('Failed to open serial port.')
            self.ser = None

    def listener_callback(self, msg: Float32MultiArray):
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
        
    def read_motor_angles(self):
        if not (self.ser and self.ser.is_open):
            self.get_logger().warn('Serial port not open')
            return
        try:
            # ---------- ① 角度 ----------
            self.ser.reset_input_buffer()          # 念のためバッファクリア
            self.ser.write(b'read_pos\n')
            angle_line = self.ser.readline().decode().strip()

            if angle_line:
                vals = [float(x) for x in angle_line.split(',') if x.strip()]
                if len(vals) == 9:
                    msg = Float32MultiArray(data=vals)
                    self.angle_pub.publish(msg)
                    self.get_logger().info(f'Current angles: {vals}')

            # ---------- ② スイッチ ----------
            self.ser.reset_input_buffer()
            self.ser.write(b'read_sw\n')
            sw_line = self.ser.readline().decode().strip()

            if sw_line:
                sw_vals = [int(x) for x in sw_line.split(',') if x.strip()]
                if len(sw_vals) == 9:
                    msg_sw = Int8MultiArray(data=sw_vals)
                    self.switch_pub.publish(msg_sw)
                    self.get_logger().debug(f'Switch states: {sw_vals}')

        except Exception as e:
            self.get_logger().error(f'Serial read error: {e}')


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