import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial

class ArduinoReader(Node):
    def __init__(self):
        super().__init__('arduino_reader')
        self.publisher_ = self.create_publisher(Int32, 'pin_state', 10)
        self.serial = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.timer = self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        if self.serial.in_waiting > 0:
            line = self.serial.readline().decode('utf-8').strip()
            if line.isdigit():
                msg = Int32()
                msg.data = int(line)
                self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
