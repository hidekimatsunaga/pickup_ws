# GPUを使わないように変更したコード
import rclpy
from rclpy.node import Node
# from keyboard_msgs.msg import Key
from std_msgs.msg import Int32, String, Int32MultiArray
import torch
import cv2
import time
import numpy as np
from ultralytics import YOLO

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('yolodetection_node')
        self.x = False
        self.y = False
        self.z = False
        self.start_time = 0
        # self.publisher_ = self.create_publisher(Key, 'keydown', 10)
        self.label_publisher_ = self.create_publisher(Int32, 'label_topic', 10)
        self.blue_box_center_publisher = self.create_publisher(Int32MultiArray, 'where_is_blue_box', 10)
        self.subscription = self.create_subscription(
            Int32,
            '/waypoint/reach_stop_waypoint_id',
            self.detect_stopline_callback,
            10
        )

        self.signal_counter = 0
        self.signal_confirmed = False
        self.confirmed_signal_bb = None

    def detect_stopline_callback(self, msg):
        print(f"stopline received. ID : {msg.data}")
        if(msg.data in [4, 6, 8, 10]):
            self.x = True
        else:
            self.x = False
            self.y = False
            self.z = False

    # def publish_keydown(self, key_code, modifiers):
    #     msg = Key()
    #     msg.code = key_code
    #     msg.modifiers = modifiers
    #     msg.header.stamp = self.get_clock().now().to_msg()
    #     self.publisher_.publish(msg)
    #     self.get_logger().info(f'KeyDown published: code={key_code}, modifiers={modifiers}')

    def publish_label(self, content):
        msg = Int32()
        msg.data = content
        self.label_publisher_.publish(msg)
        self.get_logger().info(f'label published: "{content}"')

    def publish_blue_box_center(self, center_x, center_y):
        msg = Int32MultiArray()
        msg.data = [int(center_x), int(center_y)]
        self.blue_box_center_publisher.publish(msg)
        self.get_logger().info(f'Blue-Box center published: x={center_x}, y={center_y}')

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    print(f"pytorch setting up...")
    torch.backends.cudnn.benchmark = False  # GPU使わないのでFalse

    print(f"model setting up...")
    # GPU使用なし：.to('cuda:0') を削除
    sig_car_model = YOLO('model/241206_signal_epoch225_batch32.pt')

    print(f"video setting up...")
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    frame_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    frame_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    center_x = frame_width / 2
    center_y = frame_height / 2
    central_margin = 100
    size_threshold = 0.02
    cam_setup = 0
    scale_factor = 0.5
    label_detect = False
    red_counter = 0
    blue_counter = 0
    a_frame_counter = 0
    b_frame_counter = 0
    c_frame_counter = 0

    print(f"cam loop start")
    while (rclpy.ok() and cap.isOpened()):
        ret, frame = cap.read()
        if not ret:
            break

        executor.spin_once(timeout_sec=0.01)

        if(node.x or cam_setup < 5):
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            start_time = time.time()
            results = sig_car_model(img)
            end_time = time.time()
            signal_color = None
            text_color = None
            print(f"shape0: {frame.shape[0]}, shape1: {frame.shape[1]}")
            print(f"Inference time: {end_time - start_time:.4f} seconds")
            annotated_frame = results[0].plot()
            frame = cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR)
            skip_signal_detection = False
            detected_red_conf = 0.0
            detected_blue_conf = 0.0
            detected_red = False
            detected_blue = False

            for detection in results[0].boxes:
                class_id = int(detection.cls[0].item())
                class_name = sig_car_model.names[class_id]
                if class_name == "car":
                    box = detection.xyxy[0].cpu().numpy().astype(int)
                    x1, y1, x2, y2 = box
                    box_center_x = (x1 + x2) / 2
                    box_center_y = (y1 + y2) / 2
                    box_width = x2 - x1
                    box_height = y2 - y1
                    box_area = (box_width * box_height) / (frame_width * frame_height)
                    if abs(box_center_x - center_x) < central_margin and abs(box_center_y - center_y) < central_margin and box_area > size_threshold:
                        skip_signal_detection = True
                        signal_color = 'No Going'
                        text_color = (0, 0, 255)
                        break

            for detection in results[0].boxes:
                class_id = int(detection.cls[0].item())
                class_name = sig_car_model.names[class_id]
                confidence = detection.conf[0].item()
                if class_name == "signal_red":
                    detected_red = True
                    detected_red_conf = confidence
                    red_counter += 1
                    blue_counter = 0
                elif class_name == "signal_blue":
                    detected_blue = True
                    detected_blue_conf = confidence
                    red_counter = 0
                    blue_counter += 1

            print(blue_counter)
            if detected_blue:
                if blue_counter >= 3 or (detected_red and detected_blue_conf > detected_red_conf):
                    if not skip_signal_detection:
                        # node.publish_keydown(107, 4096)
                        node.x = False
                        signal_color = 'Lets go ruby!'
                        text_color = (0, 255, 0)
                        print(f"'Trafficlight_blue' detected with confidence {detected_blue_conf}, published keydown.")
                else:
                    if not skip_signal_detection:
                        print(f"'Trafficlight_red' detected with confidence {detected_red_conf}, published keydown.")
            else:
                if not skip_signal_detection:
                    print(f"'Trafficlight_red' detected with confidence {detected_red_conf}, published keydown.")

            cv2.putText(frame, signal_color, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, text_color, 2, cv2.LINE_AA)
            small_frame = cv2.resize(frame, (0, 0), fx=scale_factor, fy=scale_factor)
            cv2.imshow('YOLOv8 Detection', small_frame)

        cam_setup += 1

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
