import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge  # ROSのImageメッセージをOpenCV形式に変換
import cv2  # OpenCVライブラリ
import os

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        self.bridge = CvBridge()
        self.image_saved = False  # 画像を保存したかどうかを追跡するフラグ

        # /aruco/image トピックを購読
        self.subscription = self.create_subscription(
            Image,
            '/aruco/image',
            self.image_callback,
            10) # QoSプロファイル
        
        self.get_logger().info('画像保存ノードを起動しました。/aruco/image トピックを待機中です...')

    def image_callback(self, msg):
        # すでに画像を保存済みの場合は、何もせずにリターン
        if self.image_saved:
            return

        self.get_logger().info('/aruco/image から画像を受信しました。')

        try:
            # ROSのImageメッセージをOpenCVの画像形式(bgr8)に変換
            # Arucoマーカーの画像は通常カラー(bgr8)かグレースケール(mono8)です
            # 必要に応じて "mono8" や "passthrough" に変更してください
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'cv_bridge 変換エラー: {e}')
            return

        # 保存するファイル名
        save_path = os.path.join(os.getcwd(), "saved_aruco_image.png")

        # OpenCVを使って画像をファイルとして保存
        try:
            cv2.imwrite(save_path, cv_image)
            self.get_logger().info(f'画像を {save_path} として保存しました。')
            self.image_saved = True  # 保存フラグを立てる

            # タスク完了のため、ノードをシャットダウン
            self.get_logger().info('画像の保存が完了したため、ノードをシャットダウンします。')
            self.destroy_node()
            rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'画像保存エラー: {e}')

def main(args=None):
    rclpy.init(args=args)
    image_saver_node = ImageSaver()
    
    try:
        rclpy.spin(image_saver_node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        # ノードが自身でシャットダウンした場合の例外処理
        pass
    finally:
        if rclpy.ok():
            image_saver_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()