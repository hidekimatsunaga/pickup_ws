#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

using namespace std::chrono_literals;

class SimpleInteractiveMarker : public rclcpp::Node
{
public:
  SimpleInteractiveMarker()
  : Node("simple_interactive_marker")
  {
    // RVizからの目標点を配信するためのPublisher
    goal_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/hose/goal_point", 10);

    // インタラクティブマーカーのサーバーを初期化
    // "simple_marker" はサーバーの名前。RVizの表示名などにも使われる
    // 変更後
    // QoSプロファイルを指定してサーバーを作成
    // 変更後
    auto qos = rclcpp::QoS(1).transient_local();
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("simple_marker", this, qos);
    // マーカーの作成
    make_marker();

    // サーバーに変更を適用して、RVizにマーカーを表示させる
    server_->applyChanges();
  }

private:
  // マーカーを動かしたときに呼び出されるコールバック関数
  void processFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback)
  {
    // マーカーの現在の座標を取得
    const auto& point = feedback->pose.position;

    // ログに座標を出力
    RCLCPP_INFO(this->get_logger(), "Marker moved to: x=%.2f, y=%.2f, z=%.2f",
                point.x, point.y, point.z);

    // geometry_msgs::msg::PointStamped 型のメッセージを作成
    auto msg = std::make_unique<geometry_msgs::msg::PointStamped>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "camera_color_optical_frame"; // ★★★ 基準となる座標系を指定してください (例: "map", "odom", "base_link")
    msg->point = point;

    // /hose/goal_point トピックに配信
    goal_point_pub_->publish(std::move(msg));
  }

  // マーカーの形状や動作を定義する関数
  void make_marker()
  {
    // 1. マーカー全体の定義
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "camera_color_optical_frame"; // ★★★ 基準となる座標系を指定
    int_marker.header.stamp = this->now();
    int_marker.name = "goal_marker";
    int_marker.description = "Hose Goal Point";
    int_marker.scale = 1.0; // マーカー全体のスケール

    // 2. 表示されるオブジェクト（今回は赤い球）の定義
    visualization_msgs::msg::Marker marker;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.scale.x = 0.1; // 球の直径
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 1.0; // 赤色
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0; // 不透明

    // 3. マーカーの操作方法（コントロール）の定義
    visualization_msgs::msg::InteractiveMarkerControl control;
    control.orientation.w = 1; // クォータニオンの初期化
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS; // X軸方向に移動
    int_marker.controls.push_back(control);

    control.name = "move_y";
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS; // Y軸方向に移動
    int_marker.controls.push_back(control);

    control.name = "move_z";
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS; // Z軸方向に移動
    int_marker.controls.push_back(control);

    // 表示オブジェクトをコントロールに追加
    // これにより、赤い球がコントロール（操作用の矢印など）に含まれる
    int_marker.controls[0].markers.push_back(marker);
    int_marker.controls[0].always_visible = true;

    // 作成したマーカーをサーバーに追加
    // "processFeedback" は、このマーカーが操作されたときに呼び出すコールバック関数
    server_->insert(int_marker, std::bind(&SimpleInteractiveMarker::processFeedback, this, std::placeholders::_1));
  }

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_pub_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleInteractiveMarker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}