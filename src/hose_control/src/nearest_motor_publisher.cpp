#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <limits>
#include <cmath>

struct DataPoint {
  double x, y, z;
  double motor1, motor2;
};

class NearestMotorPublisher : public rclcpp::Node {
public:
  NearestMotorPublisher()
  : Node("nearest_motor_publisher") {
    load_csv("motor_position_map.csv");

    sub_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/yolo_object_position", 10,
      std::bind(&NearestMotorPublisher::callback, this, std::placeholders::_1)
    );

    pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/motor_angle", 10);
  }

private:
  std::vector<DataPoint> dataset_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;

  void load_csv(const std::string &filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open CSV file: %s", filepath.c_str());
      return;
    }

    std::string line;
    // ヘッダーをスキップ
    std::getline(file, line);

    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string value;
      DataPoint dp;

      std::getline(ss, value, ','); dp.x = std::stod(value);
      std::getline(ss, value, ','); dp.y = std::stod(value);
      std::getline(ss, value, ','); dp.z = std::stod(value);
      std::getline(ss, value, ','); dp.motor1 = std::stod(value);
      std::getline(ss, value, ','); dp.motor2 = std::stod(value);

      dataset_.push_back(dp);
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu entries from CSV.", dataset_.size());
  }

  void callback(const geometry_msgs::msg::Point::SharedPtr msg) {
    if (dataset_.empty()) return;

    double min_dist = std::numeric_limits<double>::max();
    DataPoint closest;

    for (const auto &dp : dataset_) {
      double dist = std::sqrt(
        std::pow(dp.x - msg->x, 2) +
        std::pow(dp.y - msg->y, 2) +
        std::pow(dp.z - msg->z, 2));
      if (dist < min_dist) {
        min_dist = dist;
        closest = dp;
      }
    }

    std_msgs::msg::Float64MultiArray angle_msg;
    angle_msg.data = {closest.motor1, closest.motor2};
    pub_->publish(angle_msg);

    RCLCPP_INFO(this->get_logger(), "Published angles: [%f, %f]", closest.motor1, closest.motor2);
  }
};

int main(int argc, char **argv){
    rclcpp::init(argc,argv); ////ROS2通信を初期化
    auto node = std::make_shared<NearestMotorPublisher>(); //ノードを生成
    rclcpp::shutdown(); //ROS２通信をシャットダウン
    return 0;
}