#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>


#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <limits>
#include <cmath>

struct DataPoint {
  double x, y, z;
  std::vector<double> motors;  // motor1〜motor9
  double motor10;
};

class NearestMotorPublisher : public rclcpp::Node {
public:
  NearestMotorPublisher()
  : Node("nearest_motor_publisher") {
    load_csv("/home/matsunaga-h/pickup_ws/angle_arucopose_csv/aruco_motor_log_0710_185439_cleaned_file.csv");

    sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/detected_depth_points", 10,
      std::bind(&NearestMotorPublisher::callback, this, std::placeholders::_1)
    );

    pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/motor_angle", 10);
    pub_motor10_ = this->create_publisher<std_msgs::msg::Float32>("/chokudomotor/target_angle", 10);
  }

private:
  std::vector<DataPoint> dataset_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_motor10_;

  void load_csv(const std::string &filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open CSV file: %s", filepath.c_str());
      return;
    }

    std::string line;
    std::getline(file, line);  // skip header

    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string value;
      std::vector<std::string> tokens;

      while (std::getline(ss, value, ',')) {
        tokens.push_back(value);
      }

      if (tokens.size() < 19) {
        RCLCPP_WARN(this->get_logger(), "Invalid CSV row: %s", line.c_str());
        continue;
      }

      DataPoint dp;
      dp.x = std::stod(tokens[12]);
      dp.y = std::stod(tokens[13]);
      dp.z = std::stod(tokens[14]);

      dp.motors.reserve(9);
      for (int i = 1; i <= 9; ++i) {
        dp.motors.push_back(std::stod(tokens[i]));
      }

      dp.motor10 = std::stod(tokens[10]);

      dataset_.push_back(dp);
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu entries from CSV.", dataset_.size());
  }

  void callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    const auto &pt = msg->point;
    double x = pt.x;
    double y = pt.y;
    double z = pt.z;

    if (dataset_.empty()) return;

    double min_dist = std::numeric_limits<double>::max();
    const DataPoint* closest = nullptr;

    for (const auto &dp : dataset_) {
      double dist = std::sqrt(
        std::pow(dp.x - x, 2) +
        std::pow(dp.y - y, 2) +
        std::pow(dp.z - z, 2)
      );
      if (dist < min_dist) {
        min_dist = dist;
        closest = &dp;
        int index = std::distance(dataset_.begin(), std::find_if(dataset_.begin(), dataset_.end(),
          [&](const DataPoint& d) { return &d == closest; }));
        RCLCPP_INFO(this->get_logger(), "Nearest row index: %d", index);
      }
    }

    if (closest == nullptr) return;

    std_msgs::msg::Float32MultiArray angle_msg;
    angle_msg.data.reserve(closest->motors.size());
    for (const auto &val : closest->motors) {
      angle_msg.data.push_back(static_cast<float>(val));
    }
    std_msgs::msg::Float32 motor10_msg;
    motor10_msg.data = closest->motor10;
    pub_motor10_->publish(motor10_msg);

    std::stringstream ss;
    for (auto a : closest->motors) ss << a << " ";
    RCLCPP_INFO(this->get_logger(), "Published motor1-9: %s, motor10: %.2f", ss.str().c_str(), closest->motor10);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NearestMotorPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
