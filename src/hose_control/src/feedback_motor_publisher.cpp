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

class FeedbackMotorPublisher : public rclcpp::Node {
public:
  FeedbackMotorPublisher()
  : Node("feedback_motor_publisher"),
    air_value_(0.0)
  {
    this->declare_parameter<double>("air_threshold", -150.0);
    this->get_parameter("air_threshold", air_threshold_);

    load_csv("/home/matsunaga-h/pickup_ws/angle_arucopose_csv/aruco_motor_log_0714_001835.csv");

    sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point", 10,
      std::bind(&FeedbackMotorPublisher::callback, this, std::placeholders::_1)
    );

    air_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/sensor/pressure", 10,
      std::bind(&FeedbackMotorPublisher::airCallback, this, std::placeholders::_1));

    auto qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
    pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/motor_angles", qos);
    pub_motor10_ = this->create_publisher<std_msgs::msg::Float32>("/chokudomotor/target_angle", 10);
  }

private:
  std::vector<DataPoint> dataset_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_motor10_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr air_sub_;

  double air_value_;
  double air_threshold_;


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
  void airCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    air_value_ = msg->data;
  }

  void callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    const auto &pt = msg->point;
    double x = pt.x;
    double y = pt.y;
    double z = pt.z;
    /* しきい値超過なら停止 */
    if (air_value_ <= air_threshold_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Air sensor %.3f exceeds threshold %.3f → STOP",
                           air_value_, air_threshold_);

      std_msgs::msg::Float32MultiArray zero_msg;
      zero_msg.data.assign(9, 0.0f);
      pub_->publish(zero_msg);

      std_msgs::msg::Float32 zero10;
      zero10.data = 0.0f;
      pub_motor10_->publish(zero10);
      return;  // ここで処理終了
    }

    if (dataset_.empty()) return;

    // --- ここから置き換え ---
    double min_dist   = std::numeric_limits<double>::max();
    const DataPoint* closest = nullptr;
    int   final_index = -1;                     // 最終的に選ばれた行番号

    for (size_t i = 0; i < dataset_.size(); ++i) {
      const auto &dp = dataset_[i];
      double dist = std::sqrt(
        std::pow(dp.x - x, 2) +
        std::pow(dp.y - y, 2) +
        std::pow(dp.z - z, 2)
      );
      if (dist < min_dist) {
        min_dist   = dist;
        closest    = &dp;
        final_index = static_cast<int>(i);      // 行番号を記録
      }
    }

    if (closest == nullptr) return;

    // 検索が終わったあと 1 回だけ出力
    RCLCPP_INFO(this->get_logger(), "Selected row index: %d", final_index);
    
    std_msgs::msg::Float32MultiArray angle_msg;
    angle_msg.data.reserve(closest->motors.size());
    for (const auto &val : closest->motors) {
      angle_msg.data.push_back(static_cast<float>(val));
    }
    pub_->publish(angle_msg);       
    
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
  auto node = std::make_shared<FeedbackMotorPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
