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
#include "hose_control/motor_initial_position.hpp"
#include "hose_control/motor_pickup_position.hpp"

struct DataPoint {
  double x, y, z;
  std::vector<double> motors;  // motor1〜motor9
  double motor10;
};

class FeedbackMotorPublisher : public rclcpp::Node {
public:
  FeedbackMotorPublisher()
  : Node("feedback_motor_publisher"),
    air_value_(0.0),
    is_in_sequence_mode_(false), //シーケンス実行中かどうかのフラグ
    sequence_step_(0) //シーケンスの現在のステップ
  {
    this->declare_parameter<double>("air_threshold", -150.0);
    this->get_parameter("air_threshold", air_threshold_);

    load_csv("/home/matsunaga-h/pickup_ws/angle_arucopose_csv/aruco_motor_log_0718_183109_cleaned_file.csv");
    
    // ★ 修正点2: 型の違う2次元ベクトルの正しいコピー
    sequence_data_.clear();
    const auto& seq_from_header = motor_sequences::pickup_sequence;
    for (const auto& row_float : seq_from_header) {
      sequence_data_.emplace_back(row_float.begin(), row_float.end());
    }

    current_angles_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/current_motor_angles", 10,
      std::bind(&FeedbackMotorPublisher::currentAnglesCallback, this, std::placeholders::_1)
    );
    
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
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr current_angles_sub_;
  double air_value_;  
  bool is_in_sequence_mode_;
  int sequence_step_;
  std::vector<std::vector<double>> sequence_data_; // シーケンスデータを保持する変数
  std::vector<double> current_motor_angles_; // 最新のモーター角度を保持する変数
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
      dp.x = std::stod(tokens[12]);//csvの12行目
      dp.y = std::stod(tokens[13]);//csvの13行目
      dp.z = std::stod(tokens[14]);//csvの14行目

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
        // --- ★ここから修正 ---
    // シーケンス実行中は、新しい目標地点がきても無視する
    if (is_in_sequence_mode_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "In sequence mode, ignoring new goal point.");
      return;
    }

    // しきい値超過ならシーケンスを開始
    if (air_value_ <= air_threshold_) {
      RCLCPP_WARN(get_logger(), "Air sensor threshold exceeded! Starting sequence.");
      is_in_sequence_mode_ = true;
      sequence_step_ = 0;
      publishSequenceStep(); // シーケンスの最初のステップを実行
      return; // シーケンスモードに入ったので、これ以降の最近傍探索は行わない
    }
    const auto &pt = msg->point;
    double x = pt.x;
    double y = pt.y;
    double z = pt.z;

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
// ★追加：モーターの現在角度を受け取ったときのコールバック関数
  void currentAnglesCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    // ★ 修正点3: 型の違うベクターの正しい代入
    current_motor_angles_.assign(msg->data.begin(), msg->data.end());

    // シーケンスモードでなければ何もしない
    if (!is_in_sequence_mode_) {
      return;
    }

    // 目標の角度（シーケンスの現在のステップ）を取得
    const auto& target_angles = sequence_data_[sequence_step_];

    // 現在の角度が目標に到達したかチェック
    if (isCloseToTarget(target_angles, current_motor_angles_)) {
      RCLCPP_INFO(this->get_logger(), "Sequence step %d reached.", sequence_step_);

      // 次のステップに進める
      sequence_step_++;

      // もし全ステップが完了したら、シーケンスモードを終了
      if (static_cast<size_t>(sequence_step_) >= sequence_data_.size()) {
        RCLCPP_INFO(this->get_logger(), "Sequence finished.");
        is_in_sequence_mode_ = false;
        sequence_step_ = 0;
        return;
      }

      // 次の目標角度をパブリッシュ
      publishSequenceStep();
    }
  }

  // ★追加：2つの角度リストが「ほぼ同じ」か判定するヘルパー関数
  bool isCloseToTarget(const std::vector<double>& target, const std::vector<double>& current) {
    if (target.size() != current.size()) {
      return false;
    }
    double tolerance = 1.5; // 許容誤差（例: 1.5度）。モーターの性能に合わせて調整。
    for (size_t i = 0; i < target.size(); ++i) {
      if (std::abs(target[i] - current[i]) > tolerance) {
        return false; // 1つでも許容誤差を超えていたらfalse
      }
    }
    return true; // 全て許容誤差内ならtrue
  }

  // ★追加：シーケンスの現在のステップの角度をパブリッシュする関数
  void publishSequenceStep() {
    RCLCPP_INFO(this->get_logger(), "Publishing sequence step %d.", sequence_step_);
    std_msgs::msg::Float32MultiArray angle_msg;
    const auto& target_angles = sequence_data_[sequence_step_];
    for(const auto& angle : target_angles){
      angle_msg.data.push_back(static_cast<float>(angle));
    }
    pub_->publish(angle_msg);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FeedbackMotorPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
