#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <limits>
#include <cmath>
#include <iomanip>
#include "hose_control/motor_initial_position.hpp"
#include "hose_control/motor_pickup_position.hpp"
#include <algorithm> // std::sort のために追加
#include <vector>    // std::vector のために追加
#include <visualization_msgs/msg/marker_array.hpp>

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
    // Node parameters (can be overridden via YAML or command-line)
    this->declare_parameter<std::string>("csv_filepath", "/home/matsunaga-h/pickup_ws/angle_arucopose_csv_cleaned/aruco_motor_log_1202_related_cleaned_deduped.csv");
    this->get_parameter("csv_filepath", csv_filepath_);

    this->declare_parameter<double>("air_threshold", -110.0);
    this->get_parameter("air_threshold", air_threshold_);

    this->declare_parameter<int>("air_threshold_hits_required", 3);
    this->get_parameter("air_threshold_hits_required", air_threshold_hits_required_);

    this->declare_parameter<int>("k_neighbors", 4);
    this->get_parameter("k_neighbors", k_neighbors_);

    this->declare_parameter<double>("epsilon", 1e-9);
    this->get_parameter("epsilon", epsilon_);

    this->declare_parameter<double>("pickup_motor10_angle", 54.0);
    this->get_parameter("pickup_motor10_angle", pickup_motor10_angle_);

    this->declare_parameter<double>("neighbor_marker_scale", 0.02);
    this->get_parameter("neighbor_marker_scale", neighbor_marker_scale_);

    this->declare_parameter<double>("marker_lifetime_sec", 1.0);
    this->get_parameter("marker_lifetime_sec", marker_lifetime_sec_);

    this->declare_parameter<double>("tolerance", 20.0);
    this->get_parameter("tolerance", tolerance_);

    // load CSV from parameter
    load_csv(csv_filepath_);
    // ★ 修正点2: 型の違う2次元ベクトルの正しいコピー
    sequence_data_.clear();
    const auto& seq_from_header = motor_sequences::pickup_sequence;
    for (const auto& row_float : seq_from_header) {
      sequence_data_.emplace_back(row_float.begin(), row_float.end());
    }

    current_angles_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/motor_current_angles", 10,
      std::bind(&FeedbackMotorPublisher::currentAnglesCallback, this, std::placeholders::_1)
    );
    
    sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/hose/goal_point", 10,
      std::bind(&FeedbackMotorPublisher::callback, this, std::placeholders::_1)
    );

    air_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/sensor/pressure", 10,
      std::bind(&FeedbackMotorPublisher::airCallback, this, std::placeholders::_1));

    auto qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
    pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/motor_angles", qos);
    pub_motor10_ = this->create_publisher<std_msgs::msg::Float32>("/chokudomotor/target_angle", 10);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/hose/neighbor_points", 10); // ★追加

    state_pub_ = this->create_publisher<std_msgs::msg::String>("/robot/state", 10);
    hose_result_pub_ = this->create_publisher<std_msgs::msg::Bool>("/hose/result", 10);

private:
  std::vector<DataPoint> dataset_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_motor10_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr air_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr current_angles_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_; // ★追加
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hose_result_pub_;

  double air_value_;  
  bool is_in_sequence_mode_;
  int sequence_step_;
  std::vector<std::vector<double>> sequence_data_; // シーケンスデータを保持する変数
  std::vector<double> current_motor_angles_; // 最新のモーター角度を保持する変数
  double air_threshold_;
  int air_threshold_hits_required_ = 3;
  int air_threshold_hits_counter_ = 0;

  // Parameters (populated from node parameters)
  std::string csv_filepath_;
  int k_neighbors_ = 3;
  double epsilon_ = 1e-9;
  double pickup_motor10_angle_ = 54.0;
  double neighbor_marker_scale_ = 0.02;
  double marker_lifetime_sec_ = 1.0;
  double tolerance_ = 20.0;


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

    // Count consecutive threshold hits to avoid accidental spikes triggering the sequence.
    if (air_value_ <= air_threshold_) {
      air_threshold_hits_counter_ = std::min(air_threshold_hits_counter_ + 1, air_threshold_hits_required_);
    } else {
      air_threshold_hits_counter_ = 0;
    }
  }

  void callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        // --- ★ここから修正 ---
    // シーケンス実行中は、新しい目標地点がきても無視する
    if (is_in_sequence_mode_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "In sequence mode, ignoring new goal point.");
      return;
    }

    // しきい値超過ならシーケンスを開始
    if (air_threshold_hits_counter_ >= air_threshold_hits_required_) {
      RCLCPP_WARN(get_logger(), "Air sensor threshold exceeded %d times. Starting sequence.", air_threshold_hits_counter_);
      air_threshold_hits_counter_ = 0; // reset so the next run needs fresh hits
      is_in_sequence_mode_ = true;
      sequence_step_ = 0;

      // シーケンス開始時にmotor10の値を一度だけパブリッシュ
      std_msgs::msg::Float32 motor10_msg;
      // ゴミを吸着・保持するための角度を設定します。
      // この値はノードパラメータ `pickup_motor10_angle` から読み込みます。
      motor10_msg.data = static_cast<float>(pickup_motor10_angle_);
      pub_motor10_->publish(motor10_msg);
      RCLCPP_INFO(this->get_logger(), "Published motor10 for pickup sequence: %.2f", motor10_msg.data);
 
      publishSequenceStep(); // シーケンスの最初のステップを実行
      return; // シーケンスモードに入ったので、これ以降の最近傍探索は行わない
    }
    const auto &pt = msg->point;
    double x = pt.x;
    double y = pt.y;
    double z = pt.z;

    if (dataset_.empty()) return;

    // --- ここから置き換え ---

    // パラメータから取得する値を使う
    int k_neighbors = k_neighbors_;
    // パラメータ: ゼロ除算を避けるための微小値
    double epsilon = epsilon_;

    if (dataset_.size() < static_cast<size_t>(k_neighbors)) {
      RCLCPP_WARN(this->get_logger(), "Not enough data in CSV to perform interpolation.");
      return;
    }

    // 1. 全てのデータ点と目標地点との距離を計算
    std::vector<std::pair<double, const DataPoint*>> distances;
    for (const auto& dp : dataset_) {
    double dist = std::sqrt(
        std::pow(dp.x - x, 2) +
        std::pow(dp.y - y, 2) +
        std::pow(dp.z - z, 2)
    );
    distances.push_back({dist, &dp});
    }

    // 2. 距離が近い順にソート
    std::sort(distances.begin(), distances.end());

    // 3. 最も近い k 個の点（近傍点）を取得
    std::vector<std::pair<double, const DataPoint*>> neighbors(
    distances.begin(),
    distances.begin() + k_neighbors
    );

    // もし最も近い点との距離がほぼゼロなら、その点の値をそのまま使う
    if (neighbors[0].first < epsilon) {
    const DataPoint* closest = neighbors[0].second;
    
    // (元のコードと同じpublish処理)
    std_msgs::msg::Float32MultiArray angle_msg;
    angle_msg.data.assign(closest->motors.begin(), closest->motors.end());
    pub_->publish(angle_msg);
    
    std_msgs::msg::Float32 motor10_msg;
    motor10_msg.data = closest->motor10;
    pub_motor10_->publish(motor10_msg);
    
    RCLCPP_INFO(this->get_logger(), "Target point is very close to a data point. Using it directly.");
    return;
    }

    // 4. 重みの計算と各モーター角度の補間
    double total_weight = 0.0;
    std::vector<double> interpolated_motors(9, 0.0);
    double interpolated_motor10 = 0.0;
    std::vector<double> weights;

    for (const auto& neighbor : neighbors) {
    double weight = 1.0 / neighbor.first; // 距離の逆数を重みとする
    weights.push_back(weight);
    total_weight += weight;
    }

    for (size_t i = 0; i < neighbors.size(); ++i) {
    const auto& dp = *neighbors[i].second;
    const double normalized_weight = weights[i] / total_weight; // 重みの正規化

    for (size_t j = 0; j < dp.motors.size(); ++j) {
        interpolated_motors[j] += dp.motors[j] * normalized_weight;
    }
    interpolated_motor10 += dp.motor10 * normalized_weight;
    }
    // ★ここからマーカー発行処理を追加
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;
    for (const auto& neighbor : neighbors) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = msg->header.frame_id; // goal_pointと同じ座標系
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "neighbor_points";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE; // 球体マーカー
        marker.action = visualization_msgs::msg::Marker::ADD;

        // マーカーの位置を近傍点の座標に設定
        const auto& dp = *neighbor.second;
        marker.pose.position.x = dp.x;
        marker.pose.position.y = dp.y;
        marker.pose.position.z = dp.z;
        marker.pose.orientation.w = 1.0;

        // マーカーのサイズ (パラメータ化)
        marker.scale.x = static_cast<float>(neighbor_marker_scale_);
        marker.scale.y = static_cast<float>(neighbor_marker_scale_);
        marker.scale.z = static_cast<float>(neighbor_marker_scale_);

        // マーカーの色 (赤色)
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0; // 不透明

        // マーカーが自動で消えるまでの時間 (パラメータ化)
        {
          int sec = static_cast<int>(marker_lifetime_sec_);
          int nsec = static_cast<int>((marker_lifetime_sec_ - sec) * 1e9);
          marker.lifetime = rclcpp::Duration(sec, nsec);
        }

        marker_array.markers.push_back(marker);
    }
    marker_pub_->publish(marker_array);
    // 5. 補間した結果をパブリッシュ
    std_msgs::msg::Float32MultiArray angle_msg;
    angle_msg.data.assign(interpolated_motors.begin(), interpolated_motors.end());
    pub_->publish(angle_msg);

    std_msgs::msg::Float32 motor10_msg;
    motor10_msg.data = interpolated_motor10;
    pub_motor10_->publish(motor10_msg);

    std::stringstream ss;
    for (auto a : interpolated_motors) ss << std::fixed << std::setprecision(2) << a << " ";
    RCLCPP_INFO(this->get_logger(), "Published interpolated motor1-9: %s, motor10: %.2f", ss.str().c_str(), interpolated_motor10);
}
// ★追加：モーターの現在角度を受け取ったときのコールバック関数
  // ★修正：モーターの現在角度を受け取ったときのコールバック関数
  void currentAnglesCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    current_motor_angles_.assign(msg->data.begin(), msg->data.end());

    // シーケンスモードでなければ何もしない
    if (!is_in_sequence_mode_) {
      return;
    }

    // ★修正：目標の角度（シーケンスの現在のステップ）を取得
    const auto& target_data = sequence_data_[sequence_step_]; // 10要素すべてを取得
    
    if (target_data.size() != 10) {
        RCLCPP_ERROR_ONCE(this->get_logger(), "Sequence data is invalid (not 10 elements). Halting sequence check.");
        return;
    }

    // ★修正：motor1-9 の 9要素だけを比較用のベクターにコピー
    std::vector<double> target_angles_9(target_data.begin(), target_data.begin() + 9);

    // (デバッグログも必要に応じて修正してください)
    // ...

    // ★修正：9要素の目標角度と、現在の角度（current_motor_angles_）を比較
    if (isCloseToTarget(target_angles_9, current_motor_angles_)) {
      RCLCPP_INFO(this->get_logger(), "Sequence step %d reached.", sequence_step_);

      // 次のステップに進める
      sequence_step_++;

      // もし全ステップが完了したら、シーケンスモードを終了
      if (static_cast<size_t>(sequence_step_) >= sequence_data_.size()) {
        RCLCPP_INFO(this->get_logger(), "Sequence finished.");
        is_in_sequence_mode_ = false;
        sequence_step_ = 0;
        
        // ★追加：シーケンス完了時に状態とホース結果を発行
        std_msgs::msg::String state_msg;
        state_msg.data = "collecting_finished";
        state_pub_->publish(state_msg);
        RCLCPP_INFO(this->get_logger(), "Published /robot/state: collecting_finished");
        
        std_msgs::msg::Bool result_msg;
        result_msg.data = true;  // 回収成功
        hose_result_pub_->publish(result_msg);
        RCLCPP_INFO(this->get_logger(), "Published /hose/result: true (collection success)");
        
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
    double tolerance = tolerance_; // 許容誤差（度）。ノードパラメータで調整可能。
    for (size_t i = 0; i < target.size(); ++i) {
      if (std::abs(target[i] - current[i]) > tolerance) {
        return false; // 1つでも許容誤差を超えていたらfalse
      }
    }
    return true; // 全て許容誤差内ならtrue
  }

// ★修正：シーケンスの現在のステップの角度をパブリッシュする関数
  void publishSequenceStep() {
    RCLCPP_INFO(this->get_logger(), "Publishing sequence step %d.", sequence_step_);
    
    const auto& target_data = sequence_data_[sequence_step_]; // 10要素のデータを取得
    
    // ★追加：データの個数をチェック
    if (target_data.size() != 10) {
        RCLCPP_ERROR(this->get_logger(), "Sequence data for step %d has %zu elements, expected 10.", 
                     sequence_step_, target_data.size());
        return;
    }

    // ★修正：最初の9要素を /motor_angles (motor1-9) にパブリッシュ
    std_msgs::msg::Float32MultiArray angle_msg;
    angle_msg.data.assign(target_data.begin(), target_data.begin() + 9);
    pub_->publish(angle_msg);

    // ★修正：最後の1要素 (10番目) を /chokudomotor/target_angle (motor10) にパブリッシュ
    std_msgs::msg::Float32 motor10_msg;
    motor10_msg.data = static_cast<float>(target_data[9]); // 10番目の要素 (インデックスは9)
    pub_motor10_->publish(motor10_msg);
    
    RCLCPP_INFO(this->get_logger(), "Published seq motors 1-9 and motor10: %.2f", motor10_msg.data);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FeedbackMotorPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
