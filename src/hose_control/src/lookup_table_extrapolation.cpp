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
    this->declare_parameter<double>("air_threshold", -110.0);
    this->get_parameter("air_threshold", air_threshold_);

    load_csv("/home/matsunaga-h/pickup_ws/angle_arucopose_csv/aruco_motor_log_0928_161517.csv");
    
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

  }

private:
  std::vector<DataPoint> dataset_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_motor10_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr air_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr current_angles_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_; // ★追加

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

      // シーケンス開始時にmotor10の値を一度だけパブリッシュ
      std_msgs::msg::Float32 motor10_msg;
      // ゴミを吸着・保持するための角度を設定します。
      // この値は、実際のロボットの動作に合わせて調整してください。
      motor10_msg.data = 54.0; // 例: 吸着に適した角度として54.0を設定
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
    // パラメータ: 外挿に使用する近傍点の数（平面を定義するため3に固定）
    constexpr int k_neighbors = 3;
    // パラメータ: ゼロ除算や計算誤差を避けるための微小値
    constexpr double epsilon = 1e-9;

    if (dataset_.size() < k_neighbors) {
    RCLCPP_WARN(this->get_logger(), "Not enough data in CSV to perform extrapolation.");
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

    // 3. 最も近い k (3) 個の点を取得
    const auto& p1_ptr = *distances[0].second;
    const auto& p2_ptr = *distances[1].second;
    const auto& p3_ptr = *distances[2].second;

    // もし最も近い点との距離がほぼゼロなら、その点の値をそのまま使う
    if (distances[0].first < epsilon) {
        std_msgs::msg::Float32MultiArray angle_msg;
        angle_msg.data.assign(p1_ptr.motors.begin(), p1_ptr.motors.end());
        pub_->publish(angle_msg);
        
        std_msgs::msg::Float32 motor10_msg;
        motor10_msg.data = p1_ptr.motor10;
        pub_motor10_->publish(motor10_msg);
        
        RCLCPP_INFO(this->get_logger(), "Target point is very close to a data point. Using it directly.");
        return;
    }

    // 4. 平面を定義するベクトルと、目標点へのベクトルを計算
    // v1 = P2 - P1
    const double v1_x = p2_ptr.x - p1_ptr.x;
    const double v1_y = p2_ptr.y - p1_ptr.y;
    const double v1_z = p2_ptr.z - p1_ptr.z;
    // v2 = P3 - P1
    const double v2_x = p3_ptr.x - p1_ptr.x;
    const double v2_y = p3_ptr.y - p1_ptr.y;
    const double v2_z = p3_ptr.z - p1_ptr.z;
    // target_vec = Target - P1
    const double target_vec_x = x - p1_ptr.x;
    const double target_vec_y = y - p1_ptr.y;
    const double target_vec_z = z - p1_ptr.z;

    // 5. target_vec = a*v1 + b*v2 となる係数 a, b を最小二乗法で解く
    // 正規方程式 (A^T * A) * [a, b]^T = A^T * target_vec を解く
    const double dot_v1_v1 = v1_x*v1_x + v1_y*v1_y + v1_z*v1_z;
    const double dot_v1_v2 = v1_x*v2_x + v1_y*v2_y + v1_z*v2_z;
    const double dot_v2_v2 = v2_x*v2_x + v2_y*v2_y + v2_z*v2_z;

    const double det = dot_v1_v1 * dot_v2_v2 - dot_v1_v2 * dot_v1_v2;

    double a = 0.0, b = 0.0;
    // detが0に近い場合（3点がほぼ一直線上にある場合）、計算が不安定になるため、
    // 最も単純な最近傍法（一番近い点の値を使う）にフォールバックする
    if (std::abs(det) < epsilon) {
        RCLCPP_WARN(this->get_logger(), "Neighbor points are collinear. Falling back to nearest neighbor.");
        // 最近傍法と同じ処理を行う（p1_ptrの値をそのまま使う）
        std_msgs::msg::Float32MultiArray angle_msg;
        angle_msg.data.assign(p1_ptr.motors.begin(), p1_ptr.motors.end());
        pub_->publish(angle_msg);
        std_msgs::msg::Float32 motor10_msg;
        motor10_msg.data = p1_ptr.motor10;
        pub_motor10_->publish(motor10_msg);
        return;
    } else {
        const double dot_target_v1 = target_vec_x*v1_x + target_vec_y*v1_y + target_vec_z*v1_z;
        const double dot_target_v2 = target_vec_x*v2_x + target_vec_y*v2_y + target_vec_z*v2_z;
        
        // 逆行列を解いて a, b を求める
        a = (dot_target_v1 * dot_v2_v2 - dot_target_v2 * dot_v1_v2) / det;
        b = (dot_target_v2 * dot_v1_v1 - dot_target_v1 * dot_v1_v2) / det;
    }

    // 6. 係数 a, b を使ってモーター角度を外挿
    std::vector<double> extrapolated_motors(9, 0.0);
    for (size_t i = 0; i < 9; ++i) {
        const double m1 = p1_ptr.motors[i];
        const double m2 = p2_ptr.motors[i];
        const double m3 = p3_ptr.motors[i];
        // M_target = M1 + a * (M2 - M1) + b * (M3 - M1)
        extrapolated_motors[i] = m1 + a * (m2 - m1) + b * (m3 - m1);
    }

    constexpr double MIN_ANGLE = 0.0;   // モーター角度の下限値
    constexpr double MAX_ANGLE = 1700.0; // モーター角度の上限値

    // 外挿した角度を制限範囲内にクリップ
    for (auto& angle : extrapolated_motors) {
        if (angle < MIN_ANGLE) angle = MIN_ANGLE;
        if (angle > MAX_ANGLE) angle = MAX_ANGLE;
    }

    double extrapolated_motor10 = p1_ptr.motor10 + a * (p2_ptr.motor10 - p1_ptr.motor10) + b * (p3_ptr.motor10 - p1_ptr.motor10);
    // ★★★ ここから追加 ★★★
    // Z座標に応じてモーター10の角度を補正する
    // --- パラメータ（要調整）---
    // 補正が不要になる基準のZ座標。例えば、ロボットから見て平均的な地面の高さなどを設定します。
    const double Z_REFERENCE = 0.7; 
    // 補正の強さを決める係数（ゲイン）。大きいほどZの変化に敏感に反応します。
    const double MOTOR10_Z_GAIN = 3000.0; 
    // -------------------------

    // 基準Z座標と目標Z座標の差を計算
    // ユーザーの定義通り、zが小さいほど差がプラスになるように(Z_REFERENCE - z)とします
    double z_difference = Z_REFERENCE - pt.z;

    // Z座標の差にゲインを掛けて補正量を算出
    double adjustment = MOTOR10_Z_GAIN * z_difference;

    // 元の計算結果に補正量を加える
    extrapolated_motor10 += adjustment;

    RCLCPP_INFO(this->get_logger(), "Motor10 adjustment for Z(%.2f): %.2f -> Final angle before clamp: %.2f", pt.z, adjustment, extrapolated_motor10);
    // ★★★ 追加ここまで ★★★

    // ★ここからマーカー発行処理を追加 (これは元のコードと同じ)
    // ... (元のマーカー発行処理をここにそのままコピーしてください)
    visualization_msgs::msg::MarkerArray marker_array;
    int marker_id = 0;
    // 近傍点をマーカーとして表示する
    for(int i = 0; i < k_neighbors; ++i) {
        const auto& neighbor_ptr = *distances[i].second;
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = msg->header.frame_id;
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "neighbor_points";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = neighbor_ptr.x;
        marker.pose.position.y = neighbor_ptr.y;
        marker.pose.position.z = neighbor_ptr.z;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.02;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;
        marker.lifetime = rclcpp::Duration(1, 0);
        marker_array.markers.push_back(marker);
    }
    marker_pub_->publish(marker_array);
    // ... (マーカー処理ここまで)


    // 7. 外挿した結果をパブリッシュ
    std_msgs::msg::Float32MultiArray angle_msg;
    angle_msg.data.assign(extrapolated_motors.begin(), extrapolated_motors.end());
    pub_->publish(angle_msg);

    std_msgs::msg::Float32 motor10_msg;
    motor10_msg.data = extrapolated_motor10;
    pub_motor10_->publish(motor10_msg);

    std::stringstream ss;
    for (auto ang : extrapolated_motors) ss << std::fixed << std::setprecision(2) << ang << " ";
    RCLCPP_INFO(this->get_logger(), "Published extrapolated motor1-9: %s, motor10: %.2f", ss.str().c_str(), extrapolated_motor10);
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
    // // --- ここからログ出力の追加 ---
    // std::stringstream ss_target, ss_current;
    // for(const auto& angle : target_angles) ss_target << std::fixed << std::setprecision(2) << angle << " ";
    // for(const auto& angle : current_motor_angles_) ss_current << std::fixed << std::setprecision(2) << angle << " ";
    
    // RCLCPP_INFO(this->get_logger(), "--------------------");
    // RCLCPP_INFO(this->get_logger(), "目標角度: [ %s]", ss_target.str().c_str());
    // RCLCPP_INFO(this->get_logger(), "現在角度: [ %s]", ss_current.str().c_str());
    // // --- ログ出力の追加ここまで ---

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
    double tolerance = 20; // 許容誤差（例: 10度）。モーターの性能に合わせて調整。
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
