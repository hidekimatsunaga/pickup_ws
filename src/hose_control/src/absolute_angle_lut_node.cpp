// absolute_angle_lut_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdint>
#include <optional>

struct Sample {
  // 位置（CSVの座標系そのまま）
  double x{0}, y{0}, z{0};
  // 絶対角（ここでは m6,m7,m8 の3要素を想定）
  std::vector<double> motors;  // size = motor_cols_.size()
  // motor10（任意）
  double motor10{0.0};
  // 任意：元フレーム名（未使用だが将来拡張に備えて保持）
  std::string frame;
};

class AbsoluteAngleLUTNode : public rclcpp::Node {
public:
  AbsoluteAngleLUTNode()
  : Node("absolute_angle_lut_node")
  {
    // ---- Parameters ----
    csv_path_        = this->declare_parameter<std::string>("csv_path", "/home/matsunaga-h/pickup_ws/src/pcc_test/pcc_test/lut_csv/pcc_measure.csv");
    goal_topic_      = this->declare_parameter<std::string>("goal_topic", "/hose/goal_point");
    out_topic_       = this->declare_parameter<std::string>("out_topic", "/motor_angles");
    out10_topic_     = this->declare_parameter<std::string>("out10_topic", "/chokudomotor/target_angle");
    marker_topic_    = this->declare_parameter<std::string>("marker_topic", "/hose/neighbor_points");
    k_neighbors_     = this->declare_parameter<int>("k_neighbors", 4);
    max_neighbor_m_  = this->declare_parameter<double>("max_neighbor_radius", 0.10); // 10 cm 以内
    epsilon_         = this->declare_parameter<double>("epsilon", 1e-9);
    use_motor10_     = this->declare_parameter<bool>("use_motor10", false);          // ★ デフォルト無効

    // CSVカラムのマッピング（共有してもらったCSVに合わせた既定）
    // header: stamp,frame,x,y,z,m6,m7,m8,z6,z7,z8
    dataset_frame_  = this->declare_parameter<std::string>("dataset_frame", "camera_color_optical_frame");
    x_col_ = this->declare_parameter<int>("x_col", 2);
    y_col_ = this->declare_parameter<int>("y_col", 3);
    z_col_ = this->declare_parameter<int>("z_col", 4);

    // motor_cols は int64 配列で受け取り → int へ詰め替え
    std::vector<int64_t> motor_cols_i64 =
      this->declare_parameter<std::vector<int64_t>>("motor_cols", {5,6,7}); // m6,m7,m8
    motor_cols_.assign(motor_cols_i64.begin(), motor_cols_i64.end());

    // 出力先インデックス（/motor_angles のどこに書くか）：既定は 6,7,8
    std::vector<int64_t> target_idx_i64 =
      this->declare_parameter<std::vector<int64_t>>("target_indices", {6,7,8});
    target_indices_.assign(target_idx_i64.begin(), target_idx_i64.end());

    // motor10 列は今回CSVに無いので -1 を既定に
    motor10_col_ = this->declare_parameter<int>("motor10_col", -1);
    frame_col_   = this->declare_parameter<int>("frame_col", -1); // -1: 使わない

    // ---- I/O ----
    auto qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
    sub_goal_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      goal_topic_, 10, std::bind(&AbsoluteAngleLUTNode::onGoal, this, std::placeholders::_1));

    sub_current_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/motor_current_angles", 10,
      [this](std_msgs::msg::Float32MultiArray::SharedPtr m){
        last_current_angles_.assign(m->data.begin(), m->data.end());
      });

    pub_angles_  = this->create_publisher<std_msgs::msg::Float32MultiArray>(out_topic_, qos);
    pub_angle10_ = this->create_publisher<std_msgs::msg::Float32>(out10_topic_, 10);
    pub_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);

    // ---- Load CSV ----
    loadCSV(csv_path_);

    RCLCPP_INFO(get_logger(),
      "AbsoluteAngleLUTNode ready. csv=%s (N=%zu), goal_topic=%s, out=%s, out10=%s, k=%d, max_r=%.0f mm",
      csv_path_.c_str(), dataset_.size(), goal_topic_.c_str(), out_topic_.c_str(), out10_topic_.c_str(),
      k_neighbors_, max_neighbor_m_*1000.0);
  }

private:
  // Params
  std::string csv_path_, goal_topic_, out_topic_, out10_topic_, marker_topic_;
  int k_neighbors_;
  double max_neighbor_m_, epsilon_;
  bool use_motor10_;
  int x_col_, y_col_, z_col_, motor10_col_, frame_col_;
  std::string dataset_frame_;
  std::vector<int> motor_cols_;       // 読み出す motor 列番号（m6,m7,m8）
  std::vector<int> target_indices_;   // 出力先 index（6,7,8 など）
  std::vector<float> last_current_angles_;  // /motor_current_angles の最新値

  // Data
  std::vector<Sample> dataset_;

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_current_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr    pub_angles_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr              pub_angle10_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_markers_;

  // --- CSV loader ---
  void loadCSV(const std::string& path)
  {
    dataset_.clear();

    std::ifstream ifs(path);
    if (!ifs.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open CSV: %s", path.c_str());
      return;
    }
    std::string line;
    if (!std::getline(ifs, line)) {
      RCLCPP_ERROR(get_logger(), "CSV is empty: %s", path.c_str());
      return;
    }
    // 1行目はヘッダとみなしてスキップ
    size_t line_no = 1;
    while (std::getline(ifs, line)) {
      ++line_no;
      if (line.empty()) continue;

      std::vector<std::string> tok;
      splitCSV(line, tok);

      // 必要列の範囲チェック
      int need_cols = std::max({x_col_, y_col_, z_col_, motor10_col_});
      for (int c : motor_cols_) need_cols = std::max(need_cols, c);
      if (need_cols >= static_cast<int>(tok.size())) {
        RCLCPP_WARN(get_logger(), "Row %zu: not enough columns (%zu). Skip.", line_no, tok.size());
        continue;
      }

      try {
        Sample s;
        s.x = std::stod(tok.at(x_col_));
        s.y = std::stod(tok.at(y_col_));
        s.z = std::stod(tok.at(z_col_));
        if (frame_col_ >= 0 && frame_col_ < static_cast<int>(tok.size())) {
          s.frame = tok.at(frame_col_);
        }

        // motors（m6,m7,m8）だけ読む
        s.motors.reserve(motor_cols_.size());
        for (int c : motor_cols_) {
          s.motors.push_back(std::stod(tok.at(c)));
        }

        // motor10（任意）
        if (use_motor10_ && motor10_col_ >= 0) {
          s.motor10 = std::stod(tok.at(motor10_col_));
        }

        dataset_.push_back(std::move(s));
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_logger(), "Row %zu parse error: %s", line_no, e.what());
      }
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu samples from %s", dataset_.size(), path.c_str());
  }

  static void splitCSV(const std::string& line, std::vector<std::string>& out)
  {
    out.clear();
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
      out.push_back(cell);
    }
  }

  // --- Goal handler ---
  void onGoal(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (dataset_.empty()) {
      RCLCPP_WARN(get_logger(), "Dataset is empty. Ignore goal.");
      return;
    }

    const double gx = msg->point.x;
    const double gy = msg->point.y;
    const double gz = msg->point.z;

    // 1) 全点との距離
    std::vector<std::pair<double, size_t>> dist_idx;
    dist_idx.reserve(dataset_.size());
    for (size_t i = 0; i < dataset_.size(); ++i) {
      const auto& s = dataset_[i];
      const double dx = (s.x - gx), dy = (s.y - gy), dz = (s.z - gz);
      double d = std::sqrt(dx*dx + dy*dy + dz*dz);
      dist_idx.emplace_back(d, i);
    }

    // 2) ソート & 近傍抽出
    std::sort(dist_idx.begin(), dist_idx.end(),
              [](const auto& a, const auto& b){ return a.first < b.first; });

    const int K = std::max(1, std::min<int>(k_neighbors_, static_cast<int>(dist_idx.size())));
    std::vector<std::pair<double,size_t>> neigh(dist_idx.begin(), dist_idx.begin() + K);

    // 距離しきい（遠すぎると指令を出さない）
    if (neigh[0].first > max_neighbor_m_) {
      RCLCPP_WARN(get_logger(), "No neighbor within %.1f mm (nearest=%.1f mm). Skip.",
                  max_neighbor_m_*1000.0, neigh[0].first*1000.0);
      publishNeighborMarkers(msg->header.frame_id, neigh); // 可視化だけ出す
      return;
    }

    // 3) もし最小距離がほぼ0ならそのまま採用
    if (neigh[0].first < epsilon_) {
      const auto& s = dataset_[neigh[0].second];
      publishAnglesPartial(s.motors);
      if (use_motor10_) {
        std_msgs::msg::Float32 m; m.data = static_cast<float>(s.motor10);
        pub_angle10_->publish(m);
      }
      publishNeighborMarkers(msg->header.frame_id, neigh);
      RCLCPP_INFO(get_logger(), "Used exact sample (d≈0).");
      return;
    }

    // 4) 逆距離重み (IDW)
    std::vector<double> weights;
    weights.reserve(K);
    double wsum = 0.0;
    for (int i = 0; i < K; ++i) {
      double w = 1.0 / std::max(neigh[i].first, epsilon_);
      weights.push_back(w);
      wsum += w;
    }
    for (double& w : weights) w /= wsum;

    // 5) 補間（m6,m7,m8 の3本）
    std::vector<double> out3(motor_cols_.size(), 0.0);
    double out10 = 0.0;
    for (int i = 0; i < K; ++i) {
      const auto& s = dataset_[neigh[i].second];
      const double w = weights[i];
      for (size_t j = 0; j < out3.size() && j < s.motors.size(); ++j) {
        out3[j] += s.motors[j] * w;
      }
      if (use_motor10_) out10 += s.motor10 * w;
    }

    // 6) Publish（3本だけ上書き）
    publishAnglesPartial(out3);
    if (use_motor10_) {
      std_msgs::msg::Float32 m; m.data = static_cast<float>(out10);
      pub_angle10_->publish(m);
    }
    publishNeighborMarkers(msg->header.frame_id, neigh);

    // ログ
    {
      std::ostringstream oss;
      for (size_t j = 0; j < out3.size(); ++j) {
        oss << std::fixed << std::setprecision(2) << out3[j] << (j+1<out3.size() ? " " : "");
      }
      if (use_motor10_) {
        RCLCPP_INFO(get_logger(), "LUT abs angles (m6-8): [%s], motor10=%.2f", oss.str().c_str(), out10);
      } else {
        RCLCPP_INFO(get_logger(), "LUT abs angles (m6-8): [%s]", oss.str().c_str());
      }
    }
  }

  // --- Publishers ---

  // m6,m7,m8 の3値を target_indices（例: 6,7,8）に上書きし、他は /motor_current_angles を継承
  void publishAnglesPartial(const std::vector<double>& motors3)
  {
    std_msgs::msg::Float32MultiArray msg;

    // ベースは「最新の現在角」をコピー。無ければ9chゼロで初期化。
    if (!last_current_angles_.empty()) {
      msg.data = last_current_angles_;
    } else {
      msg.data.assign(9, 0.0f);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "No /motor_current_angles yet. Filling others with 0.");
    }

    // m6,m7,m8 を target_indices に上書き
    for (size_t j = 0; j < motors3.size() && j < target_indices_.size(); ++j) {
      const int idx = target_indices_[j];
      if (idx < 0) continue;
      if (static_cast<size_t>(idx) >= msg.data.size())
        msg.data.resize(idx + 1, 0.0f);
      msg.data[idx] = static_cast<float>(motors3[j]);
    }

    pub_angles_->publish(msg);
  }

  void publishNeighborMarkers(const std::string& frame_id,
                              const std::vector<std::pair<double,size_t>>& neigh)
  {
    visualization_msgs::msg::MarkerArray ma;
    rclcpp::Time now = this->get_clock()->now();
    int id = 0;
    for (const auto& nd : neigh) {
      const auto& s = dataset_[nd.second];
      visualization_msgs::msg::Marker mk;
      mk.header.frame_id = frame_id;               // goal と同じフレームで表示
      mk.header.stamp = now;
      mk.ns = "lut_neighbors";
      mk.id = id++;
      mk.type = visualization_msgs::msg::Marker::SPHERE;
      mk.action = visualization_msgs::msg::Marker::ADD;
      mk.pose.position.x = s.x;
      mk.pose.position.y = s.y;
      mk.pose.position.z = s.z;
      mk.pose.orientation.w = 1.0;
      mk.scale.x = mk.scale.y = mk.scale.z = 0.02; // 2cm
      mk.color.r = 1.0f; mk.color.g = 0.0f; mk.color.b = 0.0f; mk.color.a = 1.0f;
      mk.lifetime = rclcpp::Duration(1, 0);
      ma.markers.push_back(mk);
    }
    pub_markers_->publish(ma);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AbsoluteAngleLUTNode>());
  rclcpp::shutdown();
  return 0;
}
