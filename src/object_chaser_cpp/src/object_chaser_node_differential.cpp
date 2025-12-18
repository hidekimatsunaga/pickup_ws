#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <memory>
#include <chrono>

class ObjectChaserNodeDifferential : public rclcpp::Node
{
public:
    ObjectChaserNodeDifferential() : Node("object_chaser_node_differential"),
                                     current_camera_angle_deg_(0.0),
                                     completion_notified_(false),
                                     smoothed_target_deg_(0.0),
                                     smoothed_target_initialized_(false),
                                     is_stopped_(false)
    {
        // TF2のためのバッファとリスナーを初期化
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // === パブリッシャ ===
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/chaser/cmd_vel", 10);
        camera_swing_pub_ = this->create_publisher<std_msgs::msg::Float32>("/cameraswingmotor/target_angle", 10);
        completion_pub_ = this->create_publisher<std_msgs::msg::Bool>("/chaser/approach_completed", 10);

        // === サブスクライバ ===
        point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/detected_depth_points", 10,
            std::bind(&ObjectChaserNodeDifferential::point_callback, this, std::placeholders::_1));
        
        camera_angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/cameraswingmotor/angle", 10,
            std::bind(&ObjectChaserNodeDifferential::camera_angle_callback, this, std::placeholders::_1));
        
        state_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot/state", 10,
            std::bind(&ObjectChaserNodeDifferential::state_callback, this, std::placeholders::_1));

        // === 制御パラメータ (ロボット移動) ===
        this->declare_parameter("target_distance", 0.9);
        this->declare_parameter("stop_threshold", 0.05);
        this->declare_parameter("max_linear_speed", 0.1);
        this->declare_parameter("max_angular_speed", 0.3);

        target_distance_ = this->get_parameter("target_distance").as_double();
        stop_threshold_ = this->get_parameter("stop_threshold").as_double();
        max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();

        // === 差動駆動制御パラメータ ===
        this->declare_parameter("differential.kp_angular", 1.5);
        this->declare_parameter("differential.kp_linear", 0.3);
        this->declare_parameter("differential.angle_tolerance", 0.15);
        this->declare_parameter("differential.approach_speed", 0.08);
        this->declare_parameter("differential.turn_in_place_threshold", 0.5);

        kp_angular_ = this->get_parameter("differential.kp_angular").as_double();
        kp_linear_ = this->get_parameter("differential.kp_linear").as_double();
        angle_tolerance_ = this->get_parameter("differential.angle_tolerance").as_double();
        approach_speed_ = this->get_parameter("differential.approach_speed").as_double();
        turn_in_place_threshold_ = this->get_parameter("differential.turn_in_place_threshold").as_double();

        // === カメラ制御パラメータ ===
        this->declare_parameter("far_distance", 2.0);
        this->declare_parameter("near_distance", 0.5);
        this->declare_parameter("far_camera_angle_deg", 30.0);
        this->declare_parameter("near_camera_angle_deg", 60.0);
        this->declare_parameter("min_camera_angle_deg", 17.6);
        this->declare_parameter("max_camera_angle_deg", 63.9);

        far_distance_ = this->get_parameter("far_distance").as_double();
        near_distance_ = this->get_parameter("near_distance").as_double();
        far_camera_angle_deg_ = this->get_parameter("far_camera_angle_deg").as_double();
        near_camera_angle_deg_ = this->get_parameter("near_camera_angle_deg").as_double();
        min_camera_angle_deg_ = this->get_parameter("min_camera_angle_deg").as_double();
        max_camera_angle_deg_ = this->get_parameter("max_camera_angle_deg").as_double();

        // === カメラ揺れ対策用パラメータ ===
        this->declare_parameter("camera.target_smooth_alpha", 0.3);
        this->declare_parameter("camera.max_step_deg", 2.0);
        this->declare_parameter("camera.deadband_deg", 0.4);

        target_smooth_alpha_ = this->get_parameter("camera.target_smooth_alpha").as_double();
        camera_max_step_deg_ = this->get_parameter("camera.max_step_deg").as_double();
        camera_deadband_deg_ = this->get_parameter("camera.deadband_deg").as_double();

        // === LUT CSVパス ===
        this->declare_parameter("swing_lut_csv", 
            "/home/matsunaga-h/pickup_ws/src/object_chaser/csv/camera_swing_calib_yz.csv");
        std::string csv_path = this->get_parameter("swing_lut_csv").as_string();
        load_lut(csv_path);

        // タイムアウト処理用
        last_detection_time_ = this->now();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ObjectChaserNodeDifferential::check_timeout, this));

        RCLCPP_INFO(this->get_logger(), "Object Chaser Node (Differential Drive) has been started.");
    }

private:
    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr camera_swing_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr completion_pub_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr camera_angle_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // State variables
    double current_camera_angle_deg_;
    bool completion_notified_;
    double smoothed_target_deg_;
    bool smoothed_target_initialized_;
    bool is_stopped_;
    std::string current_robot_state_;
    rclcpp::Time last_detection_time_;

    // Control parameters
    double target_distance_;
    double stop_threshold_;
    double max_linear_speed_;
    double max_angular_speed_;

    // Differential drive parameters
    double kp_angular_;
    double kp_linear_;
    double angle_tolerance_;
    double approach_speed_;
    double turn_in_place_threshold_;

    // Camera parameters
    double far_distance_;
    double near_distance_;
    double far_camera_angle_deg_;
    double near_camera_angle_deg_;
    double min_camera_angle_deg_;
    double max_camera_angle_deg_;

    // Camera smoothing parameters
    double target_smooth_alpha_;
    double camera_max_step_deg_;
    double camera_deadband_deg_;

    // LUT data
    std::vector<double> lut_y_;
    std::vector<double> lut_z_;
    std::vector<double> lut_angle_;

    void load_lut(const std::string& csv_path)
    {
        std::ifstream file(csv_path);
        if (!file.is_open())
        {
            RCLCPP_WARN(this->get_logger(), "LUT CSV not found: %s", csv_path.c_str());
            return;
        }

        std::string line;
        // ヘッダー行をスキップ
        std::getline(file, line);

        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string cell;
            std::vector<std::string> row;

            while (std::getline(ss, cell, ','))
            {
                row.push_back(cell);
            }

            if (row.size() >= 5)
            {
                try
                {
                    double y = std::stod(row[1]);  // y_cam
                    double z = std::stod(row[2]);  // z_cam
                    double angle = std::stod(row[4]);  // camera_angle_deg
                    
                    lut_y_.push_back(y);
                    lut_z_.push_back(z);
                    lut_angle_.push_back(angle);
                }
                catch (const std::exception& e)
                {
                    RCLCPP_WARN(this->get_logger(), "Failed to parse LUT line: %s", e.what());
                }
            }
        }

        file.close();
        RCLCPP_INFO(this->get_logger(), "Loaded camera swing LUT: %s, samples=%zu", 
                    csv_path.c_str(), lut_angle_.size());
    }

    void camera_angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        current_camera_angle_deg_ = msg->data;
    }

    void state_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        current_robot_state_ = msg->data;
        
        if (current_robot_state_ != "approaching")
        {
            if (!is_stopped_)
            {
                is_stopped_ = true;
                stop_robot();
            }
            completion_notified_ = false;
        }
        else
        {
            if (is_stopped_)
            {
                is_stopped_ = false;
            }
        }
    }

    void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        if (current_robot_state_ != "approaching")
        {
            is_stopped_ = true;
            stop_robot();
            return;
        }

        if (is_stopped_)
        {
            return;
        }

        last_detection_time_ = this->now();

        // camera_color_optical_frame の座標をそのまま使う
        double x_cam = msg->point.x;  // 右(+)
        double y_cam = msg->point.y;  // 下(+)
        double z_cam = msg->point.z;  // 前(+)

        double target_x = z_cam;  // 前後
        double target_y = x_cam;  // 左右

        double distance = std::sqrt(target_x * target_x + target_y * target_y);

        // ロボット本体の移動制御（差動駆動）
        execute_robot_control_differential(target_x, target_y, distance);

        // カメラ制御
        control_camera_swing(y_cam, z_cam, distance);
    }

    void control_camera_swing(double y_cam, double z_cam, double distance)
    {
        double target_deg;
        
        if (!lut_angle_.empty())
        {
            target_deg = lookup_camera_angle_from_yz(y_cam, z_cam, 2);
        }
        else
        {
            target_deg = control_camera_swing_by_distance(distance);
        }

        // 物理的な可動範囲でクランプ
        target_deg = std::max(min_camera_angle_deg_, std::min(max_camera_angle_deg_, target_deg));

        // 平滑化処理
        double prev = smoothed_target_initialized_ ? smoothed_target_deg_ : target_deg;
        double alpha = std::max(0.0, std::min(1.0, target_smooth_alpha_));
        double smoothed = (1.0 - alpha) * target_deg + alpha * prev;

        // レート制限
        double delta = smoothed - prev;
        double max_step = std::max(0.0, camera_max_step_deg_);
        if (delta > max_step)
        {
            smoothed = prev + max_step;
        }
        else if (delta < -max_step)
        {
            smoothed = prev - max_step;
        }

        // デッドバンド
        if (std::abs(smoothed - prev) < camera_deadband_deg_)
        {
            smoothed = prev;
        }

        smoothed_target_deg_ = smoothed;
        smoothed_target_initialized_ = true;

        auto cmd_msg = std_msgs::msg::Float32();
        cmd_msg.data = smoothed;
        camera_swing_pub_->publish(cmd_msg);

        RCLCPP_INFO(this->get_logger(), 
                    "[Camera] y=%.3f, z=%.3f, dist=%.2f -> target=%.2f deg, cmd=%.2f deg",
                    y_cam, z_cam, distance, target_deg, smoothed);
    }

    double control_camera_swing_by_distance(double distance)
    {
        if (distance >= far_distance_)
        {
            return far_camera_angle_deg_;
        }
        else if (distance <= near_distance_)
        {
            return near_camera_angle_deg_;
        }
        else
        {
            double ratio = (distance - near_distance_) / (far_distance_ - near_distance_);
            return near_camera_angle_deg_ + (far_camera_angle_deg_ - near_camera_angle_deg_) * ratio;
        }
    }

    double lookup_camera_angle_from_yz(double y, double z, int k)
    {
        size_t n = lut_angle_.size();
        if (n == 0)
        {
            return near_camera_angle_deg_;
        }

        if (k <= 0) k = 1;
        if (static_cast<size_t>(k) > n) k = n;

        // 距離計算
        std::vector<std::pair<double, size_t>> dists;
        for (size_t i = 0; i < n; ++i)
        {
            double dy = y - lut_y_[i];
            double dz = z - lut_z_[i];
            double d = std::hypot(dy, dz);
            dists.push_back({d, i});
        }

        // 距離でソート
        std::sort(dists.begin(), dists.end());

        // 最も近いサンプルとの距離
        double closest_dist = dists[0].first;
        
        if (closest_dist < 0.1)
        {
            return min_camera_angle_deg_;
        }

        // k-NN 加重平均
        double eps = 1e-6;
        double num = 0.0;
        double den = 0.0;
        for (int i = 0; i < k; ++i)
        {
            double d = dists[i].first;
            size_t idx = dists[i].second;
            double w = 1.0 / (d + eps);
            num += w * lut_angle_[idx];
            den += w;
        }

        if (den <= 0.0)
        {
            return lut_angle_[dists[0].second];
        }

        return num / den;
    }

    void execute_robot_control_differential(double target_x, double target_y, double distance)
    {
        RCLCPP_INFO(this->get_logger(), "物体までの計算上の距離: %.2f m", distance);

        double distance_error = distance - target_distance_;

        auto cmd = geometry_msgs::msg::Twist();

        // 目標距離に到達したかどうか
        if (std::abs(distance_error) < stop_threshold_)
        {
            is_stopped_ = true;
            stop_robot();
            RCLCPP_INFO(this->get_logger(), "Target distance reached.");

            if (!completion_notified_)
            {
                auto completion_msg = std_msgs::msg::Bool();
                completion_msg.data = true;
                completion_pub_->publish(completion_msg);
                completion_notified_ = true;
            }
            return;
        }

        completion_notified_ = false;

        // 物体の方向角度を計算
        double target_angle = std::atan2(target_y, target_x);
        
        RCLCPP_INFO(this->get_logger(), 
                    "Target: x=%.2f, y=%.2f, angle=%.1f deg",
                    target_x, target_y, target_angle * 180.0 / M_PI);

        double angle_error = std::abs(target_angle);

        // 大きく角度がずれている場合：停止して旋回
        if (angle_error > turn_in_place_threshold_)
        {
            double angular_z = kp_angular_ * target_angle;
            angular_z = std::max(-max_angular_speed_, std::min(max_angular_speed_, angular_z));
            
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.angular.z = angular_z;
            
            RCLCPP_INFO(this->get_logger(), "Turn in place: angular.z=%.3f rad/s", angular_z);
        }
        // 角度がある程度合っている場合：前進しながら微調整
        else if (angle_error > angle_tolerance_)
        {
            double linear_x = kp_linear_ * distance_error;
            linear_x = std::max(0.0, std::min(approach_speed_, linear_x));
            
            double angular_z = kp_angular_ * target_angle;
            angular_z = std::max(-max_angular_speed_, std::min(max_angular_speed_, angular_z));
            
            cmd.linear.x = linear_x;
            cmd.linear.y = 0.0;
            cmd.angular.z = angular_z;
            
            RCLCPP_INFO(this->get_logger(), 
                        "Approach with correction: linear.x=%.3f, angular.z=%.3f",
                        linear_x, angular_z);
        }
        // 角度が十分合っている場合：まっすぐ前進
        else
        {
            double linear_x = kp_linear_ * distance_error;
            linear_x = std::max(0.0, std::min(approach_speed_, linear_x));
            
            cmd.linear.x = linear_x;
            cmd.linear.y = 0.0;
            cmd.angular.z = 0.0;
            
            RCLCPP_INFO(this->get_logger(), "Straight approach: linear.x=%.3f", linear_x);
        }

        cmd_pub_->publish(cmd);
    }

    void check_timeout()
    {
        if (current_robot_state_ != "approaching")
        {
            if (!is_stopped_)
            {
                is_stopped_ = true;
                stop_robot();
            }
            return;
        }

        auto now = this->now();
        auto duration = now - last_detection_time_;
        if (duration.seconds() > 10.0)
        {
            is_stopped_ = true;
            stop_robot();
        }
    }

    void stop_robot()
    {
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);

        if (current_robot_state_ == "collecting")
        {
            auto camera_msg = std_msgs::msg::Float32();
            camera_msg.data = max_camera_angle_deg_;
            camera_swing_pub_->publish(camera_msg);
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectChaserNodeDifferential>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
