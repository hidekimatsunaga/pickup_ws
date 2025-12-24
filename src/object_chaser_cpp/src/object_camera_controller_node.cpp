#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

class ObjectCameraControllerNode : public rclcpp::Node
{
public:
    ObjectCameraControllerNode()
        : Node("object_camera_controller_node"),
          current_camera_angle_deg_(0.0),
          smoothed_target_deg_(0.0),
          smoothed_target_initialized_(false),
          current_robot_state_(""),
          last_detection_time_(this->now())
    {
        // Publishers
        camera_swing_pub_ = this->create_publisher<std_msgs::msg::Float32>("/cameraswingmotor/target_angle", 10);

        // Subscribers
        point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/detected_depth_points", 10,
            std::bind(&ObjectCameraControllerNode::point_callback, this, std::placeholders::_1));

        camera_angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/cameraswingmotor/angle", 10,
            std::bind(&ObjectCameraControllerNode::camera_angle_callback, this, std::placeholders::_1));

        state_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot/state", 10,
            std::bind(&ObjectCameraControllerNode::state_callback, this, std::placeholders::_1));

        // Camera control parameters
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

        // Smoothing parameters
        this->declare_parameter("camera.target_smooth_alpha", 0.3);
        this->declare_parameter("camera.max_step_deg", 2.0);
        this->declare_parameter("camera.deadband_deg", 0.4);

        target_smooth_alpha_ = this->get_parameter("camera.target_smooth_alpha").as_double();
        camera_max_step_deg_ = this->get_parameter("camera.max_step_deg").as_double();
        camera_deadband_deg_ = this->get_parameter("camera.deadband_deg").as_double();

        // LUT CSV path
        this->declare_parameter("swing_lut_csv",
                               "/home/matsunaga-h/pickup_ws/src/object_chaser/csv/camera_swing_calib_yz.csv");
        std::string csv_path = this->get_parameter("swing_lut_csv").as_string();
        load_lut(csv_path);

        // Optional timer for detection timeout (no movement control here)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ObjectCameraControllerNode::check_timeout, this));

        RCLCPP_INFO(this->get_logger(), "Object Camera Controller Node started.");
    }

private:
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr camera_swing_pub_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr camera_angle_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    double current_camera_angle_deg_;
    double smoothed_target_deg_;
    bool smoothed_target_initialized_;
    std::string current_robot_state_;
    rclcpp::Time last_detection_time_;

    // Camera parameters
    double far_distance_;
    double near_distance_;
    double far_camera_angle_deg_;
    double near_camera_angle_deg_;
    double min_camera_angle_deg_;
    double max_camera_angle_deg_;

    // Smoothing
    double target_smooth_alpha_;
    double camera_max_step_deg_;
    double camera_deadband_deg_;

    // LUT
    std::vector<double> lut_y_;
    std::vector<double> lut_z_;
    std::vector<double> lut_angle_;

    void camera_angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        current_camera_angle_deg_ = msg->data;
    }

    void state_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        current_robot_state_ = msg->data;

        // When collecting, point camera to max angle immediately.
        if (current_robot_state_ == "collecting")
        {
            auto camera_msg = std_msgs::msg::Float32();
            camera_msg.data = max_camera_angle_deg_;
            camera_swing_pub_->publish(camera_msg);
        }
    }

    void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        // Only act in approaching state
        if (current_robot_state_ != "approaching")
        {
            return;
        }

        last_detection_time_ = this->now();

        // camera_color_optical_frame coordinates
        double y_cam = msg->point.y; // down(+)
        double z_cam = msg->point.z; // forward(+)

        double distance = std::sqrt(z_cam * z_cam + msg->point.x * msg->point.x);
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

        // Clamp to physical range
        target_deg = std::max(min_camera_angle_deg_, std::min(max_camera_angle_deg_, target_deg));

        // Smoothing
        double prev = smoothed_target_initialized_ ? smoothed_target_deg_ : target_deg;
        double alpha = std::max(0.0, std::min(1.0, target_smooth_alpha_));
        double smoothed = (1.0 - alpha) * target_deg + alpha * prev;

        // Rate limit
        double delta = smoothed - prev;
        double max_step = std::max(0.0, camera_max_step_deg_);
        if (delta > max_step)
            smoothed = prev + max_step;
        else if (delta < -max_step)
            smoothed = prev - max_step;

        // Deadband
        if (std::abs(smoothed - prev) < camera_deadband_deg_)
            smoothed = prev;

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
            return far_camera_angle_deg_;
        else if (distance <= near_distance_)
            return near_camera_angle_deg_;
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
            return near_camera_angle_deg_;

        if (k <= 0)
            k = 1;
        if (static_cast<size_t>(k) > n)
            k = static_cast<int>(n);

        std::vector<std::pair<double, size_t>> dists;
        dists.reserve(n);
        for (size_t i = 0; i < n; ++i)
        {
            double dy = y - lut_y_[i];
            double dz = z - lut_z_[i];
            double d = std::hypot(dy, dz);
            dists.push_back({d, i});
        }

        std::sort(dists.begin(), dists.end());

        double closest_dist = dists[0].first;
        if (closest_dist < 0.1)
            return min_camera_angle_deg_;

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
            return lut_angle_[dists[0].second];

        return num / den;
    }

    void load_lut(const std::string &csv_path)
    {
        std::ifstream file(csv_path);
        if (!file.is_open())
        {
            RCLCPP_WARN(this->get_logger(), "LUT CSV not found: %s", csv_path.c_str());
            return;
        }

        std::string line;
        // Skip header
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
                    double y = std::stod(row[1]); // y_cam
                    double z = std::stod(row[2]); // z_cam
                    double angle = std::stod(row[4]); // camera_angle_deg

                    lut_y_.push_back(y);
                    lut_z_.push_back(z);
                    lut_angle_.push_back(angle);
                }
                catch (const std::exception &e)
                {
                    RCLCPP_WARN(this->get_logger(), "Failed to parse LUT line: %s", e.what());
                }
            }
        }

        file.close();
        RCLCPP_INFO(this->get_logger(), "Loaded camera swing LUT: %s, samples=%zu",
                    csv_path.c_str(), lut_angle_.size());
    }

    void check_timeout()
    {
        // Camera node does not enforce robot stop; just logs if no detections.
        auto now = this->now();
        auto duration = now - last_detection_time_;
        if (duration.seconds() > 10.0 && current_robot_state_ == "approaching")
        {
            RCLCPP_WARN(this->get_logger(), "No detections for 10s while approaching.");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectCameraControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
