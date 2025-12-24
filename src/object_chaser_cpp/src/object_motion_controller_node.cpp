#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

class ObjectMotionControllerNode : public rclcpp::Node
{
public:
    ObjectMotionControllerNode()
        : Node("object_motion_controller_node"),
          approach_phase_(0),
          completion_notified_(false),
          is_stopped_(false),
          current_robot_state_(""),
            last_detection_time_(this->now())
    {
        // Publishers
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/chaser/cmd_vel", 10);
        completion_pub_ = this->create_publisher<std_msgs::msg::Bool>("/chaser/approach_completed", 10);

        // Subscribers
        point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/detected_depth_points", 10,
            std::bind(&ObjectMotionControllerNode::point_callback, this, std::placeholders::_1));

        state_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot/state", 10,
            std::bind(&ObjectMotionControllerNode::state_callback, this, std::placeholders::_1));

        // Control parameters
        this->declare_parameter("target_distance", 0.9);
        this->declare_parameter("stop_threshold", 0.05);
        this->declare_parameter("kp_linear", 0.3);
        this->declare_parameter("max_linear_speed", 0.1);

        target_distance_ = this->get_parameter("target_distance").as_double();
        stop_threshold_ = this->get_parameter("stop_threshold").as_double();
        kp_linear_ = this->get_parameter("kp_linear").as_double();
        max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();

        // Approach parameters
        this->declare_parameter("approach.phase1_switch_x", 1.2);
        this->declare_parameter("approach.lateral_tolerance", 0.3);
        this->declare_parameter("approach.phase1_forward_speed", 0.08);
        this->declare_parameter("approach.phase2_lateral_speed", 0.06);
        this->declare_parameter("approach.phase3_final_forward_speed", 0.08);

        phase1_switch_x_ = this->get_parameter("approach.phase1_switch_x").as_double();
        lateral_tolerance_ = this->get_parameter("approach.lateral_tolerance").as_double();
        phase1_forward_speed_ = this->get_parameter("approach.phase1_forward_speed").as_double();
        phase2_lateral_speed_ = this->get_parameter("approach.phase2_lateral_speed").as_double();
        phase3_final_forward_speed_ = this->get_parameter("approach.phase3_final_forward_speed").as_double();

        // Timer for timeout
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ObjectMotionControllerNode::check_timeout, this));

        RCLCPP_INFO(this->get_logger(), "Object Motion Controller Node started.");
    }

private:
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr completion_pub_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    int approach_phase_;
    bool completion_notified_;
    bool is_stopped_;
    std::string current_robot_state_;
    rclcpp::Time last_detection_time_;

    // Control parameters
    double target_distance_;
    double stop_threshold_;
    double kp_linear_;
    double max_linear_speed_;

    // Approach parameters
    double phase1_switch_x_;
    double lateral_tolerance_;
    double phase1_forward_speed_;
    double phase2_lateral_speed_;
    double phase3_final_forward_speed_;

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
            approach_phase_ = 0;
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

        // camera_color_optical_frame coordinates
        double x_cam = msg->point.x;  // right(+)
        double y_cam = msg->point.y;  // down(+)
        double z_cam = msg->point.z;  // forward(+)

        double target_x = z_cam;  // forward-back
        double target_y = x_cam;  // left-right

        double distance = std::sqrt(target_x * target_x + target_y * target_y);

        execute_robot_control(target_x, target_y, distance);
    }

    void execute_robot_control(double target_x, double target_y, double distance)
    {
        RCLCPP_INFO(this->get_logger(), "物体までの計算上の距離: %.2f m", distance);

        double desired_lateral_offset = 0.0;
        double err_x = target_x - target_distance_;
        double err_y = target_y - desired_lateral_offset;
        double distance_error = distance - target_distance_;

        auto cmd = geometry_msgs::msg::Twist();

        // Stop if target distance reached
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
            approach_phase_ = 0;
            return;
        }

        completion_notified_ = false;

        // Phase control
        if (approach_phase_ == 0)
        {
            if (target_x <= phase1_switch_x_)
            {
                approach_phase_ = 1;
                RCLCPP_INFO(this->get_logger(), "Approach phase -> 1 (lateral alignment)");
            }
            else
            {
                double vx = std::min(phase1_forward_speed_, max_linear_speed_);
                cmd.linear.x = vx;
                cmd.linear.y = 0.0;
                cmd.angular.z = 0.0;
                cmd_pub_->publish(cmd);
                return;
            }
        }

        if (approach_phase_ == 1)
        {
            if (std::abs(err_y) <= lateral_tolerance_)
            {
                approach_phase_ = 2;
                RCLCPP_INFO(this->get_logger(), "Approach phase -> 2 (final forward)");
            }
            else
            {
                double vy = err_y > 0 ? -phase2_lateral_speed_ : phase2_lateral_speed_;
                cmd.linear.x = 0.0;
                cmd.linear.y = vy;
                cmd.angular.z = 0.0;
                cmd_pub_->publish(cmd);
                return;
            }
        }

        if (approach_phase_ == 2)
        {
            double vx = kp_linear_ * err_x;
            double vy = kp_linear_ * err_y * 0.5;

            vx = std::max(-phase3_final_forward_speed_, std::min(phase3_final_forward_speed_, vx));
            vy = std::max(-max_linear_speed_, std::min(max_linear_speed_, vy));

            cmd.linear.x = vx;
            cmd.linear.y = vy;
            cmd.angular.z = 0.0;
            cmd_pub_->publish(cmd);
            return;
        }
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
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectMotionControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
