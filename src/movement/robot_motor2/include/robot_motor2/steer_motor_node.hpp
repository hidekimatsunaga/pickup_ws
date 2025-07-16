// #pragma once

// #include <rclcpp/rclcpp.hpp>
// #include <robot_motor/msg/steer_motor.hpp>
// #include <dynamixel_sdk_custom_interfaces/srv/get_position.hpp>
// #include <dynamixel_sdk_custom_interfaces/msg/set_position.hpp>
// #include <dynamixel_sdk_examples/motor_param.hpp>
// #include <memory>
// #include <vector>

// class SteerMotorNode : public rclcpp::Node
// {
// public:
//     SteerMotorNode(); // Constructor
//     ~SteerMotorNode(); // Destructor

// // private:
// //     // コールバック関数
// //     void steerAngleCallback(const robot_motor::msg::SteerMotor::SharedPtr ang_msg);
// //     void convertPositionRadian(const dynamixel_sdk_custom_interfaces::srv::GetPosition::Response::SharedPtr get_pos);

// //     // サブスクライバ
// //     rclcpp::Subscription<robot_motor::msg::SteerMotor>::SharedPtr steer_ang_sub_;
// //     // パブリッシャ
// //     rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr steer_set_pub_;
// //     rclcpp::Publisher<robot_motor::msg::SteerMotor>::SharedPtr steer_odom_pub_;
// //     // サービスクライアント
// //     rclcpp::Client<dynamixel_sdk_custom_intefaces::srv::GetPosition>::SharedPtr steer_client_;

// //     // モーター設定値と現在値
// //     float set_pos[size];
// //     float current_pos[size];
// // };
// private:
//     // Timer for periodic position requests
//     rclcpp::TimerBase::SharedPtr timer_;

//     // Publisher
//     rclcpp::Publisher<blower_motor::msg::SteerMotor>::SharedPtr steer_odom_pub_;

//     // Subscriber
//     rclcpp::Subscription<robot_motor::msg::SteerMotor>::SharedPtr steer_ang_sub_;

//     // Service Client
//     rclcpp::Client<dynamixel_sdk_custom_interfaces::srv::GetPosition>::SharedPtr steer_client_;

//     // Motor positions
//     std::vector<float> set_pos_;
//     std::vector<float> current_pos_;

//     motor_ids_ = {11, 12, 13};


//     // Methods
//     void timerCallback(); // Timer-based callback to request motor positions
//     void positionCallback(const robot_motor::msg::SteerMotor::SharedPtr msg); // Handle position requests
//     void processResponse(int motor_id, int position); // Convert position to radians
// };

// SteerMotorNode::SteerMotorNode()
//     : Node("steer_motor_node")
// {
//     // モーター設定値を初期化
//     for (int i = 0; i < size; i++)
//         set_pos[i] = home_pos[i];

//     // サブスクライバを作成
//     steer_ang_sub_ = this->create_subscription<robot_motor::msg::SteerMotor>(
//         "steer_angle", 10, std::bind(&SteerMotorNode::steerAngleCallback, this, std::placeholders::_1));

//     // パブリッシャを作成
//     steer_set_pub_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("set_position", 10);
//     steer_odom_pub_ = this->create_publisher<blower_motor::msg::SteerMotor>("steer_odom", 10);

//     // サービスクライアントを作成
//     steer_client_ = this->create_client<dynamixel_sdk_custom_interfaces::srv::GetPosition>("get_position");

//     RCLCPP_INFO(this->get_logger(), "SteerMotorNode initialized.");
// }

// SteerMotorNode::~SteerMotorNode()
// {
//     RCLCPP_INFO(this->get_logger(), "SteerMotorNode is shutting down.");
// }

// void SteerMotorNode::steerAngleCallback(const robot_motor::msg::SteerMotor::SharedPtr ang_msg)
// {
//     // ラジアンを位置に変換
//     // 各モーターに対して目標位置を計算し、パブリッシュ
//     for (size_t i = 0; i < motor_ids_.size(); ++i) {
//         int target_position = home_pos[i] - static_cast<int>(ang_msg->phi / M_PI * 2048);
//         // set_pos[1] = home_pos[1] - static_cast<int>(ang_msg->phi2 / M_PI * 2048);
//         // set_pos[2] = home_pos[2] - static_cast<int>(ang_msg->phi3 / M_PI * 2048);

//         RCLCPP_INFO(this->get_logger(), "Set Position: id %d = %d", i, target_position);

//         // モーターの目標位置をパブリッシュ
//         dynamixel_sdk_custom_interfaces::msg::SetPosition position_msg;
//         position_msg.id = motor_ids_[i];
//         position_msg.position = target_position;
//         // パブリッシュ
//         set_position_pub_->publish(position_msg);
//     }

//     // auto set_position_msg = dynamixel_sdk_custom_intaefaces::msg::SetPosition();
//     // set_position_msg.position1 = set_pos[0];
//     // set_position_msg.position2 = set_pos[1];
//     // set_position_msg.position3 = set_pos[2];
//     // steer_set_pub_->publish(set_position_msg);
// }

// void SteerMotorNode::convertPositionRadian(const dynamixel_sdk_custom_interfaces::srv::GetPosition::Response::SharedPtr get_pos)
// {
//     if (!get_position_client_->wait_for_service(std::chrono::seconds(1))) {
//       RCLCPP_WARN(this->get_logger(), "get_position service not available");
//       return;
//     }

//     for (const auto &motor_id : motor_ids_) {
//       auto request = std::make_shared<dynamixel_sdk_custom_interfaces::srv::GetPosition::Request>();
//       request->id = motor_id;

//       auto future = get_position_client_->async_send_request(request);
//       futures_.push_back(future.share());
//     }

//     processFutures();
// }

  
// //     current_pos[0] = M_PI * (get_pos->position1 - home_pos[0]) / 2048;
// //     current_pos[1] = M_PI * (get_pos->position2 - home_pos[1]) / 2048;
// //     current_pos[2] = M_PI * (get_pos->position3 - home_pos[2]) / 2048;

// //     RCLCPP_INFO(this->get_logger(), "Get Position: id1 = %.2f, id2 = %.2f, id3 = %.2f", current_pos[0], current_pos[1], current_pos[2]);

// //     // 現在位置をパブリッシュ
// //     auto odom_msg = blower_motor::msg::SteerMotor();
// //     odom_msg.phi1 = current_pos[0];
// //     odom_msg.phi2 = current_pos[1];
// //     odom_msg.phi3 = current_pos[2];
// //     steer_odom_pub_->publish(odom_msg);
// // }

// void processFutures()
//   {
//     for (auto it = futures_.begin(); it != futures_.end();) {
//       if (it->wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
//         try {
//           auto response = it->get();
//           RCLCPP_INFO(this->get_logger(), "Current position: %d", response->position);
//         } catch (const std::exception &e) {
//           RCLCPP_ERROR(this->get_logger(), "Failed to get position: %s", e.what());
//         }
//         it = futures_.erase(it);
//       } else {
//         ++it;
//       }
//     }
//   }
// #pragma once

// #include <rclcpp/rclcpp.hpp>
// #include <my_messages/msg/steer_motor.hpp>
// #include <dynamixel_sdk_custom_interfaces/srv/get_position.hpp>
// #include <dynamixel_sdk_custom_interfaces/msg/set_position.hpp>
// // #include <dynamixel_sdk_examples/read_write_node.hpp>
// #include "motor_param.hpp"
// #include <memory>
// #include <vector>
// #include <chrono>

// class SteerMotorNode : public rclcpp::Node
// {
// public:
//     SteerMotorNode();
//     ~SteerMotorNode();

// private:
//     // Timer for periodic position requests
//     rclcpp::TimerBase::SharedPtr timer_;

//     // Publisher
//     rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr set_position_pub_;
//     rclcpp::Publisher<my_messages::msg::SteerMotor>::SharedPtr steer_odom_pub_;

//     // Subscriber
//     rclcpp::Subscription<my_messages::msg::SteerMotor>::SharedPtr steer_ang_sub_;

//     // Service Client
//     rclcpp::Client<dynamixel_sdk_custom_interfaces::srv::GetPosition>::SharedPtr steer_client_;

//     // Motor IDs
//     std::vector<int> motor_ids_ = {11, 12, 13}; // Example motor IDs

//     // Motor positions
//     std::vector<float> set_pos_;
//     std::vector<float> current_pos_;

//     // Methods
//     void timerCallback(); // Timer-based callback to request motor positions
//     void SteerAngleCallback(const my_messages::msg::SteerMotor::SharedPtr ang_msg); // Process angle input
//     void processResponse(int motor_id, int position); // Convert position to radians
// };

// SteerMotorNode::SteerMotorNode()
//     : Node("steer_motor_node"), set_pos_(motor_ids_.size(), 0), current_pos_(motor_ids_.size(), 0)
// {
//     // Initialize publishers
//     set_position_pub_ = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("set_position", 10);
//     steer_odom_pub_ = this->create_publisher<my_messages::msg::SteerMotor>("steer_odom", 10);

//     // Initialize subscriber
//     steer_ang_sub_ = this->create_subscription<my_messages::msg::SteerMotor>(
//         "steer_angle", 10, std::bind(&SteerMotorNode::SteerAngleCallback, this, std::placeholders::_1));

//     // Initialize service client
//     steer_client_ = this->create_client<dynamixel_sdk_custom_interfaces::srv::GetPosition>("get_position");

//     // Timer to request motor positions periodically
//     timer_ = this->create_wall_timer(
//         std::chrono::milliseconds(100), std::bind(&SteerMotorNode::timerCallback, this));

//     // Initialize positions to home positions
//     for (int i = 0; i < size; ++i) {
//         set_pos_[i] = home_pos[i];
//     }
// }

// SteerMotorNode::~SteerMotorNode()
// {
//     RCLCPP_INFO(this->get_logger(), "Shutting down SteerMotorNode");
// }

// void SteerMotorNode::timerCallback()
// {
//     if (!steer_client_->wait_for_service(std::chrono::seconds(1))) {
//         RCLCPP_WARN(this->get_logger(), "GetPosition service not available");
//         return;
//     }

//     for (size_t i = 0; i < motor_ids_.size(); ++i) {
//         auto request = std::make_shared<dynamixel_sdk_custom_interfaces::srv::GetPosition::Request>();
//         auto future = steer_client_->async_send_request(request);

//         // Process response
//         future.then([this, i](std::shared_future<std::shared_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition::Response>> response_future) {
//             try {
//                 auto response = response_future.get();
//                 processResponse(i, response->position); // Assuming position1 corresponds to motor i
//             } catch (const std::exception &e) {
//                 RCLCPP_ERROR(this->get_logger(), "Error retrieving motor position: %s", e.what());
//             }
//         });
//     }
// }

// void SteerMotorNode::SteerAngleCallback(const blower_motor::msg::SteerMotor::SharedPtr ang_msg)
// {
//     for (size_t i = 0; i < motor_ids_.size(); ++i) {
//         // Convert angle to target position
//         int target_position = home_pos[i] - static_cast<int>(ang_msg->phi / M_PI * 2048);

//         // Log the target position
//         RCLCPP_INFO(this->get_logger(), "Set Position: id %zu = %d", i, target_position);

//         // Publish target position
//         dynamixel_sdk_custom_interfaces::msg::SetPosition position_msg;
//         position_msg.id = motor_ids_[i];
//         position_msg.position = target_position;
//         set_position_pub_->publish(position_msg);
//     }
// }

// void SteerMotorNode::processResponse(int motor_id, int position)
// {
//     // Convert position to radians
//     current_pos_[motor_id] = M_PI * (position - home_pos[motor_id]) / 2048;

//     // Log the current position
//     RCLCPP_INFO(this->get_logger(), "Motor %d position: %f radians", motor_id + 1, current_pos_[motor_id]);

//     // Optionally publish or use this data for further processing
// }
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <my_messages/msg/steer_motor.hpp>
#include <cmath>
#include <dynamixel_sdk_custom_interfaces/msg/set_position.hpp>
#include <dynamixel_sdk_custom_interfaces/srv/get_position.hpp>
#include "motor_param.hpp"

class SteerMotorNode : public rclcpp::Node
{
public:
    SteerMotorNode();  // Constructor
    ~SteerMotorNode(); // Destructor

    // Setting position
    float set_pos[size];
    // Current position
    float current_pos[size];

    // Method
    void convertPositionRadian(std::shared_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition::Response> get_pos);

    // Callback
    void callback(const my_messages::msg::SteerMotor::SharedPtr ang_msg);

    // Publisher
    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr steer_set_pub;
    rclcpp::Publisher<my_messages::msg::SteerMotor>::SharedPtr steer_odom_pub;

    // Subscriber
    rclcpp::Subscription<my_messages::msg::SteerMotor>::SharedPtr steer_ang_sub;

    // Service Client
    rclcpp::Client<dynamixel_sdk_custom_interfaces::srv::GetPosition>::SharedPtr steer_client;
};

SteerMotorNode::SteerMotorNode()
    : Node("steer_motor_node")
{
    for (int i = 0; i < size; i++)
        set_pos[i] = home_pos[i];

    // Subscriber
    steer_ang_sub = this->create_subscription<my_messages::msg::SteerMotor>(
        "steer_angle", 10, std::bind(&SteerMotorNode::callback, this, std::placeholders::_1));

    // Publisher
    steer_set_pub = this->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>("set_position", 10);
    steer_odom_pub = this->create_publisher<my_messages::msg::SteerMotor>("steer_odom", 10);

    // Service Client
    steer_client = this->create_client<dynamixel_sdk_custom_interfaces::srv::GetPosition>("get_position");
}

SteerMotorNode::~SteerMotorNode()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down SteerMotorNode");
}

void SteerMotorNode::callback(const my_messages::msg::SteerMotor::SharedPtr ang_msg)
{
    // Convert Radian to Position
    set_pos[0] = home_pos[0] - int(ang_msg->phi1 / M_PI * 2048);
    set_pos[1] = home_pos[1] - int(ang_msg->phi2 / M_PI * 2048);
    set_pos[2] = home_pos[2] - int(ang_msg->phi3 / M_PI * 2048);

    RCLCPP_INFO(this->get_logger(), "Updated Set Position: [%d, %d, %d]",
                static_cast<int>(set_pos[0]),
                static_cast<int>(set_pos[1]),
                static_cast<int>(set_pos[2]));
}

void SteerMotorNode::convertPositionRadian(std::shared_ptr<dynamixel_sdk_custom_interfaces::srv::GetPosition::Response> get_pos)
{
    // Convert current position to radians
    current_pos[0] = M_PI * (get_pos->position1 - home_pos[0]) / 2048;
    current_pos[1] = M_PI * (get_pos->position2 - home_pos[1]) / 2048;
    current_pos[2] = M_PI * (get_pos->position3 - home_pos[2]) / 2048;

    RCLCPP_INFO(this->get_logger(), "Current Positions (radians): [%f, %f, %f]",
                current_pos[0], current_pos[1], current_pos[2]);
}
