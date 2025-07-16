// #pragma once

// #include <rclcpp/rclcpp.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <tf2_ros/transform_broadcaster.h>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <geometry_msgs/msg/quaternion.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <cmath>
// #include <my_messages/msg/steer_motor.hpp>
// #include <my_messages/msg/drive_motor.hpp>
// #include <message_filters/subscriber.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include <message_filters/synchronizer.h>
// #include "robot_motor2/picking_robot_matrix.hpp"

// class PickingRobotOdom : public rclcpp::Node
// {
// public:
//     PickingRobotOdom();
//     ~PickingRobotOdom();
//     void calcOdometry();

//     // odometry pose information
//     double x;
//     double y;
//     double th;

//     // velocity twist information
//     double vx;
//     double vy;
//     double vth;

//     // odom frame names
//     std::string frame_id;
//     std::string child_frame_id;

//     // ROS time
//     rclcpp::Time current_time;
//     rclcpp::Time last_time;

// private:
//     void syncOdomCallback(const my_messages::msg::SteerMotor::SharedPtr steer_msg,
//                           const my_messages::msg::DriveMotor::SharedPtr drive_msg);

//     // Subscribers and synchronizer
//     std::shared_ptr<message_filters::Subscriber<my_messages::msg::SteerMotor>> steer_sub;
//     std::shared_ptr<message_filters::Subscriber<my_messages::msg::DriveMotor>> drive_sub;

//     typedef message_filters::sync_policies::ApproximateTime<my_messages::msg::SteerMotor,
//                                                             my_messages::msg::DriveMotor>
//         SyncOdomPolicy;
//     std::shared_ptr<message_filters::Synchronizer<SyncOdomPolicy>> odom_sync;

//     // Publisher and broadcaster
//     rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
//     std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

//     const int size;
//     PickingRobotMatrix kinema_matrix;
// };

// #pragma once

// #include <rclcpp/rclcpp.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <tf2_ros/transform_broadcaster.h>
// #include <cmath>
// #include <my_messages/msg/steer_motor.hpp>
// #include <my_messages/msg/drive_motor.hpp>
// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include "robot_motor2/picking_robot_matrix.hpp"
// #include <tf2/transform_datatypes.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


// using namespace my_messages::msg;

// class PickingRobotOdom : public rclcpp::Node
// {
// public:
//     PickingRobotOdom();
//     ~PickingRobotOdom();
//     void calcOdometry();

//     // odometry pose information
//     double x, y, th;

//     // velocity twist information
//     double vx, vy, vth;

//     // ros time
//     rclcpp::Time current_time;
//     rclcpp::Time last_time;

//     // odom frame names
//     std::string frame_id;
//     std::string child_frame_id;

//     rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

// private:
//     // Callback
//     void syncOdomCallback(const SteerMotor::SharedPtr steer_msg, const DriveMotor::SharedPtr drive_msg);

//     std::shared_ptr<message_filters::Subscriber<my_messages::msg::SteerMotor>> steer_sub;
//     std::shared_ptr<message_filters::Subscriber<my_messages::msg::DriveMotor>> drive_sub;

//     using SyncOdomPolicy = message_filters::sync_policies::ApproximateTime<my_messages::msg::SteerMotor, my_messages::msg::DriveMotor>;
//     std::shared_ptr<message_filters::Synchronizer<SyncOdomPolicy>> odomSync;

//     std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

//     // Param
//     const int size;

//     PickingRobotMatrix KinemaMatrix; // 運動学計算のためのクラス
// };

// PickingRobotOdom::PickingRobotOdom() :
//     Node("robot_odom_node"),
//     size(this->declare_parameter<int>("size", 3)),
//     x(0.0), y(0.0), th(0.0),
//     vx(0.0), vy(0.0), vth(0.0),
//     frame_id("odom"), child_frame_id("base_link")
// {
//     // // Initialize subscribers
//     // steer_sub = new message_filters::Subscriber<SteerMotor>(this, "steer_odom");
//     // drive_sub = new message_filters::Subscriber<DriveMotor>(this, "drive_odom");

//     // // Initialize synchronizer
//     // odomSync = std::make_shared<message_filters::Synchronizer<SyncOdomPolicy>>(SyncOdomPolicy(10), *steer_sub, *drive_sub);
//     // odomSync->registerCallback([this](const SteerMotor::SharedPtr steer_msg, const DriveMotor::SharedPtr drive_msg){this->syncOdomCallback(steer_msg, drive_msg);});

//     // // Initialize publisher and TF broadcaster
//     // odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);
//     // tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

//     // current_time = this->get_clock()->now();
//     // last_time = this->get_clock()->now();
//     // Initialize subscribers
//     steer_sub.subscribe(this, "steer_odom");
//     drive_sub.subscribe(this, "drive_odom");

//     // Initialize synchronizer
//     odomSync = std::make_shared<message_filters::Synchronizer<SyncOdomPolicy>>(SyncOdomPolicy(10), steer_sub, drive_sub);
//     odomSync->registerCallback([this](const my_messages::msg::SteerMotor::SharedPtr& steer_msg,
//                                        const my_messages::msg::DriveMotor::SharedPtr& drive_msg) {
//         this->syncOdomCallback(steer_msg, drive_msg);
//     });

//     // Initialize publisher and TF broadcaster
//     odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);
//     tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

//     current_time = this->get_clock()->now();
//     last_time = this->get_clock()->now();
// }

// PickingRobotOdom::~PickingRobotOdom()
// {
//     RCLCPP_INFO(this->get_logger(), "PickingRobotOdom node shutting down");
// }

// void PickingRobotOdom::syncOdomCallback(const SteerMotor::SharedPtr steer_msg, const DriveMotor::SharedPtr drive_msg)
// {
//     // std::array<float, 3> steer_phi;
//     // std::array<double, 3> drive_vel;

//     // steer_phi[0] = steer_msg->phi1;
//     // steer_phi[1] = steer_msg->phi2;
//     // steer_phi[2] = steer_msg->phi3;

//     // drive_vel[0] = drive_msg->vel1;
//     // drive_vel[1] = drive_msg->vel2;
//     // drive_vel[2] = drive_msg->vel3;
//     std::array<float, 3> steer_phi = {steer_msg->phi1, steer_msg->phi2, steer_msg->phi3};
//     std::array<double, 3> drive_vel = {drive_msg->vel1, drive_msg->vel2, drive_msg->vel3};


//     KinemaMatrix.calcRobotVelocity(vx, vy, vth, drive_vel.data(), steer_phi.data());
//     calcOdometry();
// }

// void PickingRobotOdom::calcOdometry()
// {
//     current_time = this->get_clock()->now();
//     double dt = (current_time - last_time).seconds();

//     double dx = (vx * cos(th) - vy * sin(th)) * dt;
//     double dy = (vx * sin(th) + vy * cos(th)) * dt;
//     double dth = vth * dt;

//     x += dx;
//     y += dy;
//     th += dth;

//     last_time = current_time;

//     // Publish Odometry
//     auto odom = nav_msgs::msg::Odometry();
//     odom.header.stamp = current_time;
//     odom.header.frame_id = frame_id;
//     odom.child_frame_id = child_frame_id;

//     odom.pose.pose.position.x = x;
//     odom.pose.pose.position.y = y;
//     odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, sin(th / 2), cos(th / 2)));

//     odom.twist.twist.linear.x = vx;
//     odom.twist.twist.linear.y = vy;
//     odom.twist.twist.angular.z = vth;

//     odom_pub->publish(odom);

//     // Broadcast TF
//     geometry_msgs::msg::TransformStamped tf_msg;
//     tf_msg.header.stamp = current_time;
//     tf_msg.header.frame_id = frame_id;
//     tf_msg.child_frame_id = child_frame_id;
//     tf_msg.transform.translation.x = x;
//     tf_msg.transform.translation.y = y;
//     tf_msg.transform.translation.z = 0.0;
//     tf_msg.transform.rotation = odom.pose.pose.orientation;

//     tf_broadcaster->sendTransform(tf_msg);
// }

// #pragma once

// #include <rclcpp/rclcpp.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <tf2_ros/transform_broadcaster.h>
// #include <cmath>
// #include <my_messages/msg/steer_motor.hpp>
// #include <my_messages/msg/drive_motor.hpp>
// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>
// #include "robot_motor2/picking_robot_matrix.hpp"

// using namespace my_messages;

// class PickingRobotOdom : public rclcpp::Node
// {
// public:
//     PickingRobotOdom();
//     ~PickingRobotOdom();
//     void calcOdometry();

//     // Odometry pose information
//     double x, y, th;

//     // Velocity twist information
//     double vx, vy, vth;

//     // Time
//     rclcpp::Time current_time, last_time;

//     // Frame names
//     std::string frame_id, child_frame_id;

//     // Publisher and TF Broadcaster
//     rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
//     std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;

// private:
//     // Callback
//     void syncOdomCallback(const my_messages::msg::SteerMotor::SharedPtr& steer_msg, 
//                           const my_messages::msg::DriveMotor::SharedPtr& drive_msg);

//     // Message Filters
//     message_filters::Subscriber<my_messages::msg::SteerMotor> steer_sub;
//     message_filters::Subscriber<my_messages::msg::DriveMotor> drive_sub;
    
//     typedef message_filters::sync_policies::ApproximateTime<my_messages::msg::SteerMotor, my_messages::msg::DriveMotor>;
//     message_filters::Synchronizer<SyncOdomPolicy> odomSync;

//     // Param
//     const int size;

//     PickingRobotMatrix KinemaMatrix; // 運動学計算クラス
// };

// PickingRobotOdom::PickingRobotOdom()
//     : Node("robot_odom_node"), 
//       size(3), x(0.0), y(0.0), th(0.0), 
//       vx(0.0), vy(0.0), vth(0.0),
//       frame_id("global"), child_frame_id("robot")
// {
//     // Initialize subscribers
//     steer_sub = std::make_shared<message_filters::Subscriber<my_messages::msg::SteerMotor>>(this, "steer_odom");
//     drive_sub = std::make_shared<message_filters::Subscriber<my_messages::msg::DriveMotor>>(this, "drive_odom");

//     // Initialize synchronizer
//     odomSync = std::make_shared<message_filters::Synchronizer<SyncOdomPolicy>>(SyncOdomPolicy(10), steer_sub, drive_sub);
//     // odomSync->registerCallback(std::bind(&PickingRobotOdom::syncOdomCallback, this, std::placeholders::_1, std::placeholders::_2));
//     // コールバックをラムダ式で登録
//     odomSync->registerCallback([this](const my_messages::msg::SteerMotor::ConstSharedPtr& steer_msg,
//                                       const my_messages::msg::DriveMotor::ConstSharedPtr& drive_msg) {
//         this->PickingRobotOdom::syncOdomCallback(steer_msg, drive_msg);
//     });
//     // Initialize publisher and TF broadcaster
//     odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);

//     current_time = this->get_clock()->now();
//     last_time = this->get_clock()->now();

//     RCLCPP_INFO(this->get_logger(), "PickingRobotOdom node initialized");
// }

// PickingRobotOdom::~PickingRobotOdom()
// {
//     RCLCPP_INFO(this->get_logger(), "Shutting down PickingRobotOdom");
// }

// void PickingRobotOdom::syncOdomCallback(const my_messages::msg::SteerMotor::ConstSharedPtr& steer_msg, 
//                                        const my_messages::msg::DriveMotor::ConstSharedPtr& drive_msg)
// {
//     std::array<float, 3> steer_phi = {steer_msg->phi1, steer_msg->phi2, steer_msg->phi3};
//     std::array<double, 3> drive_vel = {drive_msg->vel1, drive_msg->vel2, drive_msg->vel3};

//     KinemaMatrix.calcRobotVelocity(vx, vy, vth, drive_vel.data(), steer_phi.data());
//     calcOdometry();
// }

// void PickingRobotOdom::calcOdometry()
// {
//     current_time = this->get_clock()->now();
//     double dt = (current_time - last_time).seconds();

//     double dx = (vx * cos(th) - vy * sin(th)) * dt;
//     double dy = -(vx * sin(th) + vy * cos(th)) * dt;
//     double dth = vth * dt;

//     x += dx;
//     y += dy;
//     th += dth;

//     // last_time = current_time;

//     // // Publish Odometry
//     // auto odom = nav_msgs::msg::Odometry();
//     // odom.header.stamp = current_time;
//     // odom.header.frame_id = frame_id;
//     // odom.child_frame_id = child_frame_id;

//     // odom.pose.pose.position.x = x;
//     // odom.pose.pose.position.y = y;
//     // odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, sin(th / 2), cos(th / 2)));

//     // odom.twist.twist.linear.x = vx;
//     // odom.twist.twist.linear.y = vy;
//     // odom.twist.twist.angular.z = vth;

//     // odom_pub->publish(odom);

//     // // Broadcast TF
//     // geometry_msgs::msg::TransformStamped tf_msg;
//     // tf_msg.header.stamp = current_time;
//     // tf_msg.header.frame_id = frame_id;
//     // tf_msg.child_frame_id = child_frame_id;
//     // tf_msg.transform.translation.x = x;
//     // tf_msg.transform.translation.y = y;
//     // tf_msg.transform.translation.z = 0.0;
//     // tf_msg.transform.rotation = odom.pose.pose.orientation;

//     // odom_broadcaster->sendTransform(tf_msg);
// }
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <my_messages/msg/steer_motor.hpp>
#include <my_messages/msg/drive_motor.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "robot_motor2/picking_robot_matrix.hpp"

class PickingRobotOdom : public rclcpp::Node
{
public:
    PickingRobotOdom();
    ~PickingRobotOdom();
    void calcOdometry();

    // Odometry data
    double x, y, th;
    double vx, vy, vth;
    rclcpp::Time current_time, last_time;
    std::string frame_id, child_frame_id;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    // Subscribers and Synchronizer
    message_filters::Subscriber<my_messages::msg::SteerMotor> steer_sub;
    message_filters::Subscriber<my_messages::msg::DriveMotor> drive_sub;
    typedef message_filters::sync_policies::ApproximateTime<my_messages::msg::SteerMotor, my_messages::msg::DriveMotor> SyncOdomPolicy;
    message_filters::Synchronizer<SyncOdomPolicy> odomSync;
private:
    // Callback
    void syncOdomCallback(const my_messages::msg::SteerMotor::ConstSharedPtr& steer_msg,
                          const my_messages::msg::DriveMotor::ConstSharedPtr& drive_msg);
    const int size;

    // Kinematics calculation
    PickingRobotMatrix KinemaMatrix;
};
