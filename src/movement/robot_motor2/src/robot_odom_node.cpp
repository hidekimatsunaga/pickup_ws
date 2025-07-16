// #include <rclcpp/rclcpp.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include "robot_motor2/robot_odom_node.hpp"

// int main(int argc, char **argv)
// {
//     // Initialize ROS 2
//     rclcpp::init(argc, argv);

//     // Create a node
//     auto node = std::make_shared<PickingRobotOdom>();

//     // Create a TF broadcaster
//     auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

//     // Create messages for odometry and transforms
//     geometry_msgs::msg::Quaternion odom_quat;
//     geometry_msgs::msg::TransformStamped odom_trans;
//     nav_msgs::msg::Odometry odom;

//     // Set frame IDs
//     // TFメッセージの初期化
//     odom_trans.header.stamp = node->current_time;
//     odom_trans.header.frame_id = node->frame_id;
//     odom_trans.child_frame_id = node->child_frame_id;

//     // Create a loop rate
//     rclcpp::Rate loop_rate(60);

//     // Main loop
//     while (rclcpp::ok())
//     {
//         node->current_time = node->get_clock()->now();

//         // Calculate and get odometry
//         node->calcOdometry();

//         // TF transform section -------------------------------------------------------
//         odom_trans.header.stamp = node->current_time;
//         odom_trans.transform.translation.x = node->x;
//         odom_trans.transform.translation.y = node->y;
//         odom_trans.transform.translation.z = 0.0;

//         // Create a quaternion from yaw
//         odom_quat = tf2::toMsg(tf2::Quaternion(0, 0, sin(node->th / 2), cos(node->th / 2)));
//         odom_trans.transform.rotation = odom_quat;

//         // Send the transform
//         tf_broadcaster->sendTransform(odom_trans);
//         // TF transform section end ---------------------------------------------------

//         // Odom section ---------------------------------------------------------------
//         odom.header.stamp = node->current_time;
//         odom.header.frame_id = node->frame_id;
//         odom.child_frame_id = node->child_frame_id;

//         // Set the position
//         odom.pose.pose.position.x = node->x;
//         odom.pose.pose.position.y = node->y;
//         odom.pose.pose.position.z = 0.0;
//         odom.pose.pose.orientation = odom_quat;

//         // Set the velocity
//         odom.twist.twist.linear.x = node->vx;
//         odom.twist.twist.linear.y = node->vy;
//         odom.twist.twist.angular.z = node->vth;

//         // Publish the odometry message
//         node->odom_pub->publish(odom);
//         // Odom section end -----------------------------------------------------------

//         node->last_time = node->current_time;

//         rclcpp::spin_some(node);
//         loop_rate.sleep();
//     }

//     rclcpp::shutdown();
//     return 0;
// }
#include "robot_motor2/robot_odom_node.hpp"

PickingRobotOdom::PickingRobotOdom()
    : Node("robot_odom_node"),
    //   steer_sub(this, "steer_odom"),
    //   drive_sub(this, "drive_odom"),
    //   odomSync(SyncOdomPolicy(10), steer_sub, drive_sub),
    //   x(0.0), y(0.0), th(0.0),
    //   vx(0.0), vy(0.0), vth(0.0),
    //   frame_id("odom"), child_frame_id("base_link")
    x(0.0), y(0.0), th(0.0),
    vx(0.0), vy(0.0), vth(0.0),
    current_time(this->get_clock()->now()),
    last_time(this->get_clock()->now()),
    frame_id("global"),
    child_frame_id("robot"),
    steer_sub(this, "steer_odom"),
    drive_sub(this, "drive_odom"),
    odomSync(SyncOdomPolicy(10), steer_sub, drive_sub),
    size(3),
    KinemaMatrix()    
{
    odomSync.registerCallback(std::bind(&PickingRobotOdom::syncOdomCallback, this, std::placeholders::_1, std::placeholders::_2));

    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("wheel_odom", 10);
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    current_time = this->get_clock()->now();
    last_time = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(), "PickingRobotOdom node initialized");
}

PickingRobotOdom::~PickingRobotOdom()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down PickingRobotOdom");
}

void PickingRobotOdom::syncOdomCallback(const my_messages::msg::SteerMotor::ConstSharedPtr& steer_msg,
                                        const my_messages::msg::DriveMotor::ConstSharedPtr& drive_msg)
{
    std::array<float, 3> steer_phi = {steer_msg->phi1, steer_msg->phi2, steer_msg->phi3};
    std::array<double, 3> drive_vel = {drive_msg->vel1, drive_msg->vel2, drive_msg->vel3};

    KinemaMatrix.calcRobotVelocity(vx, vy, vth, drive_vel.data(), steer_phi.data());
    calcOdometry();
}

void PickingRobotOdom::calcOdometry()
{
    current_time = this->get_clock()->now();
    double dt = (current_time - last_time).seconds();

    double dx = (vx * cos(th) - vy * sin(th)) * dt;
    double dy = (vx * sin(th) + vy * cos(th)) * dt;
    double dth = vth * dt;

    x += dx;
    y += dy;
    th += dth;

    last_time = current_time;

    // Publish odometry
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = current_time;
    odom.header.frame_id = frame_id;
    odom.child_frame_id = child_frame_id;

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, sin(th / 2), cos(th / 2)));
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.angular.z = vth;

    odom_pub->publish(odom);

    // Publish TF
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = current_time;
    tf_msg.header.frame_id = frame_id;
    tf_msg.child_frame_id = child_frame_id;
    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom.pose.pose.orientation;

    tf_broadcaster->sendTransform(tf_msg);
}

int main(int argc, char **argv)
{
    // ROS 2の初期化
    rclcpp::init(argc, argv);

    // PickingRobotOdomノードを作成
    auto node = std::make_shared<PickingRobotOdom>();

    // ノードのスピン
    rclcpp::spin(node);

    // ROS 2の終了処理
    rclcpp::shutdown();
    return 0;
}