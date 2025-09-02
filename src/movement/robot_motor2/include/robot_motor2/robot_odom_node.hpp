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
    void calcOdometry(const rclcpp::Time& current_time);

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
