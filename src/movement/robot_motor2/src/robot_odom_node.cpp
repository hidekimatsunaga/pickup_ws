#include "robot_motor2/robot_odom_node.hpp"

PickingRobotOdom::PickingRobotOdom()
    : Node("robot_odom_node"),
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
    KinemaMatrix(),
    is_first_callback(true)    
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
    // メッセージのタイムスタンプを取得
    rclcpp::Time msg_time = steer_msg->header.stamp;
    
    RCLCPP_INFO(this->get_logger(), "Callback triggered! steer_time=%f, drive_time=%f", 
                steer_msg->header.stamp.sec + steer_msg->header.stamp.nanosec * 1e-9,
                drive_msg->header.stamp.sec + drive_msg->header.stamp.nanosec * 1e-9);
    
    std::array<float, 3> steer_phi = {steer_msg->phi1, steer_msg->phi2, steer_msg->phi3};
    std::array<double, 3> drive_vel = {drive_msg->vel1, drive_msg->vel2, drive_msg->vel3};

    RCLCPP_INFO(this->get_logger(), "drive_vel=[%.3f, %.3f, %.3f], steer_phi=[%.3f, %.3f, %.3f]", 
                drive_vel[0], drive_vel[1], drive_vel[2], 
                steer_phi[0], steer_phi[1], steer_phi[2]);

    KinemaMatrix.calcRobotVelocity(vx, vy, vth, drive_vel.data(), steer_phi.data());
    
    RCLCPP_INFO(this->get_logger(), "Robot velocity: vx=%.3f, vy=%.3f, vth=%.3f", vx, vy, vth);
    
    calcOdometry(msg_time);
}

void PickingRobotOdom::calcOdometry(const rclcpp::Time& current_time)
{
    // current_time = this->get_clock()->now();
    double dt = (current_time - last_time).seconds();

    RCLCPP_INFO(this->get_logger(), "calcOdometry: dt=%.6f, is_first=%d", dt, is_first_callback);

    // 初回コールバック時は dt をリセット（ノード初期化のギャップをスキップ）
    if (is_first_callback) {
        RCLCPP_INFO(this->get_logger(), "First callback - initializing last_time");
        last_time = current_time;
        is_first_callback = false;
        return;
    }

    // dtが極端に小さい、またはマイナスになる場合をスキップ
    // ロボット操作の現実を考慮して、dt の上限を緩和（初期値をスキップ後）
    if (dt <= 0.0 || dt > 10.0) {
        RCLCPP_WARN(this->get_logger(), "Skipping odom update: dt=%.6f (out of range)", dt);
        last_time = current_time;
        return;
    }

    // オドメトリ計算：ロボット座標系での速度を グローバル座標系に変換して積分
    // グローバルフレーム = "global", ロボットフレーム = "robot"
    // 計算式：
    //   dx = (vx*cos(th) - vy*sin(th)) * dt   [グローバル座標系の x 変位]
    //   dy = (vx*sin(th) + vy*cos(th)) * dt   [グローバル座標系の y 変位]
    //   dth = vth * dt                         [角度変位]
    // ここで vx, vy は ロボット座標系の速度、th は ロボットの向き角度

    double dx = (vx * cos(th) - vy * sin(th)) * dt;
    double dy = (vx * sin(th) + vy * cos(th)) * dt;
    double dth = vth * dt;

    x += dx;
    y += dy;
    th += dth;

    last_time = current_time;

    RCLCPP_INFO(this->get_logger(), "Publishing odom: x=%.3f, y=%.3f, th=%.3f (dx=%.3f, dy=%.3f, dth=%.3f)", 
                x, y, th, dx, dy, dth);

    // Publish odometry
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = current_time;
    odom.header.frame_id = frame_id;
    odom.child_frame_id = child_frame_id;

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, th);
    odom.pose.pose.orientation = tf2::toMsg(q);

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
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