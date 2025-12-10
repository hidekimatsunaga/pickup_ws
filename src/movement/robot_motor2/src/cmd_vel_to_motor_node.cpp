// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/joy.hpp>
// #include "robot_motor/msg/steer_motor.hpp"
// #include <my_messages/msg/drive_vel.hpp>
#include "robot_motor2/cmd_vel_to_motor_node.hpp"
// #include <dynamixel_sdk_custom_interfaces/srv/get_position.hpp>
// #include <my_messages/msg/steermotor.hpp>

int main(int argc, char **argv)
{
    // ROS2ノードの初期化
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdvelToMotorNode>();

    // for steer motor position publish
    auto steer = my_messages::msg::SteerMotor();
    // for right rear drive motor velocity publish
    auto drive = my_messages::msg::DriveMotor();

    // Service client for steer position
    auto steer_client = node->create_client<dynamixel_sdk_custom_interfaces::srv::GetPosition>("get_position");

    // CmdvelToMotorNodeオブジェクトの初期化
    CmdvelToMotorNode CmdToMotorNode;

    // サービスリクエストの準備
    auto get_pos_request = std::make_shared<dynamixel_sdk_custom_interfaces::srv::GetPosition::Request>();
    get_pos_request->id1 = 11;
    get_pos_request->id2 = 12;
    get_pos_request->id3 = 13;

    // 0.2秒の待機（スリープ）
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    // ループのレートを60Hzに設定
    rclcpp::Rate loop(100);

    while (rclcpp::ok())
    {
        // set steer motor position
        steer.phi1 = CmdToMotorNode.steer_phi[0];
        steer.phi2 = CmdToMotorNode.steer_phi[1];
        steer.phi3 = CmdToMotorNode.steer_phi[2];

        // set drive motor velocity
        drive.vel1 = CmdToMotorNode.drive_vel[0];
        drive.vel2 = CmdToMotorNode.drive_vel[1];
        drive.vel3 = CmdToMotorNode.drive_vel[2];

        CmdToMotorNode.steer_ang_pub->publish(steer);
        RCLCPP_INFO(node->get_logger(), "Updated steer_phi in callback: [%f, %f, %f]", CmdToMotorNode.steer_phi[0], CmdToMotorNode.steer_phi[1], CmdToMotorNode.steer_phi[2]);
        CmdToMotorNode.drive_vel_pub->publish(drive);
        RCLCPP_INFO(node->get_logger(), "Published drive_vel: [%f, %f, %f]", drive.vel1, drive.vel2, drive.vel3);

        // RCLCPP_INFO(this->get_logger(), "Received cmd_vel: vx=%f, vy=%f, vth=%f", vx, vy, vth);

        // サービス呼び出し
        if (steer_client->wait_for_service(std::chrono::seconds(1)))
        {
            auto future = steer_client->async_send_request(get_pos_request);
            if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = future.get();
                // diff_posを計算する関数
                CmdToMotorNode.calcDiffPos(response);
            }
            else
            {
                RCLCPP_ERROR(node->get_logger(), "Failed to call service get_position");
            }
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Service not available");
        }

        // Publish
        // CmdToMotorNode.steer_ang_pub->publish(steer);
        // NavToMotor.right_rear_vel_pub->publish(right_rear_drive);
        // NavToMotor.left_vel_pub->publish(left_drive);

        rclcpp::spin_some(node);
        loop.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
