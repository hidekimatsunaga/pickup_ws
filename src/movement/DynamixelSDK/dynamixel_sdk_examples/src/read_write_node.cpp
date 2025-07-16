// // Copyright 2021 ROBOTIS CO., LTD.
// // 
// // Licensed under the Apache License, Version 2.0 (the "License");
// // you may not use this file except in compliance with the License.
// // You may obtain a copy of the License at
// //
// //     http://www.apache.org/licenses/LICENSE-2.0
// //
// // Unless required by applicable law or agreed to in writing, software
// // distributed under the License is distributed on an "AS IS" BASIS,
// // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// // See the License for the specific language governing permissions and
// // limitations under the License.

// /*******************************************************************************
// // This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
// // For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
// // To test this example, please follow the commands below.
// //
// // Open terminal #1
// // $ ros2 run dynamixel_sdk_examples read_write_node
// //
// // Open terminal #2 (run one of below commands at a time)
// // $ ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/SetPosition "{id: 1, position: 1000}"
// // $ ros2 service call /get_position dynamixel_sdk_custom_interfaces/srv/GetPosition "id: 1"
// //
// // Author: Will Son
// *******************************************************************************/

// /******************************************************************************/
// /* include                                                                    */
// /******************************************************************************/
// #include <cstdio>
// #include <memory>
// #include <string>

// #include "dynamixel_sdk/dynamixel_sdk.h"
// #include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"
// #include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
// #include "rclcpp/rclcpp.hpp"
// #include "rcutils/cmdline_parser.h"

// #include "read_write_node.hpp"
// // #include "cmd_vel_to_set_position.hpp"

// /******************************************************************************/
// /* define                                                                     */
// /******************************************************************************/
// /* Control table address for X series                                         */
// #define ADDR_OPERATING_MODE       11
// #define ADDR_TORQUE_ENABLE				64
// #define ADDR_GOAL_CURRENT		  		102	 // Does NOT exist in Rot motors
// #define ADDR_GOAL_VELOCITY				104
// #define MAX_VELOCITY_LIMIT        1   // モーターの最大速度（適宜調整）
// #define ADDR_GOAL_POSITION				116
// #define ADDR_PRESENT_CURRENT			126	 // Represents "Present Load" in Rot motors
// #define ADDR_PRESENT_VELOCITY			128
// #define ADDR_PRESENT_POSITION			132

// /* Motors ID */
// #define DXL1_ID                         11
// #define DXL2_ID                         12
// #define DXL3_ID                         13
// // #define DXL4_ID                         14

// /* TORQUE ENABLE/DISABLE */
// #define TORQUE_ENABLE                   1	 // Value for enabling the torque
// #define TORQUE_DISABLE                  0	 // Value for disabling the torque

// /* OPERATING_MODE */
// #define CURRENT_BASED_POSITION_CONTROL 5
// #define POSITION_CONTROL				       5
// #define VELOCITY_CONTROL				       1
// #define TORQUE_CONTROL					       0

// /* Protocol version */ 
// #define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

// /* Default setting */
// #define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
// #define DEVICE_NAME "/dev/ttyUSB0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

// dynamixel::PortHandler * portHandler;
// dynamixel::PacketHandler * packetHandler;

// uint8_t dxl_error = 0;
// uint32_t goal_position = 0;
// int dxl_comm_result = COMM_TX_FAIL;

// /******************************************************************************/
// /* Constructor                                                                */
// /******************************************************************************/
// ReadWriteNode::ReadWriteNode()
// : Node("read_write_node")
// {
//   RCLCPP_INFO(this->get_logger(), "Run read write node");

//   this->declare_parameter("qos_depth", 10);
//   int8_t qos_depth = 0;
//   this->get_parameter("qos_depth", qos_depth);

//   const auto QOS_RKL10V =
//     rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

//   set_position_subscriber_ =
//     this->create_subscription<SetPosition>(
//     "set_position",
//     QOS_RKL10V,
//     [this](const SetPosition::SharedPtr msg) -> void
//     {
//       uint8_t dxl_error = 0;

//       // Position Value of X series is 4 byte data.
//       // For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
//       uint32_t goal_position = (unsigned int)msg->position;  // Convert int32 -> uint32

//       // Write Goal Position (length : 4 bytes)
//       // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
//       dxl_comm_result =
//       packetHandler->write4ByteTxRx(
//         portHandler,
//         (uint8_t) msg->id,
//         ADDR_GOAL_POSITION,
//         goal_position,
//         &dxl_error
//       );

//       if (dxl_comm_result != COMM_SUCCESS) {
//         RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
//       } else if (dxl_error != 0) {
//         RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
//       } else {
//         RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", msg->id, msg->position);
//       }
//     }
//     );

//   auto get_present_position =
//     [this](
//     const std::shared_ptr<GetPosition::Request> request,
//     std::shared_ptr<GetPosition::Response> response) -> void
//     {
//       // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
//       // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
//       dxl_comm_result = packetHandler->read4ByteTxRx(
//         portHandler,
//         (uint8_t) request->id,
//         ADDR_PRESENT_POSITION,
//         reinterpret_cast<uint32_t *>(&present_position),
//         &dxl_error
//       );

//       RCLCPP_INFO(
//         this->get_logger(),
//         "Get [ID: %d] [Present Position: %d]",
//         request->id,
//         present_position
//       );

//       response->position = present_position;
//     };

//   get_position_server_ = create_service<GetPosition>("get_position", get_present_position);
// }

// ReadWriteNode::~ReadWriteNode()
// {
// }

// /******************************************************************************/
// /* Function                                                                   */
// /******************************************************************************/
// void setupDynamixel(uint8_t dxl_id)
// {
//   /* Use Position Control Mode                */
//   /* #define CURRENT_BASED_POSITION_CONTROL 5 */
//   /* #define POSITION_CONTROL				        3 */
//   /* #define VELOCITY_CONTROL				        1 */
//   /* #define TORQUE_CONTROL					        0 */
//   dxl_comm_result = packetHandler->write1ByteTxRx(
//     portHandler,
//     dxl_id,
//     ADDR_OPERATING_MODE,
//     POSITION_CONTROL,
//     &dxl_error
//   );

//   if (dxl_comm_result != COMM_SUCCESS) {
//     RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set Position Control Mode.");
//   } else {
//     RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set Position Control Mode.");
//   }

//   // Enable Torque of DYNAMIXEL
//   dxl_comm_result = packetHandler->write1ByteTxRx(
//     portHandler,
//     dxl_id,
//     ADDR_TORQUE_ENABLE,
//     TORQUE_ENABLE,  /* Torque ON */
//     &dxl_error
//   );

//   if (dxl_comm_result != COMM_SUCCESS) {
//     RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to enable torque.");
//   } else {
//     RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to enable torque.");
//   }
//     // モーターの速度制限を設定
//   dxl_comm_result = packetHandler->write4ByteTxRx(
//       portHandler,
//       dxl_id,
//       ADDR_GOAL_VELOCITY,
//       MAX_VELOCITY_LIMIT,
//       &dxl_error
//   );

//   if (dxl_comm_result != COMM_SUCCESS) {
//       RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set velocity limit for ID: %d", dxl_id);
//   } else {
//       RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Set velocity limit for ID: %d to %d", dxl_id, MAX_VELOCITY_LIMIT);
//   }

// }

// int main(int argc, char * argv[])
// {
//   portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
//   packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

//   // Open Serial Port
//   dxl_comm_result = portHandler->openPort();
//   if (dxl_comm_result == false) {
//     RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to open the port!");
//     return -1;
//   } else {
//     RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to open the port.");
//   }

//   // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
//   dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
//   if (dxl_comm_result == false) {
//     RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set the baudrate!");
//     return -1;
//   } else {
//     RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set the baudrate.");
//   }

//   setupDynamixel(BROADCAST_ID);
//   std::vector<uint8_t> motor_ids = {DXL1_ID, DXL2_ID, DXL3_ID};
//   for (auto id : motor_ids) {
//     setupDynamixel(id);
//   }
//   rclcpp::init(argc, argv);

//   auto readwritenode = std::make_shared<ReadWriteNode>();
//   rclcpp::spin(readwritenode);

//   // Disable Torque of DYNAMIXEL
//   packetHandler->write1ByteTxRx(
//     portHandler,
//     BROADCAST_ID,
//     ADDR_TORQUE_ENABLE,
//     TORQUE_DISABLE,
//     &dxl_error
//   );
//   portHandler->closePort();
	
// 	rclcpp::shutdown();

//   return 0;
// }
// // int main(int argc, char *argv[])
// // {
// //   // ROS 2の初期化
// //   rclcpp::init(argc, argv);

// //   // PortHandlerとPacketHandlerの初期化
// //   portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
// //   packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

// //   // シリアルポートを開く
// //   dxl_comm_result = portHandler->openPort();
// //   if (dxl_comm_result == false) {
// //     RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to open the port!");
// //     return -1;
// //   } else {
// //     RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to open the port.");
// //   }

// //   // ボーレートを設定
// //   dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
// //   if (dxl_comm_result == false) {
// //     RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set the baudrate!");
// //     return -1;
// //   } else {
// //     RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set the baudrate.");
// //   }

// //   // 複数のモーターを初期化
// //   std::vector<int> motor_ids = {11, 12, 13}; // 必要に応じて変更
// //   for (const auto &id : motor_ids) {
// //     setupDynamixel(id);
// //   }

// //   // ROSノードの作成
// //   auto node = std::make_shared<CmdVelToSetPosition>();

// //   // ノードをスピン
// //   rclcpp::spin(node);

// //   // Dynamixelのトルクを無効化
// //   for (const auto &id : motor_ids) {
// //     packetHandler->write1ByteTxRx(
// //       portHandler,
// //       id,
// //       ADDR_TORQUE_ENABLE,
// //       TORQUE_DISABLE,  /* Torque OFF */
// //       &dxl_error
// //     );
// //   }

// //   // ポートを閉じる
// //   portHandler->closePort();

// //   // ROS 2のシャットダウン
// //   rclcpp::shutdown();

// //   return 0;
// // }
#include "read_write_node.hpp"

/* Define Control Table Addresses and Constants */
#define ADDR_OPERATING_MODE       11
#define ADDR_TORQUE_ENABLE        64
#define ADDR_GOAL_POSITION        116
#define ADDR_PRESENT_POSITION     132
#define PROTOCOL_VERSION          2.0
#define BAUDRATE                  57600
#define DEVICE_NAME               "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT4TCWV6-if00-port0"
#define TORQUE_ENABLE             1
#define TORQUE_DISABLE            0

/* Global variables for Port and Packet Handlers */
dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

/******************************************************************************/
/* Constructor                                                                */
/******************************************************************************/
ReadWriteNode::ReadWriteNode()
: Node("read_write_node")
{
  RCLCPP_INFO(this->get_logger(), "Initializing ReadWriteNode");

  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open the port
  if (!portHandler->openPort()) {
    RCLCPP_FATAL(this->get_logger(), "Failed to open the port!");
    throw std::runtime_error("Failed to open the port!");
  }

  // Set baudrate
  if (!portHandler->setBaudRate(BAUDRATE)) {
    RCLCPP_FATAL(this->get_logger(), "Failed to set the baudrate!");
    throw std::runtime_error("Failed to set the baudrate!");
  }

  // Initialize Dynamixel motors
  for (const auto &id : motor_ids_) {
    setupDynamixel(id);
  }

  // Subscriber for set_position
  set_position_subscriber_ = this->create_subscription<SetPosition>(
    "set_position", 10,
    [this](const SetPosition::SharedPtr msg) { handle_set_position(msg); });

  // Service for get_position
  get_position_server_ = this->create_service<GetPosition>(
    "get_position",
    [this](
      const std::shared_ptr<GetPosition::Request> request,
      std::shared_ptr<GetPosition::Response> response) { handle_get_position(request, response); });

  RCLCPP_INFO(this->get_logger(), "Node Initialized");
}

/******************************************************************************/
/* Destructor                                                                 */
/******************************************************************************/
ReadWriteNode::~ReadWriteNode()
{
  for (const auto &id : motor_ids_) {
    packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, nullptr);
  }
  portHandler->closePort();
}

/******************************************************************************/
/* Private Methods                                                            */
/******************************************************************************/
void ReadWriteNode::setupDynamixel(uint8_t dxl_id)
{
  uint8_t dxl_error = 0;

  // Set to position control mode
  if (packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, 4, &dxl_error) != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to set Position Control Mode for ID: %d", dxl_id);
  }

  // Enable torque
  if (packetHandler->write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error) != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to enable torque for ID: %d", dxl_id);
  }
}

void ReadWriteNode::handle_set_position(const SetPosition::SharedPtr msg)
{
  uint8_t dxl_error = 0;

    // Set goal positions for each motor
  uint32_t positions[3] = {
    static_cast<uint32_t>(msg->position1),
    static_cast<uint32_t>(msg->position2),
    static_cast<uint32_t>(msg->position3)
  };
  for (size_t i = 0; i < 3; ++i) {
    if (packetHandler->write4ByteTxRx(portHandler, motor_ids_[i], ADDR_GOAL_POSITION, positions[i], &dxl_error) != COMM_SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set position for ID: %d", motor_ids_[i]);
    } else {
      RCLCPP_INFO(this->get_logger(), "Set Position for ID: %d -> %d", motor_ids_[i], positions[i]);
    }
  }
}

void ReadWriteNode::handle_get_position(
  const std::shared_ptr<GetPosition::Request> request,
  std::shared_ptr<GetPosition::Response> response)
{
  int32_t positions[3];
  for (size_t i = 0; i < 3; ++i) {
    positions[i] = read_present_position(motor_ids_[i]);
  }
  response->position1 = positions[0];
  response->position2 = positions[1];
  response->position3 = positions[2];

  RCLCPP_INFO(this->get_logger(), "Returned Positions: [%d, %d, %d]", positions[0], positions[1], positions[2]);
}

int32_t ReadWriteNode::read_present_position(uint8_t dxl_id)
{
  uint8_t dxl_error = 0;
  uint32_t present_position = 0;

  if (packetHandler->read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION, &present_position, &dxl_error) != COMM_SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to read position for ID: %d", dxl_id);
    return -1;
  }
  return static_cast<int32_t>(present_position);
}

/******************************************************************************/
/* Main Function                                                              */
/******************************************************************************/
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<ReadWriteNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("read_write_node"), "Exception: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}