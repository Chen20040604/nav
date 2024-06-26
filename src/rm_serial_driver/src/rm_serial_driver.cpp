// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include <tf2/LinearMath/Quaternion.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// C++ system
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");

  getParams();
  // tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  // //gamemap下的两点坐标
  // double x0 = 5.0;
  // double y0 = 5.0;
  // double x1 = 5.0;
  // double y1 = 5.0;
  // //check 点坐标
  // double a = 5.0;
  // double b = 5.0;
  // //标定
  // double yaw = atan((y3 - y2) / (x3 - x2)) - atan((y1 - y0) / (x1 - x0));
  // double dx = x2 - x0*cos(yaw) + y0*sin(yaw);
  // double dy = y2 - x0*sin(yaw) - y0*cos(yaw);

  // geometry_msgs::msg::TransformStamped static_transform;
  // static_transform.header.frame_id = "map";
  // static_transform.child_frame_id = "gamemap";
  // static_transform.transform.translation.x = dx;
  // static_transform.transform.translation.y = dy;
  // static_transform.transform.translation.z = 0.0;
  // tf2::Quaternion quat;
  // quat.setRPY(0.0, 0.0, yaw); // Roll, Pitch, Yaw
  // static_transform.transform.rotation.x = quat.x();
  // static_transform.transform.rotation.y = quat.y();
  // static_transform.transform.rotation.z = quat.z();
  // static_transform.transform.rotation.w = quat.w();
  // static_broadcaster->sendTransform(static_transform);

  // // check transform
  // geometry_msgs::msg::PoseStamped pose_stamped;
  // pose_stamped.header.frame_id = "gamemap";
  // pose_stamped.pose.position.x = a;
  // pose_stamped.pose.position.y = b;
  // pose_stamped.pose.position.z = 0.0;
  // pose_stamped.pose.orientation.x = 0.0;
  // pose_stamped.pose.orientation.y = 0.0;
  // pose_stamped.pose.orientation.z = 0.0;
  // pose_stamped.pose.orientation.w = 1.0;
  // geometry_msgs::msg::PoseStamped check_pose;
  // try
  // {
  //   tf_buffer_->transform(pose_stamped, check_pose, "map");
  // }
  // catch (tf2::TransformException & ex)
  // {
  //   RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
  //   return;
  // }
  // double error_dis = sqrt(pow(check_pose.pose.position.x - a1, 2) + pow(check_pose.pose.position.y - b1, 2));
  // RCLCPP_INFO(this->get_logger(), "Transform error: %f", error_dis);

  // Create Publisher
  to_decision_pub_ = this->create_publisher<rm_decision_interfaces::msg::FromSerial>("fromjudge", 10);



    try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveData, this);
      send_thread_ = std::thread(&RMSerialDriver::sendData, this);
      closecount++;
      //std::cout << "count : " << closecount << std::endl; 
      if(closecount == 10)
      {
        closecount=0;
        exit(0);
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
    closecount++;
    if(closecount == 10)
      {
        //std::cout << "count : " << closecount << std::endl; 
        closecount=0;
        exit(0);
      }
  }


  // Create Subscription
  nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>
          ("cmd_vel", rclcpp::SensorDataQoS(),
           std::bind(&RMSerialDriver::navSendData, this, std::placeholders::_1));
  from_decision_sub_ = this->create_subscription<rm_decision_interfaces::msg::ToSerial>
          ("sentry/cmd",rclcpp::SensorDataQoS(),
           std::bind(&RMSerialDriver::decisionSendData,this,std::placeholders::_1));
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>("path", rclcpp::SensorDataQoS(),
           std::bind(&RMSerialDriver::pathSendData,this,std::placeholders::_1));

}

RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (send_thread_.joinable())
  {
    send_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}
  

void RMSerialDriver::receiveData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivePacket));
  bool receiving_data = false;  // 用于跟踪是否正在接收数据
  std::vector<uint8_t> data_buffer;  // 用于存储接收的数据
  ReceivePacket packet;
  uint16_t CRC16_init = 0xFFFF;
  uint16_t CRC_check = 0x0000;

  while (rclcpp::ok()) {
    //std::cout<< "C"<<receive_flag<< std::endl;
      try {
          // 这一行从串行端口接收一个字节的数据，将其存储在 header 向量中
      serial_driver_->port()->receive(header);

          if (receiving_data) {
              // 如果正在接收数据，将数据添加到缓冲区

              //RCLCPP_INFO(this->get_logger(), "now receiveing");

              data_buffer.push_back(header[0]);
              // std::cout << "header[0]" << static_cast<int>(header[0]) << std::endl;
              if (header[0] == 0xAA) {
                  // 如果检测到结束标识符（0xAA），则停止接收数据并处理
                  //RCLCPP_INFO(this->get_logger(), "finished receiveing");

                  receiving_data = false;
                  // for(int i = 0; i < static_cast<int>(data_buffer.size()); i++)
                  // {
                  //     //int a = int(data_buffer[i])；
                  //     std::cout << "data_buffer[" << i << "]:" << data_buffer[i] << std::endl;
                  //     //std::cout << "data_buffer[" << i << "]:" << std::hex <<std::uppercase <<a <<std::endl;
                  // }
                  // 处理接收到的数据
                  //std::cout<< "sizes: " << data_buffer.size() << std::endl;
                  //std::cout<< "size: " << sizeof(ReceivePacket) + 1 << std::endl;
                  if (data_buffer.size() == sizeof(ReceivePacket) + 1){
                      packet = fromVector(data_buffer);

                      CRC_check = crc16::Get_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet)-2,CRC16_init);

                      //std::cout<< "CRC_check: " << CRC_check << std::endl;
                      //std::cout<< "packet.checksum: " << packet.checksum << std::endl;
                      if(CRC_check == packet.checksum)
                      {
                        // 执行您的操作，例如设置参数、发布消息等
                        rm_decision_interfaces::msg::FromSerial msg;
                        //导航
                        msg.gamestart = packet.gamestart;
                        msg.color = packet.color;

                        msg.projectile_allowance_17mm = packet.projectile_allowance_17mm;
                        msg.remaining_gold_coin = packet.remaining_gold_coin;

                        msg.supply_robot_id = packet.supply_robot_id;
                        msg.supply_projectile_num = packet.supply_projectile_num;

                        msg.red_7 = packet.red_7;
                        msg.red_outpost_hp = packet.red_outpost_HP;
                        msg.red_base_hp = packet.red_base_HP;

                        msg.blue_7 = packet.blue_7;
                        msg.blue_outpost_hp = packet.blue_outpost_HP;
                        msg.blue_base_hp = packet.blue_base_HP;

                        // //transfrom to map
                        // geometry_msgs::msg::PoseStamped pose_stamped;
                        // pose_stamped.header.frame_id = "gamemap";
                        // pose_stamped.pose.position.x = packet.target_pos_x;
                        // pose_stamped.pose.position.y = packet.target_pos_y;
                        // pose_stamped.pose.position.z = 0.0;
                        // pose_stamped.pose.orientation.x = 0.0;
                        // pose_stamped.pose.orientation.y = 0.0;
                        // pose_stamped.pose.orientation.z = 0.0;
                        // pose_stamped.pose.orientation.w = 1.0;
                        // geometry_msgs::msg::PoseStamped goal_pose;
                        // try
                        // {
                        //   tf_buffer_->transform(pose_stamped, goal_pose, "map");
                        // }
                        // catch (tf2::TransformException &ex)
                        // {
                        //   RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
                        //   return;
                        // }
                        // msg.target_pos_x = goal_pose.pose.position.x;
                        // msg.target_pos_y = goal_pose.pose.position.y;
                        // msg.cmd_key = packet.cmd_key;

                      //std::cout<< "received from serial successfully "<< std::endl;
                      //std::cout<< packet.remaining_gold_coin<< std::endl;
                      to_decision_pub_->publish(msg);
                      bigyaw = packet.big_yaw;
                      receive_flag = false;
                      
                      //RCLCPP_INFO(this->get_logger(), "自身金币: %d color: %d, gamestart: %d",msg.remaining_gold_coin,msg.color,msg.gamestart);

                      }
                      else
                      {
                      std::cout<< "bad "<< std::endl;
                      }
                  }
                  //}
                  // 清空数据缓冲区
                  data_buffer.clear();
                  //header.clear();
                 // std::cout<< "bad1 "<< std::endl;
              }
          } else if (header[0] == 0x5A) {
              // 如果检测到开始标识符（0x5A），开始接收数据
              receiving_data = true;
              data_buffer.push_back(header[0]);
              //header.clear();
          }
      } catch (const std::exception & ex) {
          RCLCPP_ERROR_THROTTLE(
              get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
          reopenPort();
      }
  }

}

void RMSerialDriver::sendData()
{
  
  while (rclcpp::ok()) {
    
    if(receive_flag){
      continue;
    }
    else{
      try {
        //RCLCPP_INFO(this->get_logger(), "trying to send data");
        uint16_t CRC_check = 0x0000;
        uint16_t CRC16_init = 0xFFFF;
        //sendpacket.nav_x = 1000;
        //sendpacket.shangpo = true;
        //sendpacket.naving = true;
        CRC_check = crc16::Get_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&sendpacket), sizeof(sendpacket) - 2, CRC16_init);

        std::cout << "CRC_check: " << CRC_check << std::endl;
        std::cout <<  sizeof(sendpacket) - 2 << std::endl;
        
        sendpacket.checksum = CRC_check;

        std::vector<uint8_t> data = toVector(sendpacket);

        serial_driver_->port()->send(data);
       std::cout<<  sendpacket.shangpo << std::endl;
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
        reopenPort();
      }
      receive_flag = true;
    }
    
  }
}

void RMSerialDriver::navSendData(const geometry_msgs::msg::Twist& cmd_vel)
{


    sendpacket.header = 0xA5;
    sendpacket.naving = true;
    sendpacket.nav_x = -cmd_vel.linear.y*10000;
    sendpacket.nav_y = cmd_vel.linear.x*10000;
    // std::cout << sendpacket.nav_x << sendpacket.nav_y <<std::endl;
    
    //crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&sendpacket), sizeof(sendpacket));

}

void RMSerialDriver::decisionSendData(const rm_decision_interfaces::msg::ToSerial::SharedPtr msg)
{
    sendpacket.header = 0xA5;
    sendpacket.sentry_cmd = msg->sentry_cmd;
    sendpacket.goal_yaw = fmod(msg->diff_yaw + bigyaw +180 , 360) - 180;
    sendpacket.shangpo = msg->shangpo;
    
    //crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
}

void RMSerialDriver::pathSendData(const nav_msgs::msg::Path::SharedPtr msg)
{
  std::vector<geometry_msgs::msg::PoseStamped> transformed_poses;
  // int size = 50;
  // sendpacket.intention = 3;
  // sendpacket.sender_id = 0;
  // // 转换每个位姿
  // for (auto &pose_stamped : msg->poses)
  // {
  //   geometry_msgs::msg::PoseStamped transformed_pose;

  //   // 尝试进行转换
  //   try
  //   {
  //     tf_buffer_->transform(pose_stamped, transformed_pose, "gamemap");
  //   }
  //   catch (tf2::TransformException &ex)
  //   {
  //     RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
  //     return;
  //   }
  //   transformed_poses.push_back(transformed_pose);
  // }
  // sendpacket.start_pos_x = static_cast<unsigned int>(std::round(transformed_poses.front().pose.position.x * 10));
  // sendpacket.start_pos_y = static_cast<unsigned int> (std::round(transformed_poses.front().pose.position.y * 10));
  // if (transformed_poses.size() > 50)
  // {
  //   size = 50;
  // }
  // else
  // {
  //   size = transformed_poses.size();
  // }
  // for(int i = 0; i < size; i++)
  // {
  //   sendpacket.d_x[i] = static_cast<int> (std::round(transformed_poses[i + 1].pose.position.x * 10 - transformed_poses[i].pose.position.x * 10));
  //   sendpacket.d_y[i] = static_cast<int> (std::round(transformed_poses[i + 1].pose.position.y * 10 - transformed_poses[i].pose.position.y * 10));
  // }
}

void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  x2 = declare_parameter<float>("x2", 0.0);
  y2 = declare_parameter<float>("y2", 0.0);
  x3 = declare_parameter<float>("x3", 0.0);
  y3 = declare_parameter<float>("y4", 0.0);
  a1 = declare_parameter<float>("a1", 0.0);
  b1 = declare_parameter<float>("b1", 0.0);

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
      closecount++;
      //std::cout << "count : " << closecount << std::endl; 
      if(closecount == 0)
      {
        //std::cout << "count : " << closecount << std::endl; 
        closecount=0;
        exit(0);
      }
    }
    serial_driver_->port()->open();
      closecount++;
      //std::cout << "count : " << closecount << std::endl; 
      if(closecount == 0)
      {
        //std::cout << "count : " << closecount << std::endl; 
        closecount=0;
        exit(0);
      }
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    closecount++;
    std::cout << "count : " << closecount << std::endl; 
    if(closecount == 0)
    {
      std::cout << "count : " << closecount << std::endl; 
      closecount=0;
      exit(0);
    }
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}



}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
