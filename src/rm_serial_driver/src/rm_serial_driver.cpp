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
              data_buffer.push_back(header[0]);
              // std::cout << "header[0]" << static_cast<int>(header[0]) << std::endl;
              if (header[0] == 0xAA) {
                  // 如果检测到结束标识符（0xAA），则停止接收数据并处理
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

                      //std::cout<< "received from serial successfully "<< std::endl;
                      //std::cout<< packet.remaining_gold_coin<< std::endl;
                      to_decision_pub_->publish(msg);
                      receive_flag = false;
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
        uint16_t CRC_check = 0x0000;
        uint16_t CRC16_init = 0xFFFF;
        //sendpacket.nav_x = 1000;
        sendpacket.shangpo = true;
        //sendpacket.naving = true;
        CRC_check = crc16::Get_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&sendpacket), sizeof(sendpacket) - 2, CRC16_init);

        //std::cout << "CRC_check: " << CRC_check << std::endl;
        
        sendpacket.checksum = CRC_check;

        std::vector<uint8_t> data = toVector(sendpacket);

        serial_driver_->port()->send(data);
       //std::cout<<  sendpacket.nav_x << std::endl;
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
    sendpacket.nav_x = -cmd_vel.linear.y*5000;
    sendpacket.nav_y = cmd_vel.linear.x*5000;
    std::cout << sendpacket.nav_x << sendpacket.nav_y <<std::endl;
    
    //crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&sendpacket), sizeof(sendpacket));

}

void RMSerialDriver::decisionSendData(const rm_decision_interfaces::msg::ToSerial::SharedPtr msg)
{
    sendpacket.header = 0xA5;
    sendpacket.sentry_cmd = msg->sentry_cmd;
    
    //crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
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
