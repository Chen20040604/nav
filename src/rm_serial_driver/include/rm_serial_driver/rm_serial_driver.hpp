// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <rclcpp/create_subscription.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>

// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>


//导航的自定义消息类型
#include "rm_decision_interfaces/msg/from_serial.hpp"
#include "rm_decision_interfaces/msg/to_serial.hpp"


#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include "packet.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
namespace rm_serial_driver
{
class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);

  ~RMSerialDriver() override;
  bool receive_flag=true;
  SendPacket sendpacket;

private:
  void getParams();

  void receiveData();

  void navSendData(const geometry_msgs::msg::Twist& cmd_vel);

  void decisionSendData(const rm_decision_interfaces::msg::ToSerial::SharedPtr msg);

  void pathSendData(const nav_msgs::msg::Path::SharedPtr msg);

  void sendData();

  void reopenPort();

  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  float x2 = 0.0;
  float y2 = 0.0;
  float x3 = 0.0;
  float y3 = 0.0;
  float a1 = 0.0;
  float b1 = 0.0;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
  rclcpp::Subscription<rm_decision_interfaces::msg::ToSerial>::SharedPtr from_decision_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

  rclcpp::Publisher<rm_decision_interfaces::msg::FromSerial>::SharedPtr to_decision_pub_;

  std::thread receive_thread_;
  std::thread send_thread_;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster;

  int closecount=0;
  
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
