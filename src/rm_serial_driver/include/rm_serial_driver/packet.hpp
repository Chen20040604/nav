// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>
#include <iostream>

namespace rm_serial_driver
{
struct ReceivePacket
{
  uint8_t header = 0x5A;
  //导航
  bool gamestart;  //比赛开始 1 for start
  uint8_t color;   //改为int 哨兵颜色 　1 red 2 blue
  
  uint16_t projectile_allowance_17mm; //允许发弹量
  uint16_t remaining_gold_coin;  //剩余金币数

  uint8_t supply_robot_id;  //补弹机器人ID
  uint8_t supply_projectile_num; //补弹量
  
  uint16_t red_7;  //哨兵
  uint16_t red_outpost_HP; //前哨站
  uint16_t red_base_HP;  //基地
  
  uint16_t blue_7;
  uint16_t blue_outpost_HP;  
  uint16_t blue_base_HP;
  
  uint16_t checksum = 0;     // crc16校验位 
} __attribute__((packed));

struct SendPacket
{
  uint8_t header = 0xA5;
   
  bool naving;
  bool shangpo;

  float nav_x;
  float nav_y;
  uint32_t sentry_cmd;  //哨兵发给裁判系统的自主决策命令，如选择复活，买弹等，见裁判系统
    
  uint16_t checksum = 0;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
{
  ReceivePacket packet;
  // for(int i = 0; i < static_cast<int>(data.size()); i++)
  //                 {
  //                     //int a = int(data_buffer[i])；
  //                     std::cout << "data[" << i << "]:" << data[i] << std::endl;
  //                     //std::cout << "data_buffer[" << i << "]:" << std::hex <<std::uppercase <<a <<std::endl;
  //                 }
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket & data)
{
  std::vector<uint8_t> packet(sizeof(SendPacket));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
  return packet;
}

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
