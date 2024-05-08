#include <functional>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>


#include <memory>
#include <iostream>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <string>
#include <unistd.h>
#include <vector>
#include <fstream>
#include <cmath>
#include "rm_decision/commander.hpp"

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>

using namespace std::chrono_literals;

namespace rm_decision
{

   Commander::Commander(const rclcpp::NodeOptions & options) : Node("commander",options)
   {
      RCLCPP_INFO(this->get_logger(), "Commander node has been started.");

      //创建客户端
      nav_to_pose_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this,"navigate_to_pose");
      send_goal_options.goal_response_callback = std::bind(&Commander::goal_response_callback, this, std::placeholders::_1);
      send_goal_options.feedback_callback = std::bind(&Commander::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
      send_goal_options.result_callback = std::bind(&Commander::result_callback, this, std::placeholders::_1);
      //初始化状态
      loadNavPoints();
      RCLCPP_INFO(this->get_logger(), "导航点个数");
      currentpose.header.frame_id = "base_link";
      currentpose.pose.position.x = 1.0;
      currentpose.pose.position.y = 0.0;
      currentpose.pose.position.z = 0.0;
      currentpose.pose.orientation.x = 0.0;
      currentpose.pose.orientation.y = 0.0;
      currentpose.pose.orientation.z = 0.0;
      currentpose.pose.orientation.w = 1.0;
      move =move_points_.begin();
      attack = Route3_points_.begin();
      goal = currentpose;
      tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
      sleep(10);
      RCLCPP_INFO(this->get_logger(), "开始");
      currentState = std::make_shared<WaitState>(this);

      nav_sub_ = this->create_subscription<rm_decision_interfaces::msg::FromSerial>(
         "fromjudge", 10, std::bind(&Commander::serial_callback, this, std::placeholders::_1));
      aim_sub_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
         "/tracker/target", rclcpp::SensorDataQoS(),std::bind(&Commander::aim_callback, this, std::placeholders::_1));
      enemypose_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
         "/tracker/enemypose", 10,std::bind(&Commander::enemypose_callback, this, std::placeholders::_1));
      laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
              "scan", 10,std::bind(&Commander::laserScanCallback, this, std::placeholders::_1));
      sentry_cmd_pub_ = this->create_publisher<rm_decision_interfaces::msg::ToSerial>("sentry/cmd", 10);

      // 创建线程（处理信息和发布命令）
      commander_thread_ = std::thread(&Commander::decision, this);
      executor_thread_ = std::thread(&Commander::executor, this);
      self_cmd_thread_ = std::thread(&Commander::self_cmd, this);
   }
   // 析构函数
   Commander::~Commander(){
      if(commander_thread_.joinable()){
         commander_thread_.join();
      }
         
      if(executor_thread_.joinable()){
         executor_thread_.join();
      }
   }





   // 处理信息线程(设置状态）(now using behave tree)
   void Commander::decision(){
        rclcpp::Rate r(5);
        // behavetree init
        BT::BehaviorTreeFactory factory;
        factory.registerSimpleCondition("wait_for_start", std::bind(&Commander::wait_for_start, this));
        factory.registerSimpleCondition("dafu_ordered", std::bind(&Commander::dafu_ordered, this));
        factory.registerSimpleCondition("outpose_ordered", std::bind(&Commander::outpose_ordered, this));
        factory.registerSimpleCondition("base_ordered", std::bind(&Commander::base_ordered, this));
        factory.registerSimpleCondition("IfAddHp", std::bind(&Commander::IfAddHp, this));
        factory.registerSimpleCondition("IfDefend", std::bind(&Commander::IfDefend, this));
        factory.registerSimpleCondition("IfAttack", std::bind(&Commander::IfAttack, this));
        factory.registerSimpleCondition("IfGuard", std::bind(&Commander::IfGuard, this));

        factory.registerSimpleAction("dafu_handle", std::bind(&Commander::dafu_handle, this));
        factory.registerSimpleAction("outpose_handle", std::bind(&Commander::outpose_handle, this));
        factory.registerSimpleAction("base_handle", std::bind(&Commander::base_handle, this));
        factory.registerSimpleAction("addhp_handle", std::bind(&Commander::addhp_handle, this));
        factory.registerSimpleAction("defend_handle", std::bind(&Commander::defend_handle, this));
        factory.registerSimpleAction("attack_handle", std::bind(&Commander::attack_handle, this));
        factory.registerSimpleAction("Guard", std::bind(&Commander::Guard_handle, this));
        factory.registerSimpleAction("MoveAround", std::bind(&Commander::MoveAround_handle, this));

      auto tree = factory.createTreeFromFile("./src/rm_decision/rm_decision/config/sentry_bt.xml"); //official
      // auto tree = factory.createTreeFromFile("./rm_decision/config/sentry_bt.xml");  //for debug
      BT::Groot2Publisher publisher(tree);
       while (rclcpp::ok())
      { 
         std::cout << "behavetree is working now" << std::endl;
         tree.tickWhileRunning();
         r.sleep();
         if(dafu){
            std::thread([this] {
            std::this_thread::sleep_for(std::chrono::minutes(1));
            std::cout << "dafu is set to false" << std::endl;
            dafu = false;}).detach();
            }
         if(outpose){
            std::thread([this] {
            std::this_thread::sleep_for(std::chrono::minutes(1));
            std::cout << "outpose is set to false" << std::endl;
            outpose = false;}).detach();
            }
      }

   }


   // 执行器线程
   void Commander::executor(){
      rclcpp::Rate r(5);
      while (rclcpp::ok())
      {  
         getcurrentpose();
         currentState->handle();
         RCLCPP_INFO(this->get_logger(), "time: %lf",time);
         r.sleep();
      }
   }

   // 发布命令线程
    void Commander::self_cmd(){
      rclcpp::Rate r(1);
      rm_decision_interfaces::msg::ToSerial msg;
      // msg.sentry_cmd |= (1 << 0); set one bit to 1
      // msg.sentry_cmd &= ~(1 << 0);      set one bit to 0


      while(rclcpp::ok()){
         if(self_hp == 0){
            msg.sentry_cmd |= (1 << 0);
         }
         if(self_hp == 0 && goldcoin > 500){
            msg.sentry_cmd |= (1 << 1);
         }
         if(self_ammo < 50 && goldcoin > 500){
            buy_ammo = 200;
            for (int i = 2; i <= 12; ++i) {
            msg.sentry_cmd &= ~(1 << i);
            }
            msg.sentry_cmd |= (buy_ammo << 2);
         }
         if(self_ammo < 50 && goldcoin < 300 && goldcoin > 100){
            buy_ammo = 100;
            for (int i = 2; i <= 12; ++i) {
            msg.sentry_cmd &= ~(1 << i);
            }
            msg.sentry_cmd |= (buy_ammo << 2);
         }
         if(self_ammo < 50 && goldcoin < 150 && goldcoin > 50){
            buy_ammo = 50;
            for (int i = 2; i <= 12; ++i) {
            msg.sentry_cmd &= ~(1 << i);
            }
            msg.sentry_cmd |= (buy_ammo << 2);
         }
         if(self_hp < 100 && goldcoin > 300){
            buy_hp ++;
            msg.sentry_cmd |= (buy_hp << 17);
         }
         sentry_cmd_pub_->publish(msg);
         r.sleep();
      }
   
    }

   // 改变状态
   void Commander::setState(std::shared_ptr<State> state) {
      currentState = state;
   }
   
   
   // 导航到点   
   void Commander::nav_to_pose(geometry_msgs::msg::PoseStamped goal_pose){
      nav_to_pose_client->wait_for_action_server();
      auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
      goal_msg.pose = goal_pose;
      goal_msg.behavior_tree = "";
      send_goal_future = nav_to_pose_client->async_send_goal(goal_msg,send_goal_options);
   }

   // 请求反馈
   void Commander::goal_response_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future){
      auto goal_handle = future.get();
      if (!goal_handle) {
         RCLCPP_INFO(this->get_logger(),"Goal was rejected by server");
         checkgoal = true;
         return;
      }
      else{
         RCLCPP_INFO(this->get_logger(),"Goal accepted by server, waiting for result");
      }
   }
   
   // 过程反馈
   void Commander::feedback_callback(
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future,
  const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback
   ){
      // RCLCPP_INFO(this->get_logger(),"Received feedback: 去往目标点的距离: %.2f m",feedback->distance_remaining);
   }
   
   // 结果反馈
   void Commander::result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result){
      switch (result.code) {
         case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(),"Goal was reached!");
            nav_state = 1;
            checkgoal = true;
            break;
         case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_INFO(this->get_logger(),"Goal was aborted");
            nav_state = 2;
            checkgoal = true;
            break;
         case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(this->get_logger(),"Goal was canceled");
            nav_state = 3;
            checkgoal = true;
            break;
         default:
            RCLCPP_INFO(get_logger(),"Unknown result code");
            nav_state = 4;
            checkgoal = true;
            break;
      }
   }
   // 读取导航点
   void Commander::processPoses(std::vector<double>& pose_param,std::vector<geometry_msgs::msg::PoseStamped>& nav_points_) {
      for(uint i = 0; i < pose_param.size(); i=i+3){
         geometry_msgs::msg::PoseStamped pose;
         pose.header.frame_id ="map";
         pose.pose.position.x = pose_param[i];
         pose.pose.position.y = pose_param[i+1];
         pose.pose.position.z = pose_param[i+2];
         pose.pose.orientation.x = 0.0;
         pose.pose.orientation.y = 0.0;          
         pose.pose.orientation.z = 0.0;         
         pose.pose.orientation.w = 1.0;
         nav_points_.push_back(pose);
         RCLCPP_INFO(this->get_logger(), "传入第%d个导航点: %.2f, %.2f, %.2f",(i+3)/3,pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
      }
   }
   // 加载导航点
   void Commander::loadNavPoints() {
      RCLCPP_INFO(this->get_logger(), "开始传入导航点");
      geometry_msgs::msg::PoseStamped pose;
      std::vector<double> pose_list;
      std::vector<std::string> route_list = {"route1","route2","route3","route4"};
      std::vector<std::vector<geometry_msgs::msg::PoseStamped>>::iterator list = list_name.begin();
      this->declare_parameter("home_pose", pose_list);
      //如果战术要求可以读取多条路径
      home.header.frame_id ="map";
      home.header.stamp = this->now();
      home.pose.position.x = this->get_parameter("home_pose").as_double_array()[0];
      home.pose.position.y = this->get_parameter("home_pose").as_double_array()[1];
      home.pose.position.z = this->get_parameter("home_pose").as_double_array()[2];
      for(auto it = route_list.begin(); it != route_list.end(); it++){
         this->declare_parameter(*it, pose_list);
         auto pose_param = this->get_parameter(*it).as_double_array();
         processPoses(pose_param,*list);
         RCLCPP_INFO(this->get_logger(), "%s随机导航点个数: %ld",it->c_str(),(*list).size());
         list ++;
      }
      Patrol_points_ = list_name.at(0);
      random = Patrol_points_.begin();
      Route3_points_ = list_name.at(2);
      attack = Route3_points_.begin();
      move_points_ = list_name.at(1);
      move = Route2_points_.begin();
      
   }
   // 加载敌军hp
   uint Commander::enemyhp() {
      switch (enemy_num) {
         case 1:
            return enemy_1.hp;
         case 2:
            return enemy_2.hp;
         case 3:
            return enemy_3.hp;
         case 4:
            return enemy_4.hp;
         case 5:
            return enemy_5.hp;
      }
      return 600;
   }


   
   // 订阅回调
   void Commander::serial_callback(const rm_decision_interfaces::msg::FromSerial::SharedPtr msg) {
      gamestart = msg->gamestart;
      color = msg->color;
      self_ammo = msg->projectile_allowance_17mm;
      goldcoin = msg->remaining_gold_coin;

      if(msg->color == 1){
         self_hp = msg->red_7;
         self_base = msg->red_base_hp;
         self_outpost = msg->red_outpost_hp;
      }
      else{
         self_hp = msg->blue_7;
         self_base = msg->blue_base_hp;
         self_outpost = msg->blue_outpost_hp;
      }

      if(msg->supply_robot_id == 4){
         if(msg->supply_projectile_num == 50){
            outpose = true;
         }
         if (msg->supply_projectile_num == 100){
            dafu = true;
         }
         if(msg->supply_projectile_num == 200){
            base = true;
         }
      }
   }

   void Commander::aim_callback(const auto_aim_interfaces::msg::Target::SharedPtr msg) {
      //RCLCPP_INFO(this->get_logger(), "自瞄回调");
      tracking = msg->tracking;
      enemy_num = msg->armors_num;
   }

   void Commander::enemypose_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
      //RCLCPP_INFO(this->get_logger(), "自瞄回调");
         enemypose.header.stamp = this->now();
         enemypose.header.frame_id = "map";
         enemypose.pose.position.x = msg->point.x;
         enemypose.pose.position.y = msg->point.y;
         enemypose.pose.position.z = msg->point.z;
         RCLCPP_INFO(this->get_logger(), "敌方位置: %.2f, %.2f, %.2f",enemypose.pose.position.x ,enemypose.pose.position.y,enemypose.pose.position.z);
   }

    //取距
    double Commander::distence(const geometry_msgs::msg::PoseStamped a){
        double dis = sqrt(pow(a.pose.position.x , 2) + pow(a.pose.position.y, 2));
        return dis;
    }

    void Commander::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan){

    }



   // 巡逻模式
   void PatrolState::handle() {
      if(commander->checkgoal){
         commander->nav_to_pose(*commander->random);
         commander->random++;
         if(commander->random == commander->Patrol_points_.end()){
            commander->random = commander->Patrol_points_.begin();
         }
         commander->checkgoal = false;
      }
   }

   // goandstay模式(可用于守卫某点或逃跑等)
   void GoAndStayState::handle() {
      if(commander->checkgoal){
         commander->nav_to_pose(commander->goal);
         commander->checkgoal = false;
      }
   }

   // 追击模式
   void AttackState::handle() {
      commander->nav_to_pose(commander->enemypose);
      RCLCPP_INFO(commander->get_logger(), "敌方距离: %.2f,开始追击",commander->distence(commander->enemypose));
   }
   // 等待模式
   void WaitState::handle() {
      
   }
   
   //振荡模式
   void MoveState::handle() {
      if(commander->checkgoal){
         commander->nav_to_pose(*commander->move);
         commander->move++;
         if(commander->move == commander->move_points_.end()){
            commander->move = commander->move_points_.begin();
         }
         commander->checkgoal = false;
      }
   }

   void CjState::handle() {
      if(commander->checkgoal){
         commander->nav_to_pose(*commander->attack);
         commander->attack++;
         if(commander->attack == commander->Route3_points_.end()){
            commander->attack = commander->Route3_points_.begin();
         }
         commander->checkgoal = false;
      }
   }


   




   //timer
   // double Commander::timer(){
   // if(packet.game_progress == 0x24         && !first_start){
   //    first_start = true;
   //    startTime = std::chrono::steady_clock::now();
   // }
   // if(first_start){
   //    std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
   //    std::chrono::duration<double> dur = currentTime - startTime;
   //    double dur_sec = dur.count();
   //    return dur_sec;
   // }
   // else{
   //    return 400;
   // }
   // }


   //联盟赛获取自身坐标
   void Commander::getcurrentpose(){
      geometry_msgs::msg::TransformStamped odom_msg;
      try {
          odom_msg = tf2_buffer_->lookupTransform(
            "map", "livox_frame",
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform : %s",
             ex.what());
          return;
        }
      currentpose.header.stamp = this->now();
      currentpose.header.frame_id = "map";
      currentpose.pose.position.x = odom_msg.transform.translation.x;
      currentpose.pose.position.y = odom_msg.transform.translation.y;
      currentpose.pose.position.z = odom_msg.transform.translation.z;
      currentpose.pose.orientation = odom_msg.transform.rotation;
      RCLCPP_INFO(this->get_logger(), "当前位置: %.2f, %.2f, %.2f",currentpose.pose.position.x,currentpose.pose.position.y,currentpose.pose.position.z);
   }

} // namespace rm_decision
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_decision::Commander)
