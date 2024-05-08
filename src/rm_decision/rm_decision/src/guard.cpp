#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "rm_decision/commander.hpp"
class Nav2ExampleNode : public rclcpp::Node
{
public:
    Nav2ExampleNode() : Node("nav2_example_node")
    {
        // 创建导航客户端
        nav_client_ = create_client<nav2_msgs::action::NavigateToPose>("navigate_to_pose");
        clear_costmap_service_ = create_client<nav2_msgs::srv::ClearEntireCostmap>("clear_entire_costmap");

        // 订阅激光扫描数据
        laser_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", 10, std::bind(&Nav2ExampleNode::laserScanCallback, this, std::placeholders::_1));

        // 声明定时器
        timer_ = create_wall_timer(2s, std::bind(&Nav2ExampleNode::navigateToRandomPose, this));
    }

private:
    void navigateToRandomPose()
    {
        // 生成随机目标点坐标
        double target_x = generateRandomCoordinate();
        double target_y = generateRandomCoordinate();

        // 检查障碍物
        if (isObstacleDetected(target_x, target_y))
        {
            RCLCPP_WARN(get_logger(), "Obstacle detected at target point (%f, %f). Generating new target point.", target_x,
                        target_y);
            return;
        }

        // 创建导航目标点
        auto goal = nav2_msgs::action::NavigateToPose::Goal();
        goal.pose.pose.position.x = target_x;
        goal.pose.pose.position.y = target_y;
        goal.pose.header.frame_id = "map";

        // 发送导航目标点请求
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback = std::bind(&Nav2ExampleNode::goalResultCallback, this, std::placeholders::_1);
        auto goal_handle_future = nav_client_->async_send_goal(goal, send_goal_options);

        // 等待结果
        if (rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(get_logger(), "Failed to send goal.");
            return;
        }
    }

    void goalResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(get_logger(), "Goal reached!");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to reach goal.");
        }
    }

    bool isObstacleDetected(double x, double y) {
        // 根据机器人的激光扫描数据或环境地图，检查目标点是否与障碍物冲突
        // 这里简化为假设目标点附近范围内的任何障碍物均会导致冲突
        // 您需要根据实际情况修改此处的逻辑
        double obstacle_threshold = 0.5;  // 障碍物检测阈值

        // 假设机器人当前位置为(0, 0)
        double robot_x = 0.0;
        double robot_y = 0.0;

        // 计算目标点与机器人的距离
        double distance = std::sqrt(std::pow(x - robot_x, 2) + std::pow(y - robot_y, 2));

        if (distance < obstacle_threshold) {
            return true;  // 存在障碍物
        }

        return false;  // 无障碍物
        // ...
        }
        void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
        {
            // 在此处处理激光扫描数据
            // 您可以使用激光扫描数据来更新障碍物信息，例如生成环境地图或检测障碍物位置等
            // 这里仅作为示例，不进行具体的处理逻辑
        }

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    };

