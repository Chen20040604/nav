#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class Nav2ExampleNode : public rclcpp::Node
{
public:

    explicit Commander(const rclcpp::NodeOptions & options); // 重载构造函数

    ~Commander() override; // 析构函数

private:
    void navigateToRandomPose();

    void goalResultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result);

    bool isObstacleDetected(double x, double y);
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
};

