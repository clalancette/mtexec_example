#include <chrono>
#include <iostream>
#include <memory>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class BaseNode final : public rclcpp::Node
{
public:
  BaseNode() : Node("base_node")
  {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom",
                                                                rmw_qos_profile_sensor_data);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&BaseNode::timerCallback, this));
  }

private:
  void timerCallback()
  {
    rcutils_time_point_value_t now;
    if (rcutils_system_time_now(&now) != RCUTILS_RET_OK) {
      std::cerr << "Failed to get now" << std::endl;
      return;
    }

    auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();

    odom_msg->header.stamp.sec = RCL_NS_TO_S(now);
    odom_msg->header.stamp.nanosec = now - RCL_S_TO_NS(odom_msg->header.stamp.sec);

    odom_msg->header.frame_id = "odom";
    odom_msg->child_frame_id = "base_link";
    odom_pub_->publish(odom_msg);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_unique<BaseNode>());
  rclcpp::shutdown();
  return 0;
}
