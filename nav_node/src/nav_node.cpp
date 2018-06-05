#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "gtg_msgs/srv/go_to_goal_pose.hpp"

class NavNode final : public rclcpp::Node
{
public:
  NavNode() : Node("nav_node")
  {
    cb_grp1_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
    cb_grp2_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);

    go_to_goal_pose_srv_ = this->create_service<gtg_msgs::srv::GoToGoalPose>("go_to_goal_pose",
                                                                             std::bind(&NavNode::goToGoalPose,
                                                                                       this,
                                                                                       std::placeholders::_1,
                                                                                       std::placeholders::_2),
                                                                             rmw_qos_profile_services_default,
                                                                             cb_grp1_);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom",
                                                                   std::bind(&NavNode::odomCallback,
                                                                             this,
                                                                             std::placeholders::_1),
                                                                   rmw_qos_profile_sensor_data,
                                                                   cb_grp2_);
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::cerr << "Got odom data and notified thread " << std::this_thread::get_id() << std::endl;
  }

  void goToGoalPose(const std::shared_ptr<gtg_msgs::srv::GoToGoalPose::Request> request,
                    std::shared_ptr<gtg_msgs::srv::GoToGoalPose::Response> response)
  {
    std::cerr << "Starting goToGoalPose" << std::endl;
    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
    while (1) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();

      auto diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
      if (diff_ms.count() > 5000) {
        break;
      }
    }

    std::cerr << "Finished goToGoalPose" << std::endl;
    response->status = 0;
    response->msg = "MOOOOOO";
  }

  rclcpp::Service<gtg_msgs::srv::GoToGoalPose>::SharedPtr go_to_goal_pose_srv_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::callback_group::CallbackGroup::SharedPtr cb_grp1_;
  rclcpp::callback_group::CallbackGroup::SharedPtr cb_grp2_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto nav = std::make_shared<NavNode>();
  executor.add_node(nav);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
