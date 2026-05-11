#ifndef SMARTCAR_GOAL_CPP__GO_TO_POSE_HPP_
#define SMARTCAR_GOAL_CPP__GO_TO_POSE_HPP_

#include <memory>
#include <string>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

namespace smartcar_goal_cpp
{

class GoToPoseNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  GoToPoseNode();

  bool sendGoal(const std::string & destination);

private:
  void commandCallback(const std_msgs::msg::String::SharedPtr msg);
  std::string normalizeDestination(const std::string & command) const;

  bool navigation_enabled_;
  std::string destination_;
  double target_x_;
  double target_y_;
  double target_yaw_;
  std::string frame_id_;
  std::string action_name_;
  std::string command_topic_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
};

}  // namespace smartcar_goal_cpp

#endif  // SMARTCAR_GOAL_CPP__GO_TO_POSE_HPP_
