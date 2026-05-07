#ifndef SMARTCAR_GOAL_CPP__GO_TO_POSE_HPP_
#define SMARTCAR_GOAL_CPP__GO_TO_POSE_HPP_

#include <memory>
#include <string>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace smartcar_goal_cpp
{

class GoToPoseNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  GoToPoseNode();

  bool sendGoal();

private:
  std::string destination_;
  double target_x_;
  double target_y_;
  double target_yaw_;
  std::string frame_id_;
  std::string action_name_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
};

}  // namespace smartcar_goal_cpp

#endif  // SMARTCAR_GOAL_CPP__GO_TO_POSE_HPP_
