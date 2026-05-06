#ifndef SMART_CAR_CPP_PKG__GO_TO_POSE_HPP_
#define SMART_CAR_CPP_PKG__GO_TO_POSE_HPP_

#include <memory>
#include <string>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace smart_car_cpp_pkg
{

class GoToPoseNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  GoToPoseNode();

  bool sendGoal();

private:
  double target_x_;
  double target_y_;
  double target_yaw_;
  std::string frame_id_;
  std::string action_name_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
};

}  // namespace smart_car_cpp_pkg

#endif  // SMART_CAR_CPP_PKG__GO_TO_POSE_HPP_
