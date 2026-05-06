#include "smart_car_cpp_pkg/go_to_pose.hpp"

#include <chrono>
#include <cmath>
#include <memory>

using namespace std::chrono_literals;

namespace smart_car_cpp_pkg
{

GoToPoseNode::GoToPoseNode()
: Node("go_to_pose_cpp")
{
  target_x_ = declare_parameter<double>("target_x", 1.8955986499786377);
  target_y_ = declare_parameter<double>("target_y", -0.02082836627960205);
  target_yaw_ = declare_parameter<double>("target_yaw", 0.0);
  frame_id_ = declare_parameter<std::string>("frame_id", "map");
  action_name_ = declare_parameter<std::string>("action_name", "navigate_to_pose");

  action_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);
}

bool GoToPoseNode::sendGoal()
{
  RCLCPP_INFO(
    get_logger(),
    "Waiting for Nav2 action server '%s'...",
    action_name_.c_str());

  if (!action_client_->wait_for_action_server(30s)) {
    RCLCPP_ERROR(
      get_logger(),
      "Nav2 action server is not available. Start TurtleBot3 Nav2 first.");
    return false;
  }

  NavigateToPose::Goal goal_msg;
  goal_msg.pose.header.frame_id = frame_id_;
  goal_msg.pose.header.stamp = now();
  goal_msg.pose.pose.position.x = target_x_;
  goal_msg.pose.pose.position.y = target_y_;
  goal_msg.pose.pose.position.z = 0.0;

  const double half_yaw = target_yaw_ * 0.5;
  goal_msg.pose.pose.orientation.x = 0.0;
  goal_msg.pose.pose.orientation.y = 0.0;
  goal_msg.pose.pose.orientation.z = std::sin(half_yaw);
  goal_msg.pose.pose.orientation.w = std::cos(half_yaw);

  RCLCPP_INFO(
    get_logger(),
    "Sending goal: frame=%s x=%.3f y=%.3f yaw=%.3f rad",
    frame_id_.c_str(), target_x_, target_y_, target_yaw_);

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.goal_response_callback =
    [this](const GoalHandleNavigateToPose::SharedPtr & goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(get_logger(), "Goal was rejected by Nav2.");
        rclcpp::shutdown();
        return;
      }
      RCLCPP_INFO(get_logger(), "Goal accepted by Nav2.");
    };

  options.feedback_callback =
    [this](
      GoalHandleNavigateToPose::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Distance remaining: %.3f m",
        feedback->distance_remaining);
    };

  options.result_callback =
    [this](const GoalHandleNavigateToPose::WrappedResult & result) {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(get_logger(), "Goal reached.");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(get_logger(), "Goal was aborted.");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(get_logger(), "Goal was canceled.");
          break;
        default:
          RCLCPP_ERROR(get_logger(), "Unknown result code.");
          break;
      }
      rclcpp::shutdown();
    };

  action_client_->async_send_goal(goal_msg, options);
  return true;
}

}  // namespace smart_car_cpp_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<smart_car_cpp_pkg::GoToPoseNode>();

  if (!node->sendGoal()) {
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  return 0;
}
