#include "smart_car_cpp_pkg/go_to_pose.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

using namespace std::chrono_literals;

namespace smartcar_goal_cpp
{

namespace
{

struct Destination
{
  double x;
  double y;
  double yaw;
};

const std::unordered_map<std::string, Destination> kDestinations = {
  {"toilet", {1.9163875579833984, -1.4372011423110962, 0.0}},
  {"home", {-0.2187747061252594, 0.0520884245634079, 0.0}},
  {"charging_station", {-0.2187747061252594, 0.0520884245634079, 0.0}},
  {"stationery", {2.522963523864746, -2.9560720920562744, 0.0}},
  {"sunscreen", {-0.3035605251789093, -0.7663712501525879, 0.0}},
  {"wet_tissue", {0.7415836453437805, -2.8868584632873535, 0.0}},
};

}  // namespace

GoToPoseNode::GoToPoseNode()
: Node("go_to_pose_cpp"),
  navigation_enabled_(true)
{
  destination_ = declare_parameter<std::string>("destination", "toilet");
  frame_id_ = declare_parameter<std::string>("frame_id", "map");
  action_name_ = declare_parameter<std::string>("action_name", "navigate_to_pose");
  command_topic_ = declare_parameter<std::string>("command_topic", "/gui_command");

  action_client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);
  command_sub_ = create_subscription<std_msgs::msg::String>(
    command_topic_,
    10,
    std::bind(&GoToPoseNode::commandCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "GoToPose ready. Listening to '%s' for CMD_NAV_TO_<destination> commands.",
    command_topic_.c_str());
}

void GoToPoseNode::commandCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "CMD_SET_MODE_PERSON_FOLLOWING") {
    navigation_enabled_ = false;
    action_client_->async_cancel_all_goals();
    RCLCPP_INFO(get_logger(), "Person following mode enabled. Navigation goals paused.");
    return;
  }
  if (msg->data == "CMD_SET_MODE_NAVIGATION") {
    navigation_enabled_ = true;
    RCLCPP_INFO(get_logger(), "Navigation mode enabled. Destination goals accepted.");
    return;
  }

  const auto destination = normalizeDestination(msg->data);
  if (destination.empty()) {
    return;
  }
  if (!navigation_enabled_) {
    RCLCPP_INFO(
      get_logger(),
      "Ignoring destination '%s' while person following mode is active.",
      destination.c_str());
    return;
  }

  sendGoal(destination);
}

std::string GoToPoseNode::normalizeDestination(const std::string & command) const
{
  constexpr char prefix[] = "CMD_NAV_TO_";
  if (command.rfind(prefix, 0) == 0) {
    return command.substr(std::string(prefix).size());
  }
  return "";
}

bool GoToPoseNode::sendGoal(const std::string & destination)
{
  const auto selected = kDestinations.find(destination);
  if (selected == kDestinations.end()) {
    RCLCPP_WARN(
      get_logger(),
      "Unknown destination command '%s'. Goal ignored.",
      destination.c_str());
    return false;
  }

  destination_ = destination;
  target_x_ = selected->second.x;
  target_y_ = selected->second.y;
  target_yaw_ = selected->second.yaw;

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
    "Sending goal: destination=%s frame=%s x=%.3f y=%.3f yaw=%.3f rad",
    destination_.c_str(), frame_id_.c_str(), target_x_, target_y_, target_yaw_);

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.goal_response_callback =
    [this](const GoalHandleNavigateToPose::SharedPtr & goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(get_logger(), "Goal was rejected by Nav2.");
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
    };

  action_client_->async_send_goal(goal_msg, options);
  return true;
}

}  // namespace smartcar_goal_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<smartcar_goal_cpp::GoToPoseNode>();
  rclcpp::spin(node);
  return 0;
}
