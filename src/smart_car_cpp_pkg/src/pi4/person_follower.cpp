#include "smart_car_cpp_pkg/person_follower.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace smartcar_goal_cpp
{

PersonFollower::PersonFollower()
: Node("person_follower"),
  person_detected_(false),
  person_center_x_(0.0),
  frame_width_(640.0),
  detection_confidence_(0.0),
  pan_angle_received_(false),
  latest_pan_angle_rad_(0.0)
{
  detection_topic_ = declare_parameter<std::string>("detection_topic", "/person_detection");
  pan_angle_topic_ = declare_parameter<std::string>("pan_angle_topic", "/pan_tilt/pan_angle");
  scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

  stop_distance_m_ = declare_parameter<double>("stop_distance_m", 0.80);
  follow_distance_m_ = declare_parameter<double>("follow_distance_m", 1.30);
  far_distance_m_ = declare_parameter<double>("far_distance_m", 1.80);
  normal_linear_velocity_ = declare_parameter<double>("normal_linear_velocity", 0.08);
  fast_linear_velocity_ = declare_parameter<double>("fast_linear_velocity", 0.12);
  max_angular_velocity_ = declare_parameter<double>("max_angular_velocity", 0.18);
  body_turn_kp_ = declare_parameter<double>("body_turn_kp", 0.3);
  aligned_angle_threshold_rad_ = declare_parameter<double>("aligned_angle_threshold_rad", 0.08);
  realign_angle_threshold_rad_ = declare_parameter<double>("realign_angle_threshold_rad", 0.18);
  lost_timeout_s_ = declare_parameter<double>("lost_timeout_s", 2.00);
  min_detection_confidence_ = declare_parameter<double>("min_detection_confidence", 0.50);

  detection_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    detection_topic_, 10,
    std::bind(&PersonFollower::detectionCallback, this, std::placeholders::_1));

  pan_angle_sub_ = create_subscription<std_msgs::msg::Float64>(
    pan_angle_topic_, 10,
    std::bind(&PersonFollower::panAngleCallback, this, std::placeholders::_1));

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PersonFollower::scanCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&PersonFollower::controlLoop, this));
  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&PersonFollower::onParameterUpdate, this, std::placeholders::_1));

  const auto now_time = now();
  last_detection_time_ = now_time;

  RCLCPP_INFO(get_logger(), "Person follower started.");
  RCLCPP_INFO(get_logger(), "Detection input: std_msgs/Float32MultiArray [center_x, frame_width, confidence]");
  RCLCPP_INFO(get_logger(), "Pan angle input: std_msgs/Float64 on %s", pan_angle_topic_.c_str());
  RCLCPP_INFO(get_logger(), "This node publishes /cmd_vel only; pan-tilt servo control is handled separately.");
}

void PersonFollower::detectionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 3) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Detection message must contain [center_x, frame_width, confidence].");
    return;
  }

  person_center_x_ = msg->data[0];
  frame_width_ = std::max<double>(msg->data[1], 1.0);
  detection_confidence_ = msg->data[2];
  person_detected_ = detection_confidence_ >= min_detection_confidence_;

  if (person_detected_) {
    last_detection_time_ = now();
  }
}

void PersonFollower::panAngleCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  latest_pan_angle_rad_ = msg->data;
  pan_angle_received_ = true;
}

void PersonFollower::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  latest_scan_ = msg;
}

void PersonFollower::controlLoop()
{
  if (!latest_scan_) {
    publishStop();
    return;
  }
  if (!pan_angle_received_) {
    publishStop();
    return;
  }

  const auto current_time = now();
  const double lost_time = (current_time - last_detection_time_).seconds();
  const bool use_detection = person_detected_ && lost_time <= lost_timeout_s_;
  if (!use_detection) {
    publishStop();
    return;
  }

  const double target_angle_rad = latest_pan_angle_rad_;
  const double distance_m = calculateDistanceInDirection(*latest_scan_, target_angle_rad);
  if (!std::isfinite(distance_m)) {
    publishStop();
    return;
  }

  if (distance_m < stop_distance_m_) {
    publishStop();
    return;
  }

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = calculateLinearVelocity(distance_m);
  cmd_vel.angular.z = calculateAngularVelocity(latest_pan_angle_rad_);

  cmd_vel_pub_->publish(cmd_vel);
}

double PersonFollower::calculateDistanceInDirection(
  const sensor_msgs::msg::LaserScan & scan,
  double angle_rad) const
{
  if (scan.ranges.empty() || scan.angle_increment == 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double scan_span = scan.angle_max - scan.angle_min;
  if (scan_span <= 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double scan_angle = angle_rad;
  while (scan_angle < scan.angle_min) {
    scan_angle += 2.0 * M_PI;
  }
  while (scan_angle > scan.angle_max) {
    scan_angle -= 2.0 * M_PI;
  }
  if (scan_angle < scan.angle_min || scan_angle > scan.angle_max) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const int center_index = static_cast<int>((scan_angle - scan.angle_min) / scan.angle_increment);
  const int window = 5;
  double sum = 0.0;
  int count = 0;

  for (int offset = -window; offset <= window; ++offset) {
    const int index = center_index + offset;
    if (index < 0 || index >= static_cast<int>(scan.ranges.size())) {
      continue;
    }

    const double range = scan.ranges[index];
    if (std::isfinite(range) && range >= scan.range_min && range <= scan.range_max) {
      sum += range;
      ++count;
    }
  }

  if (count == 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  return sum / static_cast<double>(count);
}

double PersonFollower::calculateLinearVelocity(double distance_m) const
{
  if (distance_m < stop_distance_m_) {
    return 0.0;
  }
  if (distance_m < follow_distance_m_) {
    return normal_linear_velocity_;
  }
  if (distance_m >= far_distance_m_) {
    return fast_linear_velocity_;
  }
  return normal_linear_velocity_;
}

double PersonFollower::calculateAngularVelocity(double pan_angle_rad) const
{
  const double abs_pan_angle = std::abs(pan_angle_rad);
  if (abs_pan_angle <= aligned_angle_threshold_rad_) {
    return 0.0;
  }

  const double turn_velocity = body_turn_kp_ * pan_angle_rad;
  if (abs_pan_angle < realign_angle_threshold_rad_) {
    return std::clamp(turn_velocity, -max_angular_velocity_ * 0.5, max_angular_velocity_ * 0.5);
  }

  return std::clamp(turn_velocity, -max_angular_velocity_, max_angular_velocity_);
}

rcl_interfaces::msg::SetParametersResult PersonFollower::onParameterUpdate(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & parameter : parameters) {
    const auto & name = parameter.get_name();
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      result.successful = false;
      result.reason = name + " must be a double parameter.";
      return result;
    }

    const double value = parameter.as_double();
    if (name == "stop_distance_m") {
      stop_distance_m_ = value;
    } else if (name == "follow_distance_m") {
      follow_distance_m_ = value;
    } else if (name == "far_distance_m") {
      far_distance_m_ = value;
    } else if (name == "normal_linear_velocity") {
      normal_linear_velocity_ = value;
    } else if (name == "fast_linear_velocity") {
      fast_linear_velocity_ = value;
    } else if (name == "max_angular_velocity") {
      max_angular_velocity_ = value;
    } else if (name == "body_turn_kp") {
      body_turn_kp_ = value;
    } else if (name == "aligned_angle_threshold_rad") {
      aligned_angle_threshold_rad_ = value;
    } else if (name == "realign_angle_threshold_rad") {
      realign_angle_threshold_rad_ = value;
    } else if (name == "lost_timeout_s") {
      lost_timeout_s_ = value;
    } else if (name == "min_detection_confidence") {
      min_detection_confidence_ = value;
    }
  }

  if (stop_distance_m_ > follow_distance_m_ || follow_distance_m_ > far_distance_m_) {
    result.successful = false;
    result.reason = "Expected stop_distance_m <= follow_distance_m <= far_distance_m.";
    return result;
  }
  if (aligned_angle_threshold_rad_ > realign_angle_threshold_rad_) {
    result.successful = false;
    result.reason = "Expected aligned_angle_threshold_rad <= realign_angle_threshold_rad.";
    return result;
  }

  return result;
}

void PersonFollower::publishStop()
{
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
}

}  // namespace smartcar_goal_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smartcar_goal_cpp::PersonFollower>());
  rclcpp::shutdown();
  return 0;
}
