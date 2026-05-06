#include "hpp/person_follower.hpp"

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
  servo_current_angle_(0.0)
{
  detection_topic_ = declare_parameter<std::string>("detection_topic", "/person_detection");
  scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  servo_angle_topic_ = declare_parameter<std::string>("servo_angle_topic", "/pan_tilt/current_angle");
  pan_tilt_target_topic_ = declare_parameter<std::string>(
    "pan_tilt_target_topic", "/pan_tilt/target_angle");

  stop_distance_m_ = declare_parameter<double>("stop_distance_m", 1.50);
  follow_distance_m_ = declare_parameter<double>("follow_distance_m", 1.70);
  far_distance_m_ = declare_parameter<double>("far_distance_m", 1.70);
  normal_linear_velocity_ = declare_parameter<double>("normal_linear_velocity", 0.10);
  fast_linear_velocity_ = declare_parameter<double>("fast_linear_velocity", 0.16);
  max_angular_velocity_ = declare_parameter<double>("max_angular_velocity", 0.45);
  camera_fov_rad_ = declare_parameter<double>("camera_fov_rad", 1.0472);
  max_pan_tilt_angle_rad_ = declare_parameter<double>("max_pan_tilt_angle_rad", 1.5708);
  lost_pan_tilt_angle_rad_ = declare_parameter<double>("lost_pan_tilt_angle_rad", 0.0);
  servo_limit_threshold_ = declare_parameter<double>("servo_limit_threshold", 1.35);
  lost_timeout_s_ = declare_parameter<double>("lost_timeout_s", 0.50);
  min_detection_confidence_ = declare_parameter<double>("min_detection_confidence", 0.50);

  detection_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    detection_topic_, 10,
    std::bind(&PersonFollower::detectionCallback, this, std::placeholders::_1));

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PersonFollower::scanCallback, this, std::placeholders::_1));

  servo_angle_sub_ = create_subscription<std_msgs::msg::Float64>(
    servo_angle_topic_, 10,
    std::bind(&PersonFollower::servoAngleCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  pan_tilt_target_pub_ = create_publisher<std_msgs::msg::Float64>(pan_tilt_target_topic_, 10);

  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&PersonFollower::controlLoop, this));
  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&PersonFollower::onParameterUpdate, this, std::placeholders::_1));

  const auto now_time = now();
  last_detection_time_ = now_time;
  last_prediction_time_ = now_time;

  RCLCPP_INFO(get_logger(), "Person follower started.");
  RCLCPP_INFO(get_logger(), "Detection input: std_msgs/Float32MultiArray [center_x, frame_width, confidence]");
  RCLCPP_INFO(get_logger(), "Pan-tilt PWM control is handled by OpenCR.");
  RCLCPP_INFO(get_logger(), "Pan-tilt target output: std_msgs/Float64 angle in radians.");
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
    const double normalized_error = ((person_center_x_ / frame_width_) - 0.5) * 2.0;

    if (!kalman_filter_.initialized()) {
      kalman_filter_.reset(normalized_error, 0.0);
    } else {
      kalman_filter_.update(normalized_error, 0.0);
    }

    last_detection_time_ = now();
  }
}

void PersonFollower::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  latest_scan_ = msg;
}

void PersonFollower::servoAngleCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  servo_current_angle_ = msg->data;
}

void PersonFollower::controlLoop()
{
  if (!latest_scan_) {
    publishStop();
    return;
  }

  const auto current_time = now();
  const double lost_time = (current_time - last_detection_time_).seconds();
  const bool use_detection = person_detected_ && lost_time <= lost_timeout_s_;

  double normalized_error = 0.0;
  if (use_detection) {
    normalized_error = ((person_center_x_ / frame_width_) - 0.5) * 2.0;
  } else if (kalman_filter_.initialized()) {
    const double dt = (current_time - last_prediction_time_).seconds();
    normalized_error = std::clamp(kalman_filter_.predict(dt).x, -1.0, 1.0);
  } else {
    publishStop();
    return;
  }
  last_prediction_time_ = current_time;

  const double target_angle_rad = calculateTargetAngle(normalized_error);
  const double distance_m = calculateDistanceInDirection(*latest_scan_, target_angle_rad);
  if (!std::isfinite(distance_m)) {
    publishStop();
    return;
  }

  if (use_detection) {
    std_msgs::msg::Float64 pan_tilt_msg;
    pan_tilt_msg.data = calculatePanTiltTargetAngle(normalized_error);
    pan_tilt_target_pub_->publish(pan_tilt_msg);
  } else {
    std_msgs::msg::Float64 pan_tilt_msg;
    pan_tilt_msg.data = lost_pan_tilt_angle_rad_;
    pan_tilt_target_pub_->publish(pan_tilt_msg);
  }

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = use_detection ? calculateLinearVelocity(distance_m) : 0.0;

  const bool body_rotation_allowed = shouldRotateBody(use_detection);
  const double body_angular_velocity =
    body_rotation_allowed ? calculateAngularVelocity(normalized_error) : 0.0;
  cmd_vel.angular.z = body_angular_velocity;

  cmd_vel_pub_->publish(cmd_vel);
}

double PersonFollower::calculateDistanceInDirection(
  const sensor_msgs::msg::LaserScan & scan,
  double angle_rad) const
{
  if (scan.ranges.empty() || scan.angle_increment == 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const int center_index = static_cast<int>((angle_rad - scan.angle_min) / scan.angle_increment);
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
  if (distance_m <= follow_distance_m_) {
    return normal_linear_velocity_;
  }
  if (distance_m > far_distance_m_) {
    return fast_linear_velocity_;
  }
  return normal_linear_velocity_;
}

double PersonFollower::calculateTargetAngle(double normalized_error) const
{
  return std::clamp(normalized_error, -1.0, 1.0) * (camera_fov_rad_ * 0.5);
}

double PersonFollower::calculatePanTiltTargetAngle(double normalized_error) const
{
  const double target_angle = calculateTargetAngle(normalized_error);
  return std::clamp(target_angle, -max_pan_tilt_angle_rad_, max_pan_tilt_angle_rad_);
}

double PersonFollower::calculateAngularVelocity(double normalized_error) const
{
  return std::clamp(-normalized_error * max_angular_velocity_, -max_angular_velocity_, max_angular_velocity_);
}

bool PersonFollower::shouldRotateBody(bool use_detection) const
{
  if (!use_detection) {
    return true;
  }

  return std::abs(servo_current_angle_) >= servo_limit_threshold_;
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
    } else if (name == "camera_fov_rad") {
      camera_fov_rad_ = value;
    } else if (name == "max_pan_tilt_angle_rad") {
      max_pan_tilt_angle_rad_ = value;
    } else if (name == "lost_pan_tilt_angle_rad") {
      lost_pan_tilt_angle_rad_ = value;
    } else if (name == "servo_limit_threshold") {
      servo_limit_threshold_ = value;
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
