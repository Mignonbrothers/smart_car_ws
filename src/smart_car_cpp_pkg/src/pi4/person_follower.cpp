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
  stop_tilt_lowered_(false),
  stop_tilt_restored_(false),
  obstacle_trigger_published_(false),
  obstacle_avoidance_active_(false),
  ankle_detected_received_(false),
  person_following_enabled_(false),
  obstacle_avoidance_turn_direction_(1),
  last_body_turn_direction_(1),
  latest_pan_angle_rad_(0.0),
  last_ankle_detection_time_(0, 0, RCL_ROS_TIME),
  stop_tilt_lowered_time_(0, 0, RCL_ROS_TIME),
  obstacle_avoidance_started_time_(0, 0, RCL_ROS_TIME),
  obstacle_clear_start_time_(0, 0, RCL_ROS_TIME)
{
  detection_topic_ = declare_parameter<std::string>("detection_topic", "/person_detection");
  ankle_detected_topic_ = declare_parameter<std::string>("ankle_detected_topic", "/ankle_detected");
  pan_angle_topic_ = declare_parameter<std::string>("pan_angle_topic", "/pan_tilt/pan_angle");
  scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
  tilt_topic_ = declare_parameter<std::string>("tilt_topic", "/servo_tilt_cmd");
  obstacle_trigger_topic_ = declare_parameter<std::string>(
    "obstacle_trigger_topic", "/obstacle_avoidance_trigger");
  avoidance_debug_topic_ = declare_parameter<std::string>(
    "avoidance_debug_topic", "/person_follower/avoidance_debug");
  command_topic_ = declare_parameter<std::string>("command_topic", "/gui_command");
  person_following_enabled_ = declare_parameter<bool>("start_enabled", false);

  stop_distance_m_ = declare_parameter<double>("stop_distance_m", 1.00);
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
  stop_tilt_us_ = declare_parameter<int>("stop_tilt_us", 1400);
  neutral_tilt_us_ = declare_parameter<int>("neutral_tilt_us", 1800);
  stop_tilt_hold_s_ = declare_parameter<double>("stop_tilt_hold_s", 3.00);
  lowered_detection_timeout_s_ = declare_parameter<double>("lowered_detection_timeout_s", 0.50);
  ankle_detection_timeout_s_ = declare_parameter<double>("ankle_detection_timeout_s", 0.50);
  obstacle_avoidance_enabled_ = declare_parameter<bool>("obstacle_avoidance_enabled", true);
  front_clear_distance_m_ = declare_parameter<double>("front_clear_distance_m", 1.00);
  front_clear_hold_s_ = declare_parameter<double>("front_clear_hold_s", 0.30);
  min_avoidance_active_s_ = declare_parameter<double>("min_avoidance_active_s", 0.80);
  avoidance_linear_velocity_ = declare_parameter<double>("avoidance_linear_velocity", 0.0);
  avoidance_angular_velocity_ = declare_parameter<double>("avoidance_angular_velocity", 0.20);

  detection_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    detection_topic_, 10,
    std::bind(&PersonFollower::detectionCallback, this, std::placeholders::_1));

  ankle_detected_sub_ = create_subscription<std_msgs::msg::Bool>(
    ankle_detected_topic_, 10,
    std::bind(&PersonFollower::ankleDetectedCallback, this, std::placeholders::_1));

  pan_angle_sub_ = create_subscription<std_msgs::msg::Float64>(
    pan_angle_topic_, 10,
    std::bind(&PersonFollower::panAngleCallback, this, std::placeholders::_1));

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PersonFollower::scanCallback, this, std::placeholders::_1));

  command_sub_ = create_subscription<std_msgs::msg::String>(
    command_topic_, 10,
    std::bind(&PersonFollower::commandCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  tilt_pub_ = create_publisher<std_msgs::msg::Int32>(tilt_topic_, 10);
  obstacle_trigger_pub_ = create_publisher<std_msgs::msg::Bool>(obstacle_trigger_topic_, 10);
  avoidance_debug_pub_ =
    create_publisher<std_msgs::msg::Float32MultiArray>(avoidance_debug_topic_, 10);

  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&PersonFollower::controlLoop, this));
  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&PersonFollower::onParameterUpdate, this, std::placeholders::_1));

  const auto now_time = now();
  last_detection_time_ = now_time;
  last_ankle_detection_time_ = now_time;

  RCLCPP_INFO(
    get_logger(),
    "Person follower started in %s mode.",
    person_following_enabled_ ? "person following" : "navigation");
  RCLCPP_INFO(get_logger(), "Detection input: std_msgs/Float32MultiArray [center_x, frame_width, confidence]");
  RCLCPP_INFO(get_logger(), "Lowered camera ankle input: std_msgs/Bool on %s", ankle_detected_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Pan angle input: std_msgs/Float64 on %s", pan_angle_topic_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Stop tilt output: std_msgs/Int32 on %s, lower=%dus, neutral=%dus, hold=%.2fs",
    tilt_topic_.c_str(), stop_tilt_us_, neutral_tilt_us_, stop_tilt_hold_s_);
  RCLCPP_INFO(
    get_logger(),
    "Obstacle avoidance trigger: std_msgs/Bool on %s",
    obstacle_trigger_topic_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Obstacle avoidance: enabled=%s, front_clear=%.2fm",
    obstacle_avoidance_enabled_ ? "true" : "false",
    front_clear_distance_m_);
}

void PersonFollower::detectionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (!person_following_enabled_) {
    return;
  }

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

void PersonFollower::ankleDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!person_following_enabled_) {
    return;
  }

  ankle_detected_received_ = true;
  if (msg->data) {
    last_ankle_detection_time_ = now();
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

void PersonFollower::commandCallback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "CMD_SET_MODE_PERSON_FOLLOWING") {
    person_following_enabled_ = true;
    person_detected_ = false;
    obstacle_avoidance_active_ = false;
    resetStopTiltSequence();
    RCLCPP_INFO(get_logger(), "Person following mode enabled.");
  } else if (msg->data == "CMD_SET_MODE_NAVIGATION") {
    person_following_enabled_ = false;
    person_detected_ = false;
    obstacle_avoidance_active_ = false;
    resetStopTiltSequence();
    publishStop();
    RCLCPP_INFO(get_logger(), "Navigation mode enabled. Person follower paused.");
  }
}

void PersonFollower::controlLoop()
{
  if (!person_following_enabled_) {
    return;
  }

  const auto current_time = now();
  if (stop_tilt_lowered_ && !stop_tilt_restored_) {
    updateStopTiltSequence(current_time);
    publishStop();
    return;
  }

  if (!latest_scan_) {
    publishStop();
    return;
  }

  const double front_distance_m = calculateFrontDistance(*latest_scan_);
  if (obstacle_avoidance_active_) {
    if (!obstacle_avoidance_enabled_) {
      obstacle_avoidance_active_ = false;
      publishStop();
      return;
    }

    const double avoidance_elapsed_s =
      (current_time - obstacle_avoidance_started_time_).seconds();
    double clear_hold_s = 0.0;
    if (std::isfinite(front_distance_m)) {
      if (
        avoidance_elapsed_s >= min_avoidance_active_s_ &&
        front_distance_m >= front_clear_distance_m_)
      {
        if (obstacle_clear_start_time_.nanoseconds() == 0) {
          obstacle_clear_start_time_ = current_time;
        }
        clear_hold_s = (current_time - obstacle_clear_start_time_).seconds();
        if (clear_hold_s >= front_clear_hold_s_) {
          obstacle_avoidance_active_ = false;
          obstacle_clear_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
          publishStop();
          RCLCPP_INFO(
            get_logger(),
            "Front path clear: %.2fm. Returning to person following.",
            front_distance_m);
        }
      } else {
        obstacle_clear_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Front lidar distance is invalid during obstacle avoidance. Continuing avoidance command.");
    }
    publishObstacleAvoidanceDebug(front_distance_m, avoidance_elapsed_s, clear_hold_s);

    if (obstacle_avoidance_active_) {
      publishObstacleAvoidanceCommand();
    }
    return;
  }

  if (!pan_angle_received_) {
    publishStop();
    return;
  }

  const double lost_time = (current_time - last_detection_time_).seconds();
  const bool use_detection = person_detected_ && lost_time <= lost_timeout_s_;
  if (!use_detection) {
    if (stop_tilt_lowered_ && !stop_tilt_restored_) {
      updateStopTiltSequence(current_time);
    }
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
    updateStopTiltSequence(current_time);
    return;
  }

  resetStopTiltSequence();

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = calculateLinearVelocity(distance_m);
  cmd_vel.angular.z = calculateAngularVelocity(latest_pan_angle_rad_);
  if (std::abs(cmd_vel.angular.z) > 1e-6) {
    last_body_turn_direction_ = cmd_vel.angular.z > 0.0 ? 1 : -1;
  }

  cmd_vel_pub_->publish(cmd_vel);
}

double PersonFollower::calculateFrontDistance(const sensor_msgs::msg::LaserScan & scan) const
{
  return calculateDistanceInDirection(scan, 0.0);
}

void PersonFollower::startObstacleAvoidance(const std::string & reason)
{
  if (!obstacle_avoidance_enabled_ || obstacle_avoidance_active_) {
    return;
  }
  if (!latest_scan_) {
    return;
  }

  obstacle_avoidance_active_ = true;
  obstacle_avoidance_turn_direction_ = last_body_turn_direction_;
  obstacle_avoidance_started_time_ = now();
  obstacle_clear_start_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  RCLCPP_WARN(
    get_logger(),
    "%s. Starting lidar-based avoidance. turn_direction=%d",
    reason.c_str(), obstacle_avoidance_turn_direction_);
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

  auto angular_distance = [](double lhs, double rhs) {
      double diff = std::fmod(lhs - rhs + M_PI, 2.0 * M_PI);
      if (diff < 0.0) {
        diff += 2.0 * M_PI;
      }
      return std::abs(diff - M_PI);
    };

  int center_index = -1;
  double best_angle_error = std::numeric_limits<double>::infinity();
  for (int index = 0; index < static_cast<int>(scan.ranges.size()); ++index) {
    const double sample_angle = scan.angle_min + index * scan.angle_increment;
    const double angle_error = angular_distance(sample_angle, angle_rad);
    if (angle_error < best_angle_error) {
      best_angle_error = angle_error;
      center_index = index;
    }
  }

  if (center_index < 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const int window = 5;
  double sum = 0.0;
  int count = 0;

  for (int offset = -window; offset <= window; ++offset) {
    const int index = center_index + offset;
    if (index < 0 || index >= static_cast<int>(scan.ranges.size())) {
      continue;
    }

    double range = scan.ranges[index];
    if (std::isinf(range) && range > 0.0) {
      range = scan.range_max;
    }
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
    if (name == "stop_tilt_us") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = name + " must be an integer parameter.";
        return result;
      }

      const int value = parameter.as_int();
      if (value < 500 || value > 2500) {
        result.successful = false;
        result.reason = name + " must be between 500 and 2500.";
        return result;
      }
      stop_tilt_us_ = value;
      continue;
    }
    if (name == "neutral_tilt_us") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
        result.successful = false;
        result.reason = name + " must be an integer parameter.";
        return result;
      }

      const int value = parameter.as_int();
      if (value < 500 || value > 2500) {
        result.successful = false;
        result.reason = name + " must be between 500 and 2500.";
        return result;
      }
      neutral_tilt_us_ = value;
      continue;
    }
    if (name == "obstacle_avoidance_enabled") {
      if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
        result.successful = false;
        result.reason = name + " must be a bool parameter.";
        return result;
      }
      obstacle_avoidance_enabled_ = parameter.as_bool();
      continue;
    }

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
    } else if (name == "stop_tilt_hold_s") {
      stop_tilt_hold_s_ = value;
    } else if (name == "lowered_detection_timeout_s") {
      lowered_detection_timeout_s_ = value;
    } else if (name == "ankle_detection_timeout_s") {
      ankle_detection_timeout_s_ = value;
    } else if (name == "front_clear_distance_m") {
      front_clear_distance_m_ = value;
    } else if (name == "front_clear_hold_s") {
      front_clear_hold_s_ = value;
    } else if (name == "min_avoidance_active_s") {
      min_avoidance_active_s_ = value;
    } else if (name == "avoidance_linear_velocity") {
      avoidance_linear_velocity_ = value;
    } else if (name == "avoidance_angular_velocity") {
      avoidance_angular_velocity_ = value;
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
  if (stop_tilt_hold_s_ < 0.0) {
    result.successful = false;
    result.reason = "stop_tilt_hold_s must be >= 0.";
    return result;
  }
  if (lowered_detection_timeout_s_ < 0.0) {
    result.successful = false;
    result.reason = "lowered_detection_timeout_s must be >= 0.";
    return result;
  }
  if (ankle_detection_timeout_s_ < 0.0) {
    result.successful = false;
    result.reason = "ankle_detection_timeout_s must be >= 0.";
    return result;
  }
  if (front_clear_distance_m_ <= 0.0) {
    result.successful = false;
    result.reason = "front_clear_distance_m must be > 0.";
    return result;
  }
  if (front_clear_hold_s_ < 0.0) {
    result.successful = false;
    result.reason = "front_clear_hold_s must be >= 0.";
    return result;
  }
  if (min_avoidance_active_s_ < 0.0) {
    result.successful = false;
    result.reason = "min_avoidance_active_s must be >= 0.";
    return result;
  }
  if (avoidance_linear_velocity_ < 0.0) {
    result.successful = false;
    result.reason = "avoidance_linear_velocity must be >= 0.";
    return result;
  }
  if (avoidance_angular_velocity_ < 0.0) {
    result.successful = false;
    result.reason = "avoidance_angular_velocity must be >= 0.";
    return result;
  }

  return result;
}

void PersonFollower::publishStop()
{
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
}

void PersonFollower::publishObstacleAvoidanceCommand()
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = avoidance_linear_velocity_;
  cmd_vel.angular.z = obstacle_avoidance_turn_direction_ * avoidance_angular_velocity_;
  cmd_vel_pub_->publish(cmd_vel);
}

void PersonFollower::publishObstacleAvoidanceDebug(
  double front_distance_m,
  double avoidance_elapsed_s,
  double clear_hold_s)
{
  std_msgs::msg::Float32MultiArray msg;
  msg.data = {
    static_cast<float>(front_distance_m),
    static_cast<float>(front_clear_distance_m_),
    static_cast<float>(avoidance_elapsed_s),
    static_cast<float>(min_avoidance_active_s_),
    static_cast<float>(clear_hold_s),
    static_cast<float>(front_clear_hold_s_),
    obstacle_avoidance_active_ ? 1.0F : 0.0F,
  };
  avoidance_debug_pub_->publish(msg);
}

void PersonFollower::updateStopTiltSequence(const rclcpp::Time & current_time)
{
  if (!stop_tilt_lowered_) {
    publishTiltCommand(stop_tilt_us_, "Stop distance reached");
    stop_tilt_lowered_ = true;
    stop_tilt_restored_ = false;
    obstacle_trigger_published_ = false;
    ankle_detected_received_ = false;
    stop_tilt_lowered_time_ = current_time;
    return;
  }

  if (stop_tilt_restored_) {
    return;
  }

  const double elapsed_s = (current_time - stop_tilt_lowered_time_).seconds();
  if (elapsed_s >= stop_tilt_hold_s_) {
    const bool ankle_recently_detected =
      ankle_detected_received_ &&
      (current_time - last_ankle_detection_time_).seconds() <= ankle_detection_timeout_s_;
    const bool should_start_obstacle_avoidance = !ankle_recently_detected;
    if (should_start_obstacle_avoidance) {
      publishObstacleAvoidanceTrigger();
    }
    publishTiltCommand(neutral_tilt_us_, "Stop tilt hold complete");
    stop_tilt_restored_ = true;
    if (should_start_obstacle_avoidance) {
      startObstacleAvoidance("No ankle keypoint detection after camera lowered");
    }
  }
}

void PersonFollower::resetStopTiltSequence()
{
  stop_tilt_lowered_ = false;
  stop_tilt_restored_ = false;
  obstacle_trigger_published_ = false;
}

void PersonFollower::publishTiltCommand(int tilt_us, const std::string & reason)
{
  std_msgs::msg::Int32 msg;
  msg.data = std::clamp(tilt_us, 500, 2500);
  tilt_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "%s. Published tilt command: %dus", reason.c_str(), msg.data);
}

void PersonFollower::publishObstacleAvoidanceTrigger()
{
  if (obstacle_trigger_published_) {
    return;
  }

  std_msgs::msg::Bool msg;
  msg.data = true;
  obstacle_trigger_pub_->publish(msg);
  obstacle_trigger_published_ = true;
  RCLCPP_WARN(
    get_logger(),
    "No ankle keypoint detection while camera was lowered. Published obstacle avoidance trigger.");
}

}  // namespace smartcar_goal_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<smartcar_goal_cpp::PersonFollower>());
  rclcpp::shutdown();
  return 0;
}
