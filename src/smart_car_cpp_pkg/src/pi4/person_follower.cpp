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
  person_following_enabled_(false),
  last_body_turn_direction_(1),
  latest_pan_angle_rad_(0.0),
  pose_stationary_detected_(false),
  pose_stop_latched_(false),
  obstacle_avoidance_active_(false),
  avoidance_turn_direction_(1),
  wall_follow_side_(-1)
{
  detection_topic_ = declare_parameter<std::string>("detection_topic", "/person_detection");
  pan_angle_topic_ = declare_parameter<std::string>("pan_angle_topic", "/pan_tilt/pan_angle");
  pose_stationary_topic_ = declare_parameter<std::string>("pose_stationary_topic", "/pose_stationary_detected");
  pose_resume_topic_ = declare_parameter<std::string>("pose_resume_topic", "/pose_resume_detected");
  scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
  cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
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
  pose_stationary_hold_s_ = declare_parameter<double>("pose_stationary_hold_s", 3.00);
  pose_stationary_timeout_s_ = declare_parameter<double>("pose_stationary_timeout_s", 0.75);
  obstacle_avoidance_enabled_ = declare_parameter<bool>("obstacle_avoidance_enabled", true);
  obstacle_trigger_distance_m_ = declare_parameter<double>("obstacle_trigger_distance_m", 0.40);
  obstacle_front_half_width_rad_ = declare_parameter<double>("obstacle_front_half_width_rad", M_PI / 4.0);
  front_clear_distance_m_ = declare_parameter<double>("front_clear_distance_m", 1.30);
  front_clear_hold_s_ = declare_parameter<double>("front_clear_hold_s", 0.30);
  min_avoidance_active_s_ = declare_parameter<double>("min_avoidance_active_s", 0.80);
  avoidance_linear_velocity_ = declare_parameter<double>("avoidance_linear_velocity", 0.00);
  avoidance_angular_velocity_ = declare_parameter<double>("avoidance_angular_velocity", 0.20);
  obstacle_alignment_tolerance_rad_ = declare_parameter<double>("obstacle_alignment_tolerance_rad", 0.08);
  wall_follow_linear_velocity_ = declare_parameter<double>("wall_follow_linear_velocity", 0.06);
  wall_follow_angular_velocity_ = declare_parameter<double>("wall_follow_angular_velocity", 0.12);
  wall_follow_target_distance_m_ = declare_parameter<double>("wall_follow_target_distance_m", 0.65);
  wall_follow_kp_ = declare_parameter<double>("wall_follow_kp", 0.80);

  detection_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    detection_topic_, 10,
    std::bind(&PersonFollower::detectionCallback, this, std::placeholders::_1));

  pan_angle_sub_ = create_subscription<std_msgs::msg::Float64>(
    pan_angle_topic_, 10,
    std::bind(&PersonFollower::panAngleCallback, this, std::placeholders::_1));

  pose_stationary_sub_ = create_subscription<std_msgs::msg::Bool>(
    pose_stationary_topic_, 10,
    std::bind(&PersonFollower::poseStationaryCallback, this, std::placeholders::_1));

  pose_resume_sub_ = create_subscription<std_msgs::msg::Bool>(
    pose_resume_topic_, 10,
    std::bind(&PersonFollower::poseResumeCallback, this, std::placeholders::_1));

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PersonFollower::scanCallback, this, std::placeholders::_1));

  command_sub_ = create_subscription<std_msgs::msg::String>(
    command_topic_, 10,
    std::bind(&PersonFollower::commandCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&PersonFollower::controlLoop, this));
  parameter_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&PersonFollower::onParameterUpdate, this, std::placeholders::_1));

  const auto now_time = now();
  last_detection_time_ = now_time;
  pose_stationary_detected_since_ = now_time;
  last_pose_stationary_time_ = now_time;
  obstacle_avoidance_started_time_ = now_time;
  front_clear_since_time_ = now_time;

  RCLCPP_INFO(
    get_logger(),
    "Person follower started in %s mode.",
    person_following_enabled_ ? "person following" : "navigation");
  RCLCPP_INFO(get_logger(), "Detection input: std_msgs/Float32MultiArray [center_x, frame_width, confidence]");
  RCLCPP_INFO(get_logger(), "Pan angle input: std_msgs/Float64 on %s", pan_angle_topic_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Follower uses lidar distance in current pan direction: stop=%.2fm, follow=%.2fm, far=%.2fm",
    stop_distance_m_, follow_distance_m_, far_distance_m_);
  RCLCPP_INFO(
    get_logger(),
    "Pose stationary stop input: std_msgs/Bool on %s, hold=%.2fs",
    pose_stationary_topic_.c_str(), pose_stationary_hold_s_);
  RCLCPP_INFO(
    get_logger(),
    "Pose resume input: std_msgs/Bool on %s", pose_resume_topic_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "Obstacle avoidance: trigger front sector half_width=%.2frad distance<=%.2fm, exit pan angle<=%.2frad.",
    obstacle_front_half_width_rad_, obstacle_trigger_distance_m_, obstacle_alignment_tolerance_rad_);
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

void PersonFollower::panAngleCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  latest_pan_angle_rad_ = msg->data;
  pan_angle_received_ = true;
}

void PersonFollower::poseStationaryCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!person_following_enabled_) {
    return;
  }

  const auto current_time = now();
  last_pose_stationary_time_ = current_time;

  if (msg->data) {
    if (!pose_stationary_detected_) {
      pose_stationary_detected_since_ = current_time;
    }
    pose_stationary_detected_ = true;
  } else {
    pose_stationary_detected_ = false;
    pose_stationary_detected_since_ = current_time;
  }
}

void PersonFollower::poseResumeCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!person_following_enabled_) {
    return;
  }

  if (msg->data && pose_stop_latched_) {
    pose_stop_latched_ = false;
    pose_stationary_detected_ = false;
    pose_stationary_detected_since_ = now();
    last_pose_stationary_time_ = pose_stationary_detected_since_;
    RCLCPP_INFO(get_logger(), "Pose resume detected. Person following resumed.");
  }
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
    pose_stationary_detected_ = false;
    pose_stop_latched_ = false;
    obstacle_avoidance_active_ = false;
    pose_stationary_detected_since_ = now();
    last_pose_stationary_time_ = pose_stationary_detected_since_;
    RCLCPP_INFO(get_logger(), "Person following mode enabled.");
  } else if (msg->data == "CMD_SET_MODE_NAVIGATION") {
    person_following_enabled_ = false;
    person_detected_ = false;
    pose_stationary_detected_ = false;
    pose_stop_latched_ = false;
    obstacle_avoidance_active_ = false;
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

  if (!latest_scan_) {
    publishStop();
    return;
  }

  if (!pan_angle_received_) {
    publishStop();
    return;
  }

  const double pose_stationary_age = (current_time - last_pose_stationary_time_).seconds();
  const bool use_pose_stationary = pose_stationary_detected_ && pose_stationary_age <= pose_stationary_timeout_s_;
  if (!use_pose_stationary && pose_stationary_age > pose_stationary_timeout_s_) {
    pose_stationary_detected_ = false;
  }
  if (use_pose_stationary &&
    (current_time - pose_stationary_detected_since_).seconds() >= pose_stationary_hold_s_)
  {
    if (!pose_stop_latched_) {
      RCLCPP_INFO(
        get_logger(),
        "Pose stationary hold reached. Stop will apply only during normal follow driving.");
    }
    pose_stop_latched_ = true;
  }

  const double front_distance_m = calculateMinDistanceInSector(
    *latest_scan_, 0.0, obstacle_front_half_width_rad_);
  const double left_space_m = calculateAverageDistanceInSector(
    *latest_scan_, M_PI / 2.0, obstacle_front_half_width_rad_);
  const double right_space_m = calculateAverageDistanceInSector(
    *latest_scan_, -M_PI / 2.0, obstacle_front_half_width_rad_);
  if (updateObstacleAvoidance(current_time, front_distance_m, left_space_m, right_space_m)) {
    cmd_vel_pub_->publish(makeAvoidanceCommand(*latest_scan_, front_distance_m));
    return;
  }

  if (pose_stop_latched_) {
    publishStop();
    return;
  }

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

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = calculateLinearVelocity(distance_m);
  cmd_vel.angular.z = calculateAngularVelocity(latest_pan_angle_rad_);
  if (std::abs(cmd_vel.angular.z) > 1e-6) {
    last_body_turn_direction_ = cmd_vel.angular.z > 0.0 ? 1 : -1;
  }

  cmd_vel_pub_->publish(cmd_vel);
}

bool PersonFollower::updateObstacleAvoidance(
  const rclcpp::Time & current_time,
  double front_distance_m,
  double left_space_m,
  double right_space_m)
{
  if (!obstacle_avoidance_enabled_) {
    obstacle_avoidance_active_ = false;
    return false;
  }

  if (!obstacle_avoidance_active_ && !std::isfinite(front_distance_m)) {
    return false;
  }

  if (!obstacle_avoidance_active_ && front_distance_m <= obstacle_trigger_distance_m_) {
    obstacle_avoidance_active_ = true;
    avoidance_turn_direction_ = selectAvoidanceTurnDirection(left_space_m, right_space_m);
    wall_follow_side_ = avoidance_turn_direction_ > 0 ? -1 : 1;
    obstacle_avoidance_started_time_ = current_time;
    front_clear_since_time_ = current_time;
    RCLCPP_INFO(
      get_logger(),
      "Front obstacle detected at %.2fm. left=%.2fm right=%.2fm, turn=%s, follow %s wall.",
      front_distance_m, left_space_m, right_space_m,
      avoidance_turn_direction_ > 0 ? "CCW" : "CW",
      wall_follow_side_ > 0 ? "left" : "right");
    return true;
  }

  if (!obstacle_avoidance_active_) {
    return false;
  }

  const double active_time = (current_time - obstacle_avoidance_started_time_).seconds();
  if (active_time >= min_avoidance_active_s_ &&
    std::abs(latest_pan_angle_rad_) <= obstacle_alignment_tolerance_rad_)
  {
    obstacle_avoidance_active_ = false;
    RCLCPP_INFO(
      get_logger(),
      "Pan angle aligned at %.3frad. Returning from obstacle avoidance to follow driving.",
      latest_pan_angle_rad_);
    return false;
  }

  return true;
}

geometry_msgs::msg::Twist PersonFollower::makeAvoidanceCommand(
  const sensor_msgs::msg::LaserScan & scan,
  double front_distance_m) const
{
  geometry_msgs::msg::Twist cmd_vel;
  const double turn_speed = std::max(0.0, wall_follow_angular_velocity_);

  if (std::isfinite(front_distance_m) && front_distance_m <= obstacle_trigger_distance_m_) {
    cmd_vel.linear.x = avoidance_linear_velocity_;
    cmd_vel.angular.z = avoidance_turn_direction_ * turn_speed;
    return cmd_vel;
  }

  const double wall_angle = wall_follow_side_ > 0 ? M_PI / 2.0 : -M_PI / 2.0;
  const double wall_distance_m = calculateAverageDistanceInSector(
    scan, wall_angle, obstacle_front_half_width_rad_);

  if (!std::isfinite(wall_distance_m)) {
    cmd_vel.linear.x = avoidance_linear_velocity_;
    cmd_vel.angular.z = avoidance_turn_direction_ * turn_speed;
    return cmd_vel;
  }

  const double distance_error_m = wall_distance_m - wall_follow_target_distance_m_;
  const double angular_velocity = wall_follow_side_ * wall_follow_kp_ * distance_error_m;
  cmd_vel.linear.x = wall_follow_linear_velocity_;
  cmd_vel.angular.z = std::clamp(angular_velocity, -turn_speed, turn_speed);
  return cmd_vel;
}

int PersonFollower::selectAvoidanceTurnDirection(double left_space_m, double right_space_m) const
{
  if (std::isfinite(left_space_m) && !std::isfinite(right_space_m)) {
    return 1;
  }
  if (!std::isfinite(left_space_m) && std::isfinite(right_space_m)) {
    return -1;
  }
  if (!std::isfinite(left_space_m) && !std::isfinite(right_space_m)) {
    return last_body_turn_direction_ >= 0 ? 1 : -1;
  }
  return left_space_m >= right_space_m ? 1 : -1;
}

double PersonFollower::calculateMinDistanceInSector(
  const sensor_msgs::msg::LaserScan & scan,
  double center_angle_rad,
  double half_width_rad) const
{
  if (scan.ranges.empty() || scan.angle_increment == 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  auto angular_distance = [](double lhs, double rhs) {
      double diff = std::fmod(lhs - rhs + M_PI, 2.0 * M_PI);
      if (diff < 0.0) {
        diff += 2.0 * M_PI;
      }
      return std::abs(diff - M_PI);
    };

  double min_distance = std::numeric_limits<double>::infinity();
  for (int index = 0; index < static_cast<int>(scan.ranges.size()); ++index) {
    const double sample_angle = scan.angle_min + index * scan.angle_increment;
    if (angular_distance(sample_angle, center_angle_rad) > half_width_rad) {
      continue;
    }

    double range = scan.ranges[index];
    if (std::isinf(range) && range > 0.0) {
      range = scan.range_max;
    }
    if (std::isfinite(range) && range >= scan.range_min && range <= scan.range_max) {
      min_distance = std::min(min_distance, range);
    }
  }

  if (!std::isfinite(min_distance)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return min_distance;
}

double PersonFollower::calculateAverageDistanceInSector(
  const sensor_msgs::msg::LaserScan & scan,
  double center_angle_rad,
  double half_width_rad) const
{
  if (scan.ranges.empty() || scan.angle_increment == 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  auto angular_distance = [](double lhs, double rhs) {
      double diff = std::fmod(lhs - rhs + M_PI, 2.0 * M_PI);
      if (diff < 0.0) {
        diff += 2.0 * M_PI;
      }
      return std::abs(diff - M_PI);
    };

  double sum = 0.0;
  int count = 0;
  for (int index = 0; index < static_cast<int>(scan.ranges.size()); ++index) {
    const double sample_angle = scan.angle_min + index * scan.angle_increment;
    if (angular_distance(sample_angle, center_angle_rad) > half_width_rad) {
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
    } else if (name == "pose_stationary_hold_s") {
      pose_stationary_hold_s_ = value;
    } else if (name == "pose_stationary_timeout_s") {
      pose_stationary_timeout_s_ = value;
    } else if (name == "obstacle_trigger_distance_m") {
      obstacle_trigger_distance_m_ = value;
    } else if (name == "obstacle_front_half_width_rad") {
      obstacle_front_half_width_rad_ = value;
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
    } else if (name == "obstacle_alignment_tolerance_rad") {
      obstacle_alignment_tolerance_rad_ = value;
    } else if (name == "wall_follow_linear_velocity") {
      wall_follow_linear_velocity_ = value;
    } else if (name == "wall_follow_angular_velocity") {
      wall_follow_angular_velocity_ = value;
    } else if (name == "wall_follow_target_distance_m") {
      wall_follow_target_distance_m_ = value;
    } else if (name == "wall_follow_kp") {
      wall_follow_kp_ = value;
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
