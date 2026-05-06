#ifndef SMARTCAR_GOAL_CPP__PERSON_FOLLOWER_HPP_
#define SMARTCAR_GOAL_CPP__PERSON_FOLLOWER_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"

#include "hpp/kalman_filter.hpp"

namespace smartcar_goal_cpp
{

class PersonFollower : public rclcpp::Node
{
public:
  PersonFollower();

private:
  void detectionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void servoAngleCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void controlLoop();

  double calculateDistanceInDirection(const sensor_msgs::msg::LaserScan & scan, double angle_rad) const;
  double calculateLinearVelocity(double distance_m) const;
  double calculateTargetAngle(double normalized_error) const;
  double calculatePanTiltTargetAngle(double normalized_error) const;
  double calculateAngularVelocity(double normalized_error) const;
  bool shouldRotateBody(bool use_detection) const;
  rcl_interfaces::msg::SetParametersResult onParameterUpdate(
    const std::vector<rclcpp::Parameter> & parameters);
  void publishStop();

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_angle_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pan_tilt_target_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  KalmanFilter kalman_filter_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

  bool person_detected_;
  double person_center_x_;
  double frame_width_;
  double detection_confidence_;
  double servo_current_angle_;
  rclcpp::Time last_detection_time_;
  rclcpp::Time last_prediction_time_;

  double stop_distance_m_;
  double follow_distance_m_;
  double far_distance_m_;
  double normal_linear_velocity_;
  double fast_linear_velocity_;
  double max_angular_velocity_;
  double camera_fov_rad_;
  double max_pan_tilt_angle_rad_;
  double lost_pan_tilt_angle_rad_;
  double servo_limit_threshold_;
  double lost_timeout_s_;
  double min_detection_confidence_;
  std::string detection_topic_;
  std::string scan_topic_;
  std::string cmd_vel_topic_;
  std::string servo_angle_topic_;
  std::string pan_tilt_target_topic_;
};

}  // namespace smartcar_goal_cpp

#endif  // SMARTCAR_GOAL_CPP__PERSON_FOLLOWER_HPP_
