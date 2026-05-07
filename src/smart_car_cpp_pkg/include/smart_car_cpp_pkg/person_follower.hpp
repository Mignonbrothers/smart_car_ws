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

namespace smartcar_goal_cpp
{

class PersonFollower : public rclcpp::Node
{
public:
  PersonFollower();

private:
  void detectionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void panAngleCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void controlLoop();

  double calculateDistanceInDirection(const sensor_msgs::msg::LaserScan & scan, double angle_rad) const;
  double calculateLinearVelocity(double distance_m) const;
  double calculateAngularVelocity(double pan_angle_rad) const;
  rcl_interfaces::msg::SetParametersResult onParameterUpdate(
    const std::vector<rclcpp::Parameter> & parameters);
  void publishStop();

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pan_angle_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

  bool person_detected_;
  double person_center_x_;
  double frame_width_;
  double detection_confidence_;
  bool pan_angle_received_;
  double latest_pan_angle_rad_;
  rclcpp::Time last_detection_time_;

  double stop_distance_m_;
  double follow_distance_m_;
  double far_distance_m_;
  double normal_linear_velocity_;
  double fast_linear_velocity_;
  double max_angular_velocity_;
  double body_turn_kp_;
  double aligned_angle_threshold_rad_;
  double realign_angle_threshold_rad_;
  double lost_timeout_s_;
  double min_detection_confidence_;
  std::string detection_topic_;
  std::string pan_angle_topic_;
  std::string scan_topic_;
  std::string cmd_vel_topic_;
};

}  // namespace smartcar_goal_cpp

#endif  // SMARTCAR_GOAL_CPP__PERSON_FOLLOWER_HPP_
