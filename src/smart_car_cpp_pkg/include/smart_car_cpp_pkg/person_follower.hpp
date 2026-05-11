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
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

namespace smartcar_goal_cpp
{

class PersonFollower : public rclcpp::Node
{
public:
  PersonFollower();

private:
  void detectionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void ankleDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void panAngleCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void commandCallback(const std_msgs::msg::String::SharedPtr msg);
  void controlLoop();

  double calculateDistanceInDirection(const sensor_msgs::msg::LaserScan & scan, double angle_rad) const;
  double calculateFrontDistance(const sensor_msgs::msg::LaserScan & scan) const;
  void startObstacleAvoidance(const std::string & reason);
  double calculateLinearVelocity(double distance_m) const;
  double calculateAngularVelocity(double pan_angle_rad) const;
  rcl_interfaces::msg::SetParametersResult onParameterUpdate(
    const std::vector<rclcpp::Parameter> & parameters);
  void publishStop();
  void publishObstacleAvoidanceCommand();
  void publishObstacleAvoidanceDebug(
    double front_distance_m,
    double avoidance_elapsed_s,
    double clear_hold_s);
  void updateStopTiltSequence(const rclcpp::Time & current_time);
  void resetStopTiltSequence();
  void publishTiltCommand(int tilt_us, const std::string & reason);
  void publishObstacleAvoidanceTrigger();

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ankle_detected_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr pan_angle_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tilt_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_trigger_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr avoidance_debug_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

  bool person_detected_;
  double person_center_x_;
  double frame_width_;
  double detection_confidence_;
  bool pan_angle_received_;
  bool stop_tilt_lowered_;
  bool stop_tilt_restored_;
  bool obstacle_trigger_published_;
  bool obstacle_avoidance_active_;
  bool ankle_detected_received_;
  bool person_following_enabled_;
  int obstacle_avoidance_turn_direction_;
  int last_body_turn_direction_;
  double latest_pan_angle_rad_;
  rclcpp::Time last_detection_time_;
  rclcpp::Time last_ankle_detection_time_;
  rclcpp::Time stop_tilt_lowered_time_;
  rclcpp::Time obstacle_avoidance_started_time_;
  rclcpp::Time obstacle_clear_start_time_;

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
  int stop_tilt_us_;
  int neutral_tilt_us_;
  double stop_tilt_hold_s_;
  double lowered_detection_timeout_s_;
  double ankle_detection_timeout_s_;
  bool obstacle_avoidance_enabled_;
  double front_clear_distance_m_;
  double front_clear_hold_s_;
  double min_avoidance_active_s_;
  double avoidance_linear_velocity_;
  double avoidance_angular_velocity_;
  std::string detection_topic_;
  std::string ankle_detected_topic_;
  std::string pan_angle_topic_;
  std::string scan_topic_;
  std::string cmd_vel_topic_;
  std::string tilt_topic_;
  std::string obstacle_trigger_topic_;
  std::string avoidance_debug_topic_;
  std::string command_topic_;
};

}  // namespace smartcar_goal_cpp

#endif  // SMARTCAR_GOAL_CPP__PERSON_FOLLOWER_HPP_
