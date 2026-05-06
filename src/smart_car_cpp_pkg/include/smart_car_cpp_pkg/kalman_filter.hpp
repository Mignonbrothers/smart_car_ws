#ifndef SMARTCAR_GOAL_CPP__KALMAN_FILTER_HPP_
#define SMARTCAR_GOAL_CPP__KALMAN_FILTER_HPP_

namespace smartcar_goal_cpp
{

struct TrackingState
{
  double x;
  double y;
  double vx;
  double vy;
};

class KalmanFilter
{
public:
  KalmanFilter();

  void reset(double x, double y);
  TrackingState predict(double dt);
  TrackingState update(double measured_x, double measured_y);
  TrackingState state() const;
  bool initialized() const;

private:
  double x_[4];
  double p_[4][4];
  double q_;
  double r_;
  bool initialized_;
};

}  // namespace smartcar_goal_cpp

#endif  // SMARTCAR_GOAL_CPP__KALMAN_FILTER_HPP_
