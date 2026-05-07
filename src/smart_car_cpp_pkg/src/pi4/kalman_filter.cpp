#include "smart_car_cpp_pkg/kalman_filter.hpp"

#include <algorithm>

namespace smartcar_goal_cpp
{

KalmanFilter::KalmanFilter()
: x_{0.0, 0.0, 0.0, 0.0},
  p_{},
  q_(0.05),
  r_(0.20),
  initialized_(false)
{
  for (int i = 0; i < 4; ++i) {
    p_[i][i] = 1.0;
  }
}

void KalmanFilter::reset(double x, double y)
{
  x_[0] = x;
  x_[1] = y;
  x_[2] = 0.0;
  x_[3] = 0.0;

  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      p_[row][col] = row == col ? 1.0 : 0.0;
    }
  }

  initialized_ = true;
}

TrackingState KalmanFilter::predict(double dt)
{
  if (!initialized_) {
    return state();
  }

  dt = std::clamp(dt, 0.01, 1.0);

  x_[0] += x_[2] * dt;
  x_[1] += x_[3] * dt;

  double f[4][4] = {
    {1.0, 0.0, dt, 0.0},
    {0.0, 1.0, 0.0, dt},
    {0.0, 0.0, 1.0, 0.0},
    {0.0, 0.0, 0.0, 1.0},
  };

  double fp[4][4] = {};
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      for (int k = 0; k < 4; ++k) {
        fp[row][col] += f[row][k] * p_[k][col];
      }
    }
  }

  double next_p[4][4] = {};
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      for (int k = 0; k < 4; ++k) {
        next_p[row][col] += fp[row][k] * f[col][k];
      }
      if (row == col) {
        next_p[row][col] += q_;
      }
    }
  }

  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      p_[row][col] = next_p[row][col];
    }
  }

  return state();
}

TrackingState KalmanFilter::update(double measured_x, double measured_y)
{
  if (!initialized_) {
    reset(measured_x, measured_y);
    return state();
  }

  const double z[2] = {measured_x, measured_y};
  const double innovation[2] = {
    z[0] - x_[0],
    z[1] - x_[1],
  };

  double s[2][2] = {
    {p_[0][0] + r_, p_[0][1]},
    {p_[1][0], p_[1][1] + r_},
  };
  const double det = s[0][0] * s[1][1] - s[0][1] * s[1][0];
  if (det == 0.0) {
    return state();
  }

  double s_inv[2][2] = {
    {s[1][1] / det, -s[0][1] / det},
    {-s[1][0] / det, s[0][0] / det},
  };

  double k[4][2] = {};
  for (int row = 0; row < 4; ++row) {
    k[row][0] = p_[row][0] * s_inv[0][0] + p_[row][1] * s_inv[1][0];
    k[row][1] = p_[row][0] * s_inv[0][1] + p_[row][1] * s_inv[1][1];
  }

  for (int row = 0; row < 4; ++row) {
    x_[row] += k[row][0] * innovation[0] + k[row][1] * innovation[1];
  }

  double next_p[4][4] = {};
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      next_p[row][col] = p_[row][col] - k[row][0] * p_[0][col] - k[row][1] * p_[1][col];
    }
  }

  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      p_[row][col] = next_p[row][col];
    }
  }

  return state();
}

TrackingState KalmanFilter::state() const
{
  return TrackingState{x_[0], x_[1], x_[2], x_[3]};
}

bool KalmanFilter::initialized() const
{
  return initialized_;
}

}  // namespace smartcar_goal_cpp
