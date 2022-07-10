// Copyright 2022
#ifndef KALMAN_FILTER_SRC_LINEAR_KALMAN_FILTER_HPP
#define KALMAN_FILTER_SRC_LINEAR_KALMAN_FILTER_HPP

#include <Eigen/Dense>

#include <cassert>

namespace KF_LIB {
template <typename T, size_t state_size, size_t meas_size>
class LinearKalmanFilter {
private:
  size_t state_size_{state_size};
  size_t meas_size_{meas_size};

  Eigen::Matrix<T, state_size, state_size> Q;
  Eigen::Matrix<T, meas_size, meas_size> R;

  Eigen::Matrix<T, state_size, state_size> P;
  Eigen::Matrix<T, meas_size, meas_size> S;
  Eigen::Matrix<T, state_size, meas_size> K;

  Eigen::Matrix<T, state_size, state_size> F;
  Eigen::Matrix<T, meas_size, state_size> H;

  Eigen::Matrix<T, state_size, 1> x;
  Eigen::Matrix<T, meas_size, 1> z;

public:
  LinearKalmanFilter() {
    Q.setZero();
    R.setZero();
    P.setZero();
    S.setZero();
    K.setZero();
    x.setZero();
    z.setZero();
    F.setZero();
    H.setZero();
  }

  size_t StateSize() const { return state_size_; }

  size_t MeasSize() const { return meas_size_; }

  Eigen::Matrix<T, state_size, 1> State() const { return x; }

  void State(const Eigen::Matrix<T, state_size, 1> &state) { x = state; }

  Eigen::Matrix<T, meas_size, 1> Meas() const { return z; }

  void Meas(const Eigen::Matrix<T, meas_size, 1> &meas) { z = meas; }

  void Transition() {
    x = F * x;
    P = F * P * F.transpose() + Q;
  }

  void Observation() {
    Eigen::Matrix<T, meas_size, 1> y = z - (H * x);
    S = H * P * H.transpose() + R;

    K = P * H.transpose() * S.inverse();

    x = x + K * y;
    P = P - K * H * P;
  }
};
} // namespace KF_LIB

#endif // KALMAN_FILTER_SRC_LINEAR_KALMAN_FILTER_HPP