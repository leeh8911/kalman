// Copyright 2022
#ifndef SRC_SIMPLE_KALMAN_LINEAR_KALMAN_UTILS_HPP_
#define SRC_SIMPLE_KALMAN_LINEAR_KALMAN_UTILS_HPP_

#include "src/simple_kalman/linear_kalman_filter.hpp"

extern "C" KF_LIB::LinearKalmanFilter* CreateLinearKalmanFilter(
    size_t state_size, size_t meas_size);
extern "C" void DeleteLinearKalmanFilter(KF_LIB::LinearKalmanFilter* p);
extern "C" void SetKalmanParams(KF_LIB::LinearKalmanFilter* p,
                                const Eigen::MatrixXd& F,
                                const Eigen::MatrixXd& H,
                                const Eigen::MatrixXd& Q,
                                const Eigen::MatrixXd& R);
#endif  // SRC_SIMPLE_KALMAN_LINEAR_KALMAN_UTILS_HPP_