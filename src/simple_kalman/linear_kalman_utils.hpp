// Copyright 2022
#ifndef SRC_SIMPLE_KALMAN_LINEAR_KALMAN_UTILS_HPP_
#define SRC_SIMPLE_KALMAN_LINEAR_KALMAN_UTILS_HPP_

#include <vector>

#include "src/simple_kalman/linear_kalman_filter.hpp"

extern "C" KF_LIB::LinearKalmanFilter* CreateLinearKalmanFilter(
    size_t state_size, size_t meas_size);
extern "C" void DeleteLinearKalmanFilter(KF_LIB::LinearKalmanFilter* p);
extern "C" void SetKalmanParams(KF_LIB::LinearKalmanFilter* p,
                                const Eigen::MatrixXd& F,
                                const Eigen::MatrixXd& H,
                                const Eigen::MatrixXd& Q,
                                const Eigen::MatrixXd& R);
extern "C" void SetState(KF_LIB::LinearKalmanFilter* p, const double* arr,
                         int row, int col);
extern "C" Eigen::MatrixXd GetState(KF_LIB::LinearKalmanFilter* p);
extern "C" void SetMeas(KF_LIB::LinearKalmanFilter* p,
                        const Eigen::MatrixXd& meas);
extern "C" void UpdateKalman(KF_LIB::LinearKalmanFilter* p);

extern "C" Eigen::MatrixXd* ConvertArrayToEigen(const double* arr, int row,
                                                int col);
#endif  // SRC_SIMPLE_KALMAN_LINEAR_KALMAN_UTILS_HPP_