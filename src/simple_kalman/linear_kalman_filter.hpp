// Copyright 2022
#ifndef KALMAN_FILTER_SRC_LINEAR_KALMAN_FILTER_HPP
#define KALMAN_FILTER_SRC_LINEAR_KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <cassert>
#include <iostream>

namespace KF_LIB {

class LinearKalmanFilter {
    size_t state_size_;
    size_t meas_size_;

    Eigen::MatrixXd F;
    Eigen::MatrixXd H;

    Eigen::MatrixXd P;
    Eigen::MatrixXd S;

    Eigen::MatrixXd state;
    Eigen::MatrixXd meas;

    Eigen::MatrixXd K;

 public:
    LinearKalmanFilter(size_t state_size, size_t meas_size);
    void PredictionModel(const Eigen::MatrixXd& m);
    void ObservationModel(const Eigen::MatrixXd& m);

    void StateCov(const Eigen::MatrixXd& m);
    void MeasCov(const Eigen::MatrixXd& m);

    void State(const Eigen::MatrixXd& m);
    void Meas(const Eigen::MatrixXd& m);
};

}  // namespace KF_LIB

#endif  // KALMAN_FILTER_SRC_LINEAR_KALMAN_FILTER_HPP