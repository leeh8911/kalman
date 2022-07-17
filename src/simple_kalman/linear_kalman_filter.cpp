// Copyright 2022

#include "src/simple_kalman/linear_kalman_filter.hpp"

#include <Eigen/Dense>
#include <cassert>
#include <utility>

namespace KF_LIB {

LinearKalmanFilter::LinearKalmanFilter(size_t state_size, size_t meas_size)
    : state_size_(state_size), meas_size_(meas_size) {
    F = Eigen::MatrixXd(state_size_, state_size_).setZero();
    H = Eigen::MatrixXd(meas_size, state_size_).setZero();

    P = Eigen::MatrixXd(state_size_, state_size_).setZero();
    S = Eigen::MatrixXd(meas_size, meas_size).setZero();

    state = Eigen::MatrixXd(state_size_, 1).setZero();
    meas = Eigen::MatrixXd(meas_size, 1).setZero();
}

void LinearKalmanFilter::PredictionModel(const Eigen::MatrixXd& m) { F = m; }
void LinearKalmanFilter::ObservationModel(const Eigen::MatrixXd& m) { H = m; }

void LinearKalmanFilter::StateCov(const Eigen::MatrixXd& m) { P = m; }
void LinearKalmanFilter::MeasCov(const Eigen::MatrixXd& m) { S = m; }

void LinearKalmanFilter::State(const Eigen::MatrixXd& m) { state = m; }
void LinearKalmanFilter::Meas(const Eigen::MatrixXd& m) { meas = m; }
}  // namespace KF_LIB
