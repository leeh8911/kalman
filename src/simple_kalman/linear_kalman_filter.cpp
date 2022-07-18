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

    Q = Eigen::MatrixXd(state_size_, state_size_).setZero();
    R = Eigen::MatrixXd(meas_size, meas_size).setZero();

    state_ = Eigen::MatrixXd(state_size_, 1).setZero();
    meas_ = Eigen::MatrixXd(meas_size, 1).setZero();
}

void LinearKalmanFilter::PredictionModel(const Eigen::MatrixXd& m) { F = m; }
void LinearKalmanFilter::ObservationModel(const Eigen::MatrixXd& m) { H = m; }

Eigen::MatrixXd LinearKalmanFilter::PredictionModel() const { return F; }
Eigen::MatrixXd LinearKalmanFilter::ObservationModel() const { return H; }

void LinearKalmanFilter::StateCov(const Eigen::MatrixXd& m) { P = m; }
void LinearKalmanFilter::MeasCov(const Eigen::MatrixXd& m) { S = m; }

Eigen::MatrixXd LinearKalmanFilter::StateCov() const { return P; };
Eigen::MatrixXd LinearKalmanFilter::MeasCov() const { return S; };

void LinearKalmanFilter::State(const Eigen::MatrixXd& m) { state_ = m; }
void LinearKalmanFilter::Meas(const Eigen::MatrixXd& m) { meas_ = m; }

Eigen::MatrixXd LinearKalmanFilter::State() const { return state_; }
Eigen::MatrixXd LinearKalmanFilter::Meas() const { return meas_; }

void LinearKalmanFilter::Estimate() {
    state_ = F * state_;
    P = F * P * F.transpose() + Q;

    S = H * P * H.transpose() + R;
    K = P * H.transpose() * S.inverse();

    state_ = state_ + K * (meas_ - H * state_);
    P = P - K * H * P;
}
void LinearKalmanFilter::Estimate(const Eigen::MatrixXd& m) {
    Meas(m);

    Estimate();
}

}  // namespace KF_LIB
