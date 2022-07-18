// Copyright 2022

#include "src/simple_kalman/linear_kalman_filter.hpp"

#include <Eigen/Dense>
#include <cassert>
#include <utility>

namespace KF_LIB {

LinearKalmanFilter::LinearKalmanFilter(size_t state_size, size_t meas_size)
    : state_size_(state_size), meas_size_(meas_size) {
    F = Eigen::MatrixXd(state_size_, state_size_).setZero();
    H = Eigen::MatrixXd(meas_size_, state_size_).setZero();

    P = Eigen::MatrixXd(state_size_, state_size_).setZero();
    S = Eigen::MatrixXd(meas_size_, meas_size_).setZero();
    K = Eigen::MatrixXd(state_size_, meas_size_);

    Q = Eigen::MatrixXd(state_size_, state_size_).setZero();
    R = Eigen::MatrixXd(meas_size_, meas_size_).setZero();

    state_ = Eigen::MatrixXd(state_size_, 1).setZero();
    meas_ = Eigen::MatrixXd(meas_size_, 1).setZero();
}

void LinearKalmanFilter::PredictionModel(const Eigen::MatrixXd& m) { F = m; }
void LinearKalmanFilter::ObservationModel(const Eigen::MatrixXd& m) { H = m; }

Eigen::MatrixXd LinearKalmanFilter::PredictionModel() const { return F; }
Eigen::MatrixXd LinearKalmanFilter::ObservationModel() const { return H; }

void LinearKalmanFilter::PredictionCov(const Eigen::MatrixXd& m) { Q = m; }
void LinearKalmanFilter::ObservationCov(const Eigen::MatrixXd& m) { R = m; }

Eigen::MatrixXd LinearKalmanFilter::PredictionCov() const { return Q; }
Eigen::MatrixXd LinearKalmanFilter::ObservationCov() const { return R; }

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


std::ostream& LinearKalmanFilter::operator<<(std::ostream& os) {
    os << "Kalman Filter\n";
    os << "Transition Matrix\n" << F << "\n";
    os << "Observation Matrix\n" << H << "\n";
    os << "Transition  Covar Matrix\n" << Q << "\n";
    os << "Observation Covar Matrix\n" << R << "\n";
    os << "State Covar Matrix\n" << P << "\n";
    os << "Meas Covar Matrix\n" << S << "\n";
    os << "Kalman Gain\n" << K << "\n";
    os << "State & Meas\n" << state_ << "\n" <<meas_ << "\n";
    return os;
}

std::ostream& operator<<(std::ostream& os, LinearKalmanFilter& kf) {
    return kf.operator<<(os);
}

}  // namespace KF_LIB
