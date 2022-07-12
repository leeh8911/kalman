// Copyright 2022
#ifndef KALMAN_FILTER_SRC_EXTENDED_KALMAN_FILTER_HPP
#define KALMAN_FILTER_SRC_EXTENDED_KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <cassert>
#include <functional>

namespace KF_LIB {
template <typename T, size_t input_size, size_t output_size>
Eigen::Matrix<T, output_size, input_size> CalcNumericJacobian(
    std::function<
        Eigen::Matrix<T, output_size, 1>(Eigen::Matrix<T, input_size, 1>)> &f) {
    Eigen::Matrix<T, output_size, input_size> des;

    return des;
}
template <typename T, size_t state_size, size_t meas_size>
class ExtendedKalmanFilter {
 private:
    size_t state_size_{state_size};
    size_t meas_size_{meas_size};

    std::function<Eigen::Matrix<T, state_size, 1>(
        Eigen::Matrix<T, state_size, 1>)>
        transition;
    std::function<Eigen::Matrix<T, meas_size, 1>(
        Eigen::Matrix<T, state_size, 1>)>
        observation;

    Eigen::Matrix<T, state_size, state_size> Q_;
    Eigen::Matrix<T, meas_size, meas_size> R_;

    Eigen::Matrix<T, state_size, state_size> P_;
    Eigen::Matrix<T, meas_size, meas_size> S_;
    Eigen::Matrix<T, state_size, meas_size> K_;

    Eigen::Matrix<T, state_size, state_size> F_;
    Eigen::Matrix<T, meas_size, state_size> H_;

    Eigen::Matrix<T, state_size, 1> x_;
    Eigen::Matrix<T, meas_size, 1> z_;

 public:
    ExtendedKalmanFilter() {
        Q_.setZero();
        R_.setZero();
        P_.setZero();
        S_.setZero();
        K_.setZero();
        x_.setZero();
        z_.setZero();
        F_.setZero();
        H_.setZero();
    }

    size_t StateSize() const { return state_size_; }

    size_t MeasSize() const { return meas_size_; }

    Eigen::Matrix<T, state_size, 1> State() const { return x_; }
    void State(const Eigen::Matrix<T, state_size, 1> &state) { x_ = state; }

    Eigen::Matrix<T, state_size, state_size> StateCov() const { return P_; }
    void StateCov(const Eigen::Matrix<T, state_size, state_size> &cov) {
        P_ = cov;
    }

    Eigen::Matrix<T, meas_size, 1> Meas() const { return z_; }
    void Meas(const Eigen::Matrix<T, meas_size, 1> &meas) { z_ = meas; }

    Eigen::Matrix<T, meas_size, meas_size> MeasCov() const { return S_; }
    void MeasCov(const Eigen::Matrix<T, meas_size, meas_size> &cov) {
        S_ = cov;
    }

    std::function<
        Eigen::Matrix<T, state_size, 1>(Eigen::Matrix<T, state_size, 1>)>
    Transition() const {
        return transition;
    }
    void Transition(const std::function<Eigen::Matrix<T, state_size, 1>(
                        Eigen::Matrix<T, state_size, 1>)> &f) {
        transition = f;
    }

    std::function<
        Eigen::Matrix<T, meas_size, 1>(Eigen::Matrix<T, state_size, 1>)>
    Observation() const {
        return observation;
    }
    void Observation(const std::function<Eigen::Matrix<T, meas_size, 1>(
                         Eigen::Matrix<T, state_size, 1>)> &f) {
        observation = f;
    }

    Eigen::Matrix<T, state_size, state_size> TransitionCov() const {
        return Q_;
    }
    void TransitionCov(const Eigen::Matrix<T, state_size, state_size> &Q) {
        Q_ = Q;
    }

    Eigen::Matrix<T, meas_size, state_size> ObservationCov() const {
        return R_;
    }
    void ObservationCov(const Eigen::Matrix<T, meas_size, state_size> &R) {
        R_ = R;
    }

    void Transition() {
        // x_ = trainsition(x_);
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    void Observation() {
        Eigen::Matrix<T, meas_size, 1> y = z_ - observation(x_);
        S_ = H_ * P_ * H_.transpose() + R_;

        K_ = P_ * H_.transpose() * S_.inverse();

        x_ = x_ + K_ * y;
        P_ = P_ - K_ * H_ * P_;
    }

    void Step(Eigen::Matrix<T, meas_size, 1> meas) {
        Meas(meas);

        Transition();

        Observation();
    }
};
}  // namespace KF_LIB

#endif  // KALMAN_FILTER_SRC_EXTENDED_KALMAN_FILTER_HPP