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

    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;

    Eigen::MatrixXd state_;
    Eigen::MatrixXd meas_;

    Eigen::MatrixXd K;

 public:
    LinearKalmanFilter(size_t state_size, size_t meas_size);

    void PredictionModel(const Eigen::MatrixXd& m);
    void ObservationModel(const Eigen::MatrixXd& m);

    Eigen::MatrixXd PredictionModel() const;
    Eigen::MatrixXd ObservationModel() const;

    void PredictionCov(const Eigen::MatrixXd& m);
    void ObservationCov(const Eigen::MatrixXd& m);

    Eigen::MatrixXd PredictionCov() const;
    Eigen::MatrixXd ObservationCov() const;

    void StateCov(const Eigen::MatrixXd& m);
    void MeasCov(const Eigen::MatrixXd& m);

    Eigen::MatrixXd StateCov() const;
    Eigen::MatrixXd MeasCov() const;

    void State(const Eigen::MatrixXd& m);
    void Meas(const Eigen::MatrixXd& m);

    Eigen::MatrixXd State() const;
    Eigen::MatrixXd Meas() const;

    void Estimate();
    void Estimate(const Eigen::MatrixXd& m);

    std::ostream& operator<<(std::ostream& os);
    friend std::ostream& operator<<(std::ostream& os, LinearKalmanFilter& kf);
};

}  // namespace KF_LIB

#endif  // KALMAN_FILTER_SRC_LINEAR_KALMAN_FILTER_HPP