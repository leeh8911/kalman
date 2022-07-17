// Copyright 2022
#include <gtest/gtest.h>

#include <iostream>

#include "src/simple_kalman/linear_kalman_filter.hpp"

class TestLinearKalmanFilter : public testing::Test {
 public:
    void SetUp(void) override {}
    void TearDown(void) override {}
};

TEST_F(TestLinearKalmanFilter, TestSample) {
    KF_LIB::LinearKalmanFilter kf(2, 2);

    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(2, 2);

    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(2, 2);

    Eigen::MatrixXd meas(2, 1);
    meas = {1, 2};

    kf.ObservationModel(H);
    kf.PredictionModel(F);

    std::cout << kf.State().transpose() << std::endl;
    kf.Estimate();
    std::cout << kf.State().transpose() << std::endl;
}