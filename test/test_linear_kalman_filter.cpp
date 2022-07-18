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

    double arr[2] = {1, 2};
    Eigen::MatrixXd state = Eigen::Map<Eigen::MatrixXd>(arr, 2, 1);
    Eigen::MatrixXd meas = Eigen::Map<Eigen::MatrixXd>(arr, 2, 1);

    kf.ObservationModel(H);
    kf.PredictionModel(F);

    kf.StateCov(P);
    kf.ObservationCov(R);
    kf.PredictionCov(Q);

    kf.Meas(meas);
    kf.State(state);

    kf.Estimate();

    EXPECT_EQ(kf.State(), state);
    EXPECT_EQ(kf.Meas(), meas);
}