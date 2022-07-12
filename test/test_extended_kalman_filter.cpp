// Copyright 2022
#include <gtest/gtest.h>

#include <iostream>
#include <src/extended_kalman_filter.hpp>

class TestExtendedKalmanFilter : public testing::Test {
 public:
    void SetUp(void) override {}
    void TearDown(void) override {}
};

TEST_F(TestExtendedKalmanFilter, CreateSizeTest) {
    KF_LIB::ExtendedKalmanFilter<double, 2, 2> ekf;

    EXPECT_EQ(ekf.StateSize(), 2);
    EXPECT_EQ(ekf.MeasSize(), 2);
}

TEST_F(TestExtendedKalmanFilter, CheckStateSize) {
    KF_LIB::ExtendedKalmanFilter<double, 2, 2> ekf;

    auto state = ekf.State();
    EXPECT_EQ(state.rows(), 2);
    EXPECT_EQ(state.cols(), 1);

    // If wrong size matrix input, then occurs build error.
    // ekf.State(Eigen::Matrix<double, 3, 1>{});

    ekf.State(Eigen::Matrix<double, 2, 1>{10, 1});
    state = ekf.State();
    EXPECT_DOUBLE_EQ(state[0], 10);
    EXPECT_DOUBLE_EQ(state[1], 1);
}

TEST_F(TestExtendedKalmanFilter, CheckMeasSize) {
    KF_LIB::ExtendedKalmanFilter<double, 2, 2> ekf;

    auto meas = ekf.Meas();
    EXPECT_EQ(meas.rows(), 2);
    EXPECT_EQ(meas.cols(), 1);

    // If wrong size matrix input, then occurs build error.
    // ekf.Meas(Eigen::Matrix<double, 3, 1>{});

    ekf.Meas(Eigen::Matrix<double, 2, 1>{10, 1});
    meas = ekf.Meas();
    EXPECT_DOUBLE_EQ(meas[0], 10);
    EXPECT_DOUBLE_EQ(meas[1], 1);
}

TEST_F(TestExtendedKalmanFilter, SimpleCheckKalmanFilterTwoStep) {
    KF_LIB::ExtendedKalmanFilter<double, 2, 2> ekf;

    ekf.Meas(Eigen::Matrix<double, 2, 1>(10, 10));

    Eigen::Matrix<double, 2, 2> F, H, Q, R, P;
    F << 1, 1, 0, 1;
    H << 1, 0, 0, 1;
    P << 1, 0, 0, 1;
    Q << 1, 0, 0, 1;
    R << 1, 0, 0, 1;

    ekf.StateCov(P);

    // ekf.TransitionMatrix(F);
    // ekf.ObservationMatrix(H);

    ekf.TransitionCov(Q);
    ekf.ObservationCov(R);

    ekf.Transition();
    ekf.Observation();

    auto state = ekf.State();
    std::cout << state << "\n";
}

TEST_F(TestExtendedKalmanFilter, SimpleCheckKalmanFilterOneStep) {
    KF_LIB::ExtendedKalmanFilter<double, 2, 2> ekf;

    Eigen::Matrix<double, 2, 2> F, H, Q, R, P;
    F << 1, 1, 0, 1;
    H << 1, 0, 0, 1;
    P << 1, 0, 0, 1;
    Q << 1, 0, 0, 1;
    R << 1, 0, 0, 1;

    ekf.StateCov(P);

    ekf.Transition(nullptr);
    ekf.Observation(nullptr);

    ekf.TransitionCov(Q);
    ekf.ObservationCov(R);

    ekf.Step(Eigen::Matrix<double, 2, 1>(10, 10));

    auto state = ekf.State();
    std::cout << state << "\n";
}