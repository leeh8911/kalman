// Copyright 2022
#include <gtest/gtest.h>

#include <iostream>
#include <src/linear_kalman_filter.hpp>

class TestLinearKalmanFilter : public testing::Test {
 public:
    void SetUp(void) override {}
    void TearDown(void) override {}
};

TEST_F(TestLinearKalmanFilter, CreateSizeTest) {
    KF_LIB::LinearKalmanFilter<double, 2, 2> lkf;

    EXPECT_EQ(lkf.StateSize(), 2);
    EXPECT_EQ(lkf.MeasSize(), 2);
}

TEST_F(TestLinearKalmanFilter, CheckStateSize) {
    KF_LIB::LinearKalmanFilter<double, 2, 2> lkf;

    auto state = lkf.State();
    EXPECT_EQ(state.rows(), 2);
    EXPECT_EQ(state.cols(), 1);

    // If wrong size matrix input, then occurs build error.
    // lkf.State(Eigen::Matrix<double, 3, 1>{});

    lkf.State(Eigen::Matrix<double, 2, 1>{10, 1});
    state = lkf.State();
    EXPECT_DOUBLE_EQ(state[0], 10);
    EXPECT_DOUBLE_EQ(state[1], 1);
}

TEST_F(TestLinearKalmanFilter, CheckMeasSize) {
    KF_LIB::LinearKalmanFilter<double, 2, 2> lkf;

    auto meas = lkf.Meas();
    EXPECT_EQ(meas.rows(), 2);
    EXPECT_EQ(meas.cols(), 1);

    // If wrong size matrix input, then occurs build error.
    // lkf.Meas(Eigen::Matrix<double, 3, 1>{});

    lkf.Meas(Eigen::Matrix<double, 2, 1>{10, 1});
    meas = lkf.Meas();
    EXPECT_DOUBLE_EQ(meas[0], 10);
    EXPECT_DOUBLE_EQ(meas[1], 1);
}

TEST_F(TestLinearKalmanFilter, SimpleCheckKalmanFilterTwoStep) {
    KF_LIB::LinearKalmanFilter<double, 2, 2> lkf;

    lkf.Meas(Eigen::Matrix<double, 2, 1>(10, 10));

    Eigen::Matrix<double, 2, 2> F, H, Q, R, P;
    F << 1, 1, 0, 1;
    H << 1, 0, 0, 1;
    P << 1, 0, 0, 1;
    Q << 1, 0, 0, 1;
    R << 1, 0, 0, 1;

    lkf.StateCov(P);

    lkf.TransitionMatrix(F);
    lkf.ObservationMatrix(H);

    lkf.TransitionCov(Q);
    lkf.ObservationCov(R);

    lkf.Transition();
    lkf.Observation();

    auto state = lkf.State();
    std::cout << state << "\n";
}

TEST_F(TestLinearKalmanFilter, SimpleCheckKalmanFilterOneStep) {
    KF_LIB::LinearKalmanFilter<double, 2, 2> lkf;

    Eigen::Matrix<double, 2, 2> F, H, Q, R, P;
    F << 1, 1, 0, 1;
    H << 1, 0, 0, 1;
    P << 1, 0, 0, 1;
    Q << 1, 0, 0, 1;
    R << 1, 0, 0, 1;

    lkf.StateCov(P);

    lkf.TransitionMatrix(F);
    lkf.ObservationMatrix(H);

    lkf.TransitionCov(Q);
    lkf.ObservationCov(R);

    lkf.Step(Eigen::Matrix<double, 2, 1>(10, 10));

    auto state = lkf.State();
    std::cout << state << "\n";
}