// Copyright 2022
#include <gtest/gtest.h>

#include <iostream>

#include "src/random_variable.hpp"

class TestLinearKalmanFilter : public testing::Test {
 public:
    void SetUp(void) override {}
    void TearDown(void) override {}
};

TEST_F(TestLinearKalmanFilter, TestSample) {
    KF_LIB::RandomVariable rv(2);

    Eigen::MatrixXd mean_ = Eigen::Map<Eigen::MatrixXd>({0, 0});
    EXPECT_EQ(rv.Size(), 2);
    EXPECT_EQ(*(rv.Mean()), mean_);
}