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
    KF_LIB::LinearKalmanFilter kf(3, 3);
}