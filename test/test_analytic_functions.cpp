// Copyright 2022
#include <gtest/gtest.h>

#include <iostream>
#include <src/analytic_functions.hpp>

class TestAnalyticFunctions : public testing::Test {
 public:
    void SetUp(void) override {}
    void TearDown(void) override {}
};

TEST_F(TestAnalyticFunctions, SinTest) {
    KF_LIB::Sin<double> ksin;

    double result = ksin(30 * M_PI / 180);
    double diff = ksin.Difference();

    EXPECT_DOUBLE_EQ(result, 0.5);
    EXPECT_DOUBLE_EQ(diff, sqrt(3) / 2);
}