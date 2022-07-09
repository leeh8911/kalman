#include <gtest/gtest.h>
#include <src/linear_kalman_filter.hpp>

class TestLinearKalmanFilter : public testing::Test {
 public:
    void SetUp(void) override {}
    void TearDown(void) override {}
};

TEST_F(TestLinearKalmanFilter, HelloWorldTest) {
    KF_LIB::LinearKalmanFilter lkf(1, 1);

    EXPECT_EQ(lkf.StateSize(), 1);
    EXPECT_EQ(lkf.MeasSize(), 1);
}