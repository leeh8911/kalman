#include <gtest/gtest.h>
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
  // lkf.State(Eigen::Matrix<double, 3, 1>{});

  lkf.Meas(Eigen::Matrix<double, 2, 1>{10, 1});
  meas = lkf.Meas();
  EXPECT_DOUBLE_EQ(meas[0], 10);
  EXPECT_DOUBLE_EQ(meas[1], 1);
}