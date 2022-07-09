// Liscence
#ifndef KALMAN_FILTER_SRC_LINEAR_KALMAN_FILTER_HPP
#define KALMAN_FILTER_SRC_LINEAR_KALMAN_FILTER_HPP
#include <Eigen/Dense>

namespace KF_LIB {
class LinearKalmanFilter {
 private:
    size_t state_size_;
    size_t meas_size_;

    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd F;
    Eigen::MatrixXd H;
    Eigen::MatrixXd x;
    Eigen::MatrixXd z;
    Eigen::MatrixXd x_pred;
    Eigen::MatrixXd z_pred;
    Eigen::MatrixXd x_hat;
    Eigen::MatrixXd z_hat;
 public:
    LinearKalmanFilter() {}
    LinearKalmanFilter(size_t state_size, size_t meas_size) : state_size_(state_size), meas_size_(meas_size) {
        Q = Eigen::MatrixXd(state_size_, state_size_);
    }


    size_t StateSize() const {return state_size_;}
    size_t MeasSize() const {return meas_size_;}

    void MeasCov(const Eigen::MatrixXd &cov);
};
}  // namespace KF_LIB

#endif //KALMAN_FILTER_SRC_LINEAR_KALMAN_FILTER_HPP