// Liscence

#include "linear_kalman_filter.hpp"

#include <cassert>
#include <utility>
#include <Eigen/Dense>

namespace KF_LIB {

    void LinearKalmanFilter::MeasCov(const Eigen::MatrixXd &cov) {
        assert((cov.rows() == R.rows() ) && (cov.cols() == R.cols()));
        R = std::move(cov);
    }
}  // namespace KF_LIB
