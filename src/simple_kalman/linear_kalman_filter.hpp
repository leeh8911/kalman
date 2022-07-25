// Copyright 2022
#ifndef KALMAN_FILTER_SRC_LINEAR_KALMAN_FILTER_HPP
#define KALMAN_FILTER_SRC_LINEAR_KALMAN_FILTER_HPP

#include <Eigen/Dense>

namespace KF_LIB {
template<typename T, int size>
class Gauss {
    Eigen::Matrix<T, size, 1> mean_;
    Eigen::Matrix<T, size, size> std_;

    int size_{size};
 public:
    Gauss();
    ~Gauss();

    Eigen::Matrix<T, size, 1> Mean() const;
    void Mean(const Eigen::Matrix<T, size, 1>& mean);
};

}  // namespace KF_LIB

#endif  // KALMAN_FILTER_SRC_LINEAR_KALMAN_FILTER_HPP