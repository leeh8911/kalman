// Copyright 2022

#include "src/simple_kalman/linear_kalman_filter.hpp"

#include <Eigen/Dense>
#include <cassert>
#include <utility>
#include <memory>

namespace KF_LIB {

template<typename T, int size>
Gauss<T, size>::Gauss() {
    mean_.setZero();
    std_.setIdentity();
}

template<typename T, int size>
Gauss<T, size>::~Gauss() {
}


template<typename T, int size>
Eigen::Matrix<T, size, 1> Gauss<T, size>::Mean() const {
    return mean_;
}

template<typename T, int size>
void Gauss<T, size>::Mean(const Eigen::Matrix<T, size, 1>& mean) {
    mean_ = mean;
}

template class Gauss<double, 3>;

}  // namespace KF_LIB
