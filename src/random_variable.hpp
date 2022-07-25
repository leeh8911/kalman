// Copyright 2022
#ifndef KALMAN_FILTER_SRC_RANDOM_VARIABLE_HPP_
#define KALMAN_FILTER_SRC_RANDOM_VARIABLE_HPP_

#include <Eigen/Dense>

namespace KF_LIB {

using Matrix = Eigen::MatrixXd;

class RandomVariable {
    Matrix* mean_;
    Matrix* std_;
    size_t size_;

 public:
    RandomVariable();
    ~RandomVariable();
    RandomVariable(size_t sz);
    RandomVariable(const RandomVariable& rv);

    size_t Size() const;
    void Size(const size_t sz);

    Matrix* Mean() const;
    void Mean(const Matrix* m);

    Matrix* Std() const;
    void Std(const Matrix* m);
};

}  // namespace KF_LIB

#endif  // KALMAN_FILTER_SRC_RANDOM_VARIABLE_HPP_