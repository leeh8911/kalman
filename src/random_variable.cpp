// Copyright 2022

#include "src/random_variable.hpp"

#include <algorithm>

#include <Eigen/Dense>

namespace KF_LIB {

RandomVariable::RandomVariable() : size_(0), mean_(nullptr), std_(nullptr) {}

RandomVariable::RandomVariable(size_t sz)
    : size_(sz), mean_(new Matrix(sz, 1)), std_(new Matrix(sz, sz)) {
    mean_->setZero();
    std_->setIdentity();
}

RandomVariable::RandomVariable(const RandomVariable& rv) 
    : size_(rv.Size()), mean_(new Matrix(rv.Size(), 1)), std_(new Matrix(rv.Size(), rv.Size())) {

        *mean_ = *rv.mean_;
        *std_ = *rv.std_;
}

RandomVariable::~RandomVariable() {
    if (mean_ != nullptr) {
        delete mean_;
    }
    if (std_ != nullptr) {
        delete std_;
    }
}

size_t RandomVariable::Size() const { return size_; }
void RandomVariable::Size(const size_t sz) { size_ = sz; }


Matrix* RandomVariable::Mean() const {
    return mean_;
}
void RandomVariable::Mean(const Matrix* m) {
    *mean_ = *m;
}

Matrix* RandomVariable::Std() const {
    return std_;
}
void RandomVariable::Std(const Matrix* m) {
    *std_ = *m;
}
}  // namespace KF_LIB
