// Copyright 2022
#ifndef SRC_MULTI_TRACKER_HPP_
#define SRC_MULTI_TRACKER_HPP_

#include <Eigen/Dense>
#include <vector>

#include "IEstimator.hpp"

namespace MT {

class MultiTracker {
    using MEAS = Eigen::MatrixXd;

    std::vector<IEstimator*> targets;
    std::vector<MEAS> measures;

    size_t meas_size_;
    size_t state_size_;

    std::vector<size_t> Match();

 public:
    MultiTracker(size_t state_size, size_t meas_size)
        : state_size_(state_size), meas_size_(meas_size) {}
    void Update();
};

}  // namespace MT

#endif  // SRC_MULTI_TRACKER_HPP_
