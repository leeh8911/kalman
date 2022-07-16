// Copyright 2022

#include "src/multi_tracker.hpp"

#include <vector>

#include <Eigen/Dense>

#include "src/IEstimator.hpp"
#include "src/hungarian.hpp"

namespace MT {

MultiTracker::Update() {

    std::vector<size_t> assign = Match();

    for(auto est : targets) {
        est.
    }

}

std::vector<size_t> MultiTracker::Match() {

}
}  // namespace MT
