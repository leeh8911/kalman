// Copyright 2022
#ifndef KALMAN_FILTER_SRC_ANALYTIC_FUNCTIONS_HPP
#define KALMAN_FILTER_SRC_ANALYTIC_FUNCTIONS_HPP
#include <cmath>

namespace KF_LIB {

template <typename T>
class AnalyticFunction {
    std::function<T(T)> f;
    std::function<T(T)> derivative;

    T result;
    T diff;

 public:
    AnalyticFunction() : f(nullptr), derivative(nullptr) {}
    virtual ~AnalyticFunction() {}
    T operator()(const T src) {
        result = f(src);
        diff = derivative(src);
        return result;
    }
    T Difference() const { return diff; }
    AnalyticFunction operator+(const AnalyticFunction& lhs) {
        AnalyticFunction des;
        des.f = f + lhs.f;
        des.derivative = derivative + lhs.derivative;
        return des;
    }
    AnalyticFunction operator-(const AnalyticFunction& lhs) {
        AnalyticFunction des;
        des.f = f - lhs.f;
        des.derivative = derivative - lhs.derivative;
        return des;
    }
    AnalyticFunction operator*(const AnalyticFunction& lhs) {
        AnalyticFunction des;
        des.f = f * lhs.f;
        des.derivative = derivative * lhs.f + f * lhs.derivative;
        return des;
    }
    AnalyticFunction operator/(const AnalyticFunction& lhs) {
        AnalyticFunction des;
        des.f = f / lhs.f;
        des.diff = (diff * lhs.f - f * lhs.diff) / (lhs.f * lhs.f);
        return des;
    }
};

template <typename T = double>
class Sin : public AnalyticFunction<T> {
    std::function<T(T)> f = (T(*)(T))&std::sin;
    std::function<T(T)> derivative = (T(*)(T))&std::cos;
};

}  // namespace KF_LIB

#endif  // KALMAN_FILTER_SRC_ANALYTIC_FUNCTIONS_HPP