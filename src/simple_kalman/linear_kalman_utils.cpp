

#include "src/simple_kalman/linear_kalman_utils.hpp"

#include <eigen3/unsupported/Eigen/CXX11/Tensor>
#include <iostream>
#include <vector>

#include "src/simple_kalman/linear_kalman_filter.hpp"

KF_LIB::LinearKalmanFilter* CreateLinearKalmanFilter(size_t state_size,
                                                     size_t meas_size) {
    KF_LIB::LinearKalmanFilter* p =
        new KF_LIB::LinearKalmanFilter(state_size, meas_size);

    return p;
}

void DeleteLinearKalmanFilter(KF_LIB::LinearKalmanFilter* p) { delete p; }

void SetKalmanParams(KF_LIB::LinearKalmanFilter* p, const Eigen::MatrixXd& F,
                     const Eigen::MatrixXd& H, const Eigen::MatrixXd& Q,
                     const Eigen::MatrixXd& R) {
    p->PredictionModel(F);
    p->PredictionCov(Q);
    p->ObservationModel(H);
    p->ObservationCov(R);
}

void SetState(KF_LIB::LinearKalmanFilter* p, const Eigen::MatrixXd& state) {
    p->State(state);
}

Eigen::MatrixXd GetState(KF_LIB::LinearKalmanFilter* p) { return p->State(); }

void SetMeas(KF_LIB::LinearKalmanFilter* p, const Eigen::MatrixXd& meas) {
    p->Meas(meas);
}

void UpdateKalman(KF_LIB::LinearKalmanFilter* p) { p->Estimate(); }

void UpdateKalman(KF_LIB::LinearKalmanFilter* p, const Eigen::MatrixXd& meas) {
    p->Estimate(meas);
}

void* ConvertNumpyToEigen(const double* arr, int row, int col) {
    Eigen::MatrixXd* dst = new Eigen::MatrixXd;
    dst->setZero(row, col);

    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            dst->coeffRef(i, j) = arr[i * col + j];
        }
    }

    std::cout << *dst << std::endl;

    return static_cast<void*>(dst);
}
