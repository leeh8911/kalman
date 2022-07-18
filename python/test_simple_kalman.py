import numpy as np
import matplotlib.pyplot as plt
import ctypes


class LinearKalmanFilter(object):
    def __init__(self, state_size, meas_size, F, H, Q, R):
        lib = ctypes.CDLL("build/src/libKALMAN.so")
        self.obj = lib.CreateLinearKalmanFilter(state_size, meas_size)

        lib.SetKalmanParams(self.obj, F, H, Q, R)


F = np.array([[1, 1], [0, 1]])
H = np.array([[1, 0], [0, 1]])

Q = np.array([[1, 0], [0, 1]])
R = np.array([[1, 0], [0, 1]])

lkf = LinearKalmanFilter(2, 2, F, H, Q, R)
