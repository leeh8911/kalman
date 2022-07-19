import numpy as np
import matplotlib.pyplot as plt
import ctypes


lib = ctypes.CDLL("build/src/libKALMAN.so")
kf = lib.CreateLinearKalmanFilter(2, 2)

F = np.array([[1, 1], [0, 1]])
H = np.array([[1, 0], [0, 1]])

Q = np.array([[1, 0], [0, 1]])
R = np.array([[1, 0], [0, 1]])

lib.ConvertNumpyToEigen(F)
