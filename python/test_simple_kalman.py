import numpy as np
import matplotlib.pyplot as plt
import ctypes


lib = ctypes.CDLL("build/src/libKALMAN.so")
kf = lib.CreateLinearKalmanFilter(2, 2)

F = np.array([[1, 1], [0, 1]])
H = np.array([[1, 0], [0, 1]])

Q = np.array([[1, 0], [0, 1]])
R = np.array([[1, 0], [0, 1]])

p = [1, 1, 0, 1]

ConvertArrayToEigen = lib.ConvertArrayToEigen
double_array = ctypes.c_double * 4
arr = double_array(1, 1, 0, 1)
ConvertArrayToEigen.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
ConvertArrayToEigen.restype = ctypes.c_void_p

lib.SetState(kf, arr, 2, 2)
