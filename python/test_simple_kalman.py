import numpy as np
import matplotlib.pyplot as plt
import ctypes

lib = ctypes.CDLL("build/src/libKALMAN.so")

lib.printHello()
