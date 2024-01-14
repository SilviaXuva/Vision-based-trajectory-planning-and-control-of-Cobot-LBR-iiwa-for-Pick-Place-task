import numpy as np

def MeanSquareError(x: np.ndarray, y: np.ndarray):
    summation = 0 
    n = len(x)
    for i in range (0, n):
        difference = x[i] - y[i]
        squared_difference = difference**2
        summation = summation + squared_difference
        MSE = summation/n
    return MSE