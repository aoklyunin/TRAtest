import numpy as np


def getDHMatrix(a, alpha, d, theta):
    return np.matrix(
        [[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
         [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
         [0, np.sin(theta), np.cos(theta), d],
         [0, 0, 0, 1]
         ])
