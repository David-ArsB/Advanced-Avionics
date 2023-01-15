'''
Objective: Accurately estimate aircraft position in the air at a given time.
Assumptions:
Measurement inputs: GPS (x, y, z), IMU (ddx_ddt, ddy_ddt, ddz_ddt)
'''

import numpy as np


def stateExtrapolationEquation(ECSS, u_n, w_n, F, G):
    '''

    :param x_n0: Estimated current system states
    :param u_n: Input variables
    :param w_n: Process Noise
    :param F: State Transition Matrix
    :param G: Input Transition Matrix

    :return: x_n1: Predicted System State
    '''


def KalmanFilter(x_00, A, B, u, P, Q, H, m, R):
    '''
    
    '''

    x_10 = A @ x_00 + B @ u
    P = A @ P @ A.transpose() + Q
    # Kalman Gain
    j = np.linalg.inv(H @ P @ H.transpose() + R)
    K = P @ H.transpose() @ j
    x_11 = x_10 + K @ (m - H @ x_10)
    I = np.eye(4)
    P = (I - K @ H) @ P @ (I - K @ H).transpose() + K @ R @ K.transpose()

    return x_11.transpose(), P, K


if __name__ == "__main__":
    dt = 1

    # State Transition Matrix
    A = np.matrix([[1, 0, dt, 0],
                   [0, 1, 0, dt],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])

    # Control Transition Matrix
    B = np.zeros((4, 2))

    # Control Vector
    u = np.zeros((2, 1))

    # Covariance Matrix of Current State
    P = np.zeros((4, 4))

    # Process Noise Matrix
    Qa = np.zeros((4, 4))
    Q = A * Qa * A.transpose()

    # Measurement Transition Matrix
    H = np.matrix([[1, 0, 0, 0],
                   [0, 1, 0, 0]])

    # Measurement Vector
    m = np.zeros((2, 1))

    # Measurement Uncertainty
    sigma_m = 6
    R = np.eye(2) * sigma_m

    # Kalman Gain
    j = np.linalg.inv(H * P * H.transpose() + R)
    K = P * H.transpose() * j
