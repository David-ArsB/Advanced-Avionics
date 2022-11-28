import matplotlib.pyplot as plt
import numpy as np

import KalmanFilter
from Vehicle import vehicle

if __name__ == "__main__":

    """
    Simulate a vehicle fying in loiter around a central point.

    The vehicle measures it's position through GPS positioning and IMU data.
        - GPS Data is randomized given an uncertainty around the vehicle's actual position; it is a simulated value.
        - The vehicle doesn't know its actual position.
        - IMU data will also be randomized given an uncertainty.

    """

    numPt = 25
    centralPos = np.array([0, 0])
    loiterRadius = 50

    actualInitPosition = np.array([0, 0])  # [m]
    actualInitVelocity = np.array([-5, 5])  # tangential [m/s]
    actualInitAccel = np.array([0., 0.])  # Constant speed [m/s2]
    dt = 1  # loiterRadius * np.pi * 2 / np.linalg.norm(actualInitVelocity) / numPt # seconds

    vehicle = vehicle(actualInitPosition, actualInitVelocity, actualInitAccel, loiterRadius)

    GPS_Measure = vehicle.measureGPS()
    vehiclePositions = np.array([(actualInitPosition[0], actualInitPosition[1])])
    GPSPositions = np.array([(GPS_Measure[0], GPS_Measure[1])])

    dt = 1

    # State Transition Matrix
    A = np.array([[1, 0, dt, 0],
                  [0, 1, 0, dt],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])

    # Control Transition Matrix - Nul
    B = np.zeros((4, 2))

    # Control Vector - Nul
    u = np.zeros((2, 1))

    # Covariance Matrix of Current State
    P = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]]) * 500

    # Process Noise Matrix
    Qa = np.array([[0, 0, 0, 0],
                   [0, 0, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]]) * 0.1

    Q = A * Qa * A.transpose()

    Q = np.array([[dt ** 3 / 3, dt ** 2 / 2, 0, 0],
                  [0, 0, dt ** 3 / 3, dt ** 2 / 2],
                  [dt ** 2 / 2, dt , 0, 0],
                  [0, 0, dt ** 2 / 2, dt ]]) * 0.2 ** 2

    Q = np.array([[dt ** 4 / 4, 0          , dt ** 3 / 2, 0          ],
                  [0          , dt ** 4 / 4, 0          , dt ** 3 / 2],
                  [dt ** 3 / 2, 0          , dt ** 2    , 0          ],
                  [0          , dt ** 3 / 2, 0          , dt ** 2]]) * 0.2 ** 2



    # Measurement Transition Matrix
    H = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0]])

    # Measurement Vector
    m = np.zeros((2, 1))

    # Measurement Uncertainty
    sigma_m = 9
    R = np.eye(2) * sigma_m

    GPS0 = vehicle.measureGPS()
    x = np.array([[GPS0[0], GPS0[1], 0, 0]])
    KFPositions = np.array([[x[0, 0], x[0, 1]]])
    ki = []
    for i in range(numPt):
        vehicle.move(dt)
        vehiclePositions = np.append(vehiclePositions, [[vehicle.actPos[0], vehicle.actPos[1]]], axis=0)
        GPS_Measure = vehicle.measureGPS()
        m = np.array([[GPS_Measure[0], GPS_Measure[1]]])
        x, P, K = KalmanFilter.KalmanFilter(x.transpose(), A, B, u, P, Q, H, m.transpose(), R)
        ki.append(K)
        KFPositions = np.append(KFPositions, [[x[0][0], x[0][1]]], axis=0)
        GPSPositions = np.append(GPSPositions, [[GPS_Measure[0], GPS_Measure[1]]], axis=0)

    KF_diff = []
    GPS_diff = []
    for i in range(0, numPt):
        KF_diff.append(KFPositions[i, :] - vehiclePositions[i, :])
        GPS_diff.append(GPSPositions[i, :] - vehiclePositions[i, :])

    plt.figure(3)
    plt.plot(vehiclePositions[:, 0], vehiclePositions[:, 1], 'b-', label='Actual Position')
    plt.plot(GPSPositions[:, 0], GPSPositions[:, 1], 'r-', label='GPS Position')
    plt.plot(KFPositions[:, 0], KFPositions[:, 1], 'g-', label='Kalman Filter')

    # plt.xlim(-loiterRadius-10, loiterRadius+10)
    # plt.ylim(-loiterRadius-10, loiterRadius+10)
    ax = plt.gca()
    # ax.set_aspect('equal', adjustable='box')
    ax.legend()
    plt.show()

    GPS_diff = np.array(GPS_diff)
    KF_diff = np.array(KF_diff)
    print('GPS X-axis mean error: ' + str(np.mean(abs(GPS_diff[:, 0]))))
    print('GPS Y-axis mean error: ' + str(np.mean(abs(GPS_diff[:, 1]))))
    print('KF X-axis mean error: ' + str(np.mean(abs(KF_diff[:, 0]))))
    print('KF Y-axis mean error: ' + str(np.mean(abs(KF_diff[:, 1]))))
    plt.figure(3)
    plt.plot(list(range(0, numPt)), GPS_diff[:, 0], 'r-', label='GPS Position (x)')
    plt.plot(range(0, numPt), GPS_diff[:, 1], 'y-', label='GPS Position (y)')
    plt.plot(range(0, numPt), KF_diff[:, 0], 'g-', label='Kalman Filter (x)')
    plt.plot(range(0, numPt), KF_diff[:, 1], 'b-', label='Kalman Filter (y)')
    ax = plt.gca()
    ax.legend()
    plt.show()

    ki = np.array(ki)
    plt.figure(3)
    plt.plot(list(range(0, numPt)), ki[:, 0], 'r-', label='Kalman gain [K]')
    ax = plt.gca()
    ax.legend()
    plt.show()
