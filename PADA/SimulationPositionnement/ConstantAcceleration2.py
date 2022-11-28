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

    numPt = 600
    centralPos = np.array([0, 0])
    loiterRadius = 50

    actualInitPosition = np.array([0, 0])  # [m]
    actualInitVelocity = np.array([5.5, 0.5])  # tangential [m/s]
    actualInitAccel = np.array([0.0, -0.00])  # Constant speed [m/s2]
    dt = 1  # loiterRadius * np.pi * 2 / np.linalg.norm(actualInitVelocity) / numPt # seconds

    vehicle = vehicle(actualInitPosition, actualInitVelocity, actualInitAccel, loiterRadius)

    GPS_Measure = vehicle.measureGPS()
    vehiclePositions = np.array([(actualInitPosition[0], actualInitPosition[1])])
    GPSPositions = np.array([(GPS_Measure[0], GPS_Measure[1])])

    dt = 2

    # State Transition Matrix
    A = np.array([[1, dt, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, dt],
                  [0, 0, 0, 1]])

    # Control Transition Matrix - Nul
    B = np.array([[0.5 * dt ** 2, 0],
                  [dt, 0],
                  [0, 0.5 * dt ** 2],
                  [0, dt]])

    # Control Vector - Nul
    u = np.zeros((2, 1))

    # Covariance Matrix of Current State
    P = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]]) * 500

    P = np.eye(4) * 500

    # Process Noise Matrix

    Q = np.array([[dt ** 4 / 4, dt ** 3 / 2, 0, 0],
                  [dt ** 3 / 2, dt ** 2    , 0, 0],
                  [0, 0, dt ** 4 / 4, dt ** 3 / 2],
                  [0, 0, dt ** 3 / 2, dt ** 2    ]])* 0.21 ** 2

    # Measurement Transition Matrix
    H = np.array([[1, 0, 0, 0],
                  [0, 0, 1, 0]])

    # Measurement Vector
    m = np.zeros((2, 1))

    # Measurement Uncertainty
    sigma_m = 400
    R = np.eye(2) * sigma_m

    GPS0 = vehicle.measureGPS()
    x = np.array([[GPS0[0], 0, GPS0[1], 0]])
    KFPositions = np.array([[x[0, 0], x[0, 1]]])
    ki = []
    for i in range(numPt):

        vehicle.move(dt,[np.sin(dt*i/50)/0.5,np.sin(dt*i/20-np.pi)/7])
        if i>2:
            error = (np.random.random_sample(2) - 0.5) * 0.025 * 2
            ax = (vehiclePositions[i, 0] - 2 * vehiclePositions[i - 1, 0] + vehiclePositions[i - 2, 0]) / dt ** 2 + \
                 error[0]
            ay = (vehiclePositions[i, 1] - 2 * vehiclePositions[i - 1, 1] + vehiclePositions[i - 2, 1]) / dt ** 2 + \
                 error[1]
            u = np.array([[ax],[ay]])
        vehiclePositions = np.append(vehiclePositions, [[vehicle.actPos[0], vehicle.actPos[1]]], axis=0)
        GPS_Measure = vehicle.measureGPS()
        m = np.array([[GPS_Measure[0], GPS_Measure[1]]])
        x, P, K = KalmanFilter.KalmanFilter(x.transpose(), A, B, u, P, Q, H, m.transpose(), R)
        ki.append(K)
        KFPositions = np.append(KFPositions, [[x[0][0], x[0][2]]], axis=0)
        GPSPositions = np.append(GPSPositions, [[GPS_Measure[0], GPS_Measure[1]]], axis=0)

    KF_diff = []
    GPS_diff = []
    for i in range(0, numPt):
        KF_diff.append(KFPositions[i, :] - vehiclePositions[i, :])
        GPS_diff.append(GPSPositions[i, :] - vehiclePositions[i, :])

    plt.figure(3)
    plt.plot(vehiclePositions[:, 0], vehiclePositions[:, 1], 'b-', label='Actual Position')
    plt.plot(GPSPositions[:, 0], GPSPositions[:, 1], 'r-', label='GPS Position')
    plt.plot(KFPositions[:, 0], KFPositions[:, 1], 'go-', markersize=4, label='Kalman Filter')

    # plt.xlim(-loiterRadius-10, loiterRadius+10)
    # plt.ylim(-loiterRadius-10, loiterRadius+10)
    ax = plt.gca()
    # ax.set_aspect('equal', adjustable='box')
    ax.legend()
    plt.show()

    GPS_diff = np.array(GPS_diff)
    KF_diff = np.array(KF_diff)
    print('GPS mean error: ' + str(np.mean(np.sqrt(GPS_diff[:, 0]**2+GPS_diff[:, 1]**2))))
    print('KF mean error: ' + str(np.mean(np.sqrt(abs(KF_diff[:, 0])**2+abs(KF_diff[:, 1])**2)  )))
    print('GPS X-axis mean error: ' + str(np.mean(abs(GPS_diff[:, 0]))))
    print('GPS Y-axis mean error: ' + str(np.mean(abs(GPS_diff[:, 1]))))
    #print('KF Y-axis mean error: ' + str(np.mean(abs(KF_diff[:, 1]))))
    plt.figure(3)
    plt.plot(list(range(0, numPt)), np.sqrt(abs(GPS_diff[:, 0])**2+abs(GPS_diff[:, 1])**2)  , 'r-', label='GPS Position')
    plt.plot(list(range(0, numPt)), np.sqrt(abs(KF_diff[:, 0])**2+abs(KF_diff[:, 1])**2) , 'g-', label='Kalman Filter')
    #plt.plot(list(range(0, numPt)), GPS_diff[:, 0], 'r-', label='GPS Position (x)')np.mean(np.sqrt(abs(GPS_diff[:, 0])**2+abs(GPS_diff[:, 1])**2)  )
    #plt.plot(range(0, numPt), GPS_diff[:, 1], 'y-', label='GPS Position (y)')
    #plt.plot(range(0, numPt), KF_diff[:, 0], 'g-', label='Kalman Filter (x)')
    #plt.plot(range(0, numPt), KF_diff[:, 1], 'b-', label='Kalman Filter (y)')
    ax = plt.gca()
    ax.legend()
    plt.show()

    ki = np.array(ki)
    plt.figure(3)
    plt.plot(list(range(0, numPt)), ki[:, 0], 'r-', label='Kalman gain [K]')
    ax = plt.gca()
    ax.legend()
    plt.show()
