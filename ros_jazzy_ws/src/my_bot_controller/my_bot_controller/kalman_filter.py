import numpy as np
import math

class RobotKalmanFilter:
    def __init__(self, dt=0.1):
        self.dt = dt
        self.x = np.zeros((3, 1))  # [x, y, yaw]
        self.P = np.eye(3) * 1.0   # Confidence
        self.Q = np.eye(3) * 0.05  # Process noise
        self.R = np.eye(3) * 0.2   # Measurement noise

    def predict(self, v, omega):
        theta = self.x[2, 0]
        # Update state based on velocity commands
        self.x[0, 0] += v * np.cos(theta) * self.dt
        self.x[1, 0] += v * np.sin(theta) * self.dt
        self.x[2, 0] += omega * self.dt
        
        # Jacobian for EKF
        F = np.array([
            [1, 0, -v * np.sin(theta) * self.dt],
            [0, 1,  v * np.cos(theta) * self.dt],
            [0, 0, 1]
        ])
        self.P = F @ self.P @ F.T + self.Q
        return self.x

    def update(self, z):
        z = np.array(z).reshape(3, 1)
        y = z - self.x
        y[2,0] = math.atan2(math.sin(y[2,0]), math.cos(y[2,0]))

        S = self.P + self.R
        K = self.P @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I_KH = np.eye(3) - K
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T
        return self.x