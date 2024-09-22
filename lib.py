import numpy as np

class KalmanFilter:
    """ This class implements a Kalman Filter.
    """

    def __init__(self, F, H, Q, R, x0, P0):
        self.F = F   # State transition matrix
        self.H = H   # Observation matrix
        self.Q = Q   # Process noise covariance
        self.R = R   # Measurement noise covariance
        self.x = x0  # State estimate
        self.P = P0  # Error covariance

    def Step(self, y):
        """ Execute a step of the Kalman Filter.

        Inputs:
            y: current measurement
            x_k_1: previous state estimate
            P_k_1: previous error covariance
        
        Returns:
            x_up: updated state estimate
            P_up: updated error covariance
        """

        # Predict
        x_k = self.F @ self.x
        P_k = self.F @ self.P @ self.F.T + self.Q

        # Update
        K = P_k @ self.H.T @ np.linalg.inv(self.H @ P_k @ self.H.T + self.R)
        self.x = x_k + K @ (y - self.H @ x_k)
        self.P = P_k - K @ self.H @ P_k

class DCMotor:
    """ This class represents a DC motor with electrical and mechanical characteristics. """

    def __init__(self, J, b, kt, ke, R, L, T):
        self.J = J                  # Rotational inertia (kg*m^2)
        self.b = b                  # Damping factor (Nm/(rad/s))
        self.kt = kt                # Torque sensitivity (Nm/A)
        self.ke = ke                # Back EMF constant (V/(rad/s))
        self.R = R                  # Terminal resistance (Ohm)
        self.L = L                  # Terminal inductance (H)
        self.T = T                  # Time step (s)
        self.omega = 0              # Angular velocity (rad/s)
        self.theta = 0              # Angular position (rad)
        self.i = 0                  # Current (A)

    def Step(self, V):
        """ Update the angular velocity and current of the motor based on the applied voltage V. """
        
        # Calculate the derivatives
        omega_dot = (self.kt * self.i - self.b * self.omega) / self.J
        i_dot = (V - self.ke * self.omega - self.R * self.i) / self.L

        # Update angular velocity, position, and current using Euler integration
        self.omega += omega_dot * self.T
        self.theta += self.omega * self.T
        self.i += i_dot * self.T