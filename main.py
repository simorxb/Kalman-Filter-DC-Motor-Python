import numpy as np
from lib import KalmanFilter, DCMotor
import matplotlib.pyplot as plt

def main():
    # -------- Configuration --------

    # Simulation parameters

    time_step = 0.001
    end_time = 5
    length = round(end_time/time_step)

    t = np.zeros(length)
    omega = np.zeros(length)
    i = np.zeros(length)
    theta = np.zeros(length)
    noise = np.zeros(length)
    omega_est = np.zeros(length)
    V = np.zeros(length)

    # DC Motor

    m = 0.1             # Mass of the disc (kg)
    r = 0.05            # Radius of the disc (m)
    J = 0.5 * m * r**2  # Moment of inertia of the disc (kg*m^2)
    b = 0.0000095       # Viscous friction coefficient (N*m*s)
    kt = 0.0187         # Torque constant (N*m/A)
    R = 0.6             # Armature resistance (Ohm)
    L = 0.35 / 1000     # Armature inductance (H)
    ke = 0.0191         # Back EMF constant (V*s/rad)

    # Initialize DC_Motor with given parameters
    dc = DCMotor(J, b, kt, ke, R, L, time_step)

    # Kalman Filter

    T = 0.01            # Sampling time (s)
    H = np.array([[1, 0]])  # Measurement matrix
    F = np.array([[1, T], [0, 1]])  # State transition matrix
    G = np.array([[0.5 * T**2], [T]])  # Input matrix

    s_alpha = 300       # Angular acceleration noise standard deviation
    s_theta = 1         # Measurement noise standard deviation

    Q = G @ G.T * s_alpha**2  # Process noise covariance matrix
    RR = s_theta**2     # Measurement noise covariance

    P0 = np.zeros((2, 2))  # Initial error covariance matrix
    x0 = np.array([[0], [0]])  # Initial state estimate [theta; omega]

    # Initialize Kalman Filter
    kf = KalmanFilter(F, H, Q, RR, x0, P0)

    # Iterate through time steps
    for idx in range(0, length):
        t[idx] = idx*time_step

        # Set input voltage
        if t[idx] > 1:
            V[idx] = 12
        else:
            V[idx] = 0
        
        # Execute a DC motor step
        dc.Step(V[idx])

        # Store variables
        theta[idx] = dc.theta
        omega[idx] = dc.omega
        i[idx] = dc.i

        # Noise
        noise[idx] = np.random.normal(0, s_theta)

        # Add noise to measurement
        theta_meas = dc.theta + noise[idx]

        # Kalman filter step
        if idx % np.round(T / time_step) == 0:
            kf.Step(theta_meas)

        omega_est[idx] = kf.x[1]

    # Plot angular velocity comparison
    plt.figure(1)
    plt.plot(t, omega, label="Actual")
    plt.plot(t, omega_est, label="Estimated by Kalman Filter")
    plt.xlabel("Time [s]")
    plt.ylabel("Angular Velocity [rad/s]")
    plt.title("Angular Velocity Comparison")
    plt.legend()
    plt.grid()

    # Plot measurement noise
    plt.figure(2)
    plt.plot(t, noise*180/np.pi)
    plt.xlabel("Time [s]")
    plt.ylabel("Noise [deg]")
    plt.title("Measurement Noise")
    plt.legend()
    plt.grid()

    # Display the plots

    plt.show()

main()