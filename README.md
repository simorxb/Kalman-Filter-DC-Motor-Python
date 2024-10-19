# Kalman Filter - DC Motor Angular Velocity Estimation in Python

## Summary
This project demonstrates how to use a Kalman filter to estimate the angular velocity of a DC motor, using Python for implementation. The estimation is based on noisy position readings, making the Kalman filter an optimal choice for real-time velocity tracking, even in the presence of noise. Both the Kalman filter and the DC motor are simulated in Python, offering a practical approach to real-time system estimation in control systems.

## Project Overview
Kalman filters are widely used in control systems and signal processing to estimate unknown variables in noisy environments. This project focuses on using a Kalman filter to estimate the angular velocity of a DC motor, using only a noisy angular position measurement.

### Kalman Filter - Theory
A Kalman filter is an optimal estimator that works by predicting the next state of a dynamic system based on its model and updating that prediction with real-time measurements. For linear dynamic systems with Gaussian noise, the Kalman filter provides the best estimate in the least-mean-square sense. The filter updates its estimate using the system model and noisy measurements, offering accurate state estimation even with system uncertainties.

The Kalman filter model follows the equations:
- State prediction: $$\hat x_{k|k-1} = F x_{k-1|k-1} + B u_k$$
- Covariance prediction: 
  $$\hat P_{k|k-1} = F P_{k-1|k-1} F^T + Q$$
- Innovation: 
  $$\tilde y_k = z_k - H \hat{x}_{k|k-1}$$
- Kalman gain: 
  $$K_k = \hat{P}_{k|k-1} H^T S^{-1}_k$$
- Updated state: 
  $$x_{k|k} = \hat{x}_{k|k-1} + K_k \tilde{y}_k$$

### DC Motor Model
This project models a DC motor's electrical and mechanical characteristics, including the armature circuit, back Electro-Magnetic Force (EMF), and rotor dynamics. The motor's behavior is captured using differential equations, where the current and angular velocity are affected by the applied voltage and motor parameters.

The electrical equation:
$$L \frac{di}{dt} + R i = V - k_e \omega$$

The mechanical equation:
$$J \dot{\omega} + b \omega = \tau, \quad \tau = k_t i$$

Key parameters derived from a real DC motor's datasheet (Moog C23-L33-W10) include:
- Torque sensitivity ($k_t$): 0.0187 Nm/A
- Back EMF constant ($k_e$): 0.0191 V/(rad/s)
- Terminal resistance ($R$): 0.6 Ohm
- Terminal inductance ($L$): 0.035 H
- Damping factor ($b$): 0.0000095 Nm/(rad/s)
- Rotor inertia ($J$): 0.000125 kg m²

### Python Implementation
The DC motor and Kalman filter are implemented in Python as two separate classes:
1. **DCMotor**: This class models the motor's dynamics, handling parameters such as velocity, position, and current, with methods to simulate its evolution over time.
2. **KalmanFilter**: This class handles the Kalman filter algorithm, maintaining state estimates and error covariance, and executing prediction and correction steps.

The simulation runs in a loop, with the DC motor's differential equations updated at a 1 ms interval. The Kalman filter, simulating a real-world embedded system, is updated every 10 ms to reflect reasonable sample time constraints.

### Simulation Results
The project includes plots comparing the estimated angular velocity ($\omega_{est}$) against the true angular velocity ($\omega$), demonstrating the filter's effectiveness in noisy conditions. The filter performs well even with significant noise, producing accurate velocity estimations despite extreme noise levels in the position measurements.

## Author
This project is developed by Simone Bertoni. Learn more about my work on my personal website - [Simone Bertoni - Control Lab](https://simonebertonilab.com/).

## Contact
For further communication, connect with me on [LinkedIn](https://www.linkedin.com/in/simone-bertoni-control-eng/).
