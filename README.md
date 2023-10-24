# Project-1
I have defined an optimal control problem for a simple and nontrivial dynamical system. In this example, I consider a classic problem in control theory: the inverted pendulum.

Dynamical System:
The inverted pendulum is a well-known dynamical system in which a rigid pole (the pendulum) is attached to a moving cart. The goal is to balance the pole upright on the cart by applying horizontal forces to the cart. The system can be described by the following state variables and dynamics:

State Variables:
Position of the cart along a horizontal track: ‘x’ (in meters).
Velocity of the cart: ‘x_dot’ (in meters per second).
Angle of the pendulum with respect to the vertical: ‘theta’ (in radians).
Angular velocity of the pendulum: ‘theta_dot’ (in radians per second).
Action Variable:
Horizontal force applied to the cart: ‘F’ (in Newtons).

Objective Function:
The objective of the control problem is to design a control policy that stabilizes the inverted pendulum at an upright position while minimizing the energy or effort required to do so. This can be formulated as a cost or performance index, such as:

Minimize:
J = ∫[0, T] (k1 * (theta^2 + theta_dot^2) + k2 * F^2) dt

Where:

‘T’ is the time horizon.
‘k1’ and ‘k2’ are positive weighting factors to balance the trade-off between stabilizing the pendulum and minimizing control effort.

Constraints:

Dynamics:
The dynamics of the system are described by ordinary differential equations (ODEs):
‘x_dot = v’
‘v_dot = (F + m * sin(theta) * (l * theta_dot^2 - g * cos(theta))) / (M + m * (1 - cos(theta)^2))’
‘theta_dot = omega’
‘omega_dot = ((M + m) * g * sin(theta) - m * l * omega^2 * sin(theta) * cos(theta) - (F + m * sin(theta) * (l * omega^2 - g * cos(theta))) * cos(theta)) / (l * (M + m * (1 - cos(theta)^2)))’


Here, ‘m’ is the mass of the pendulum, ‘M’ is the mass of the cart, ‘l’ is the length of the pendulum, ‘g’ is the gravitational acceleration, and ‘F’ is the control input.
Control Constraints:
The control input ‘F’ must be within certain bounds, e.g., ‘F_min <= F <= F_max’.
Initial and Final Conditions:
Initial state conditions are given, e.g., ‘x(0) = x0’, ‘v(0) = v0’, ‘theta(0) = theta0’, ‘omega(0) = omega0’.
Final state conditions may also be imposed.

Assumptions:

The system is controllable, which means that it is possible to apply control inputs to drive the system from any initial state to any desired final state.
The ODEs accurately describe the dynamics of the inverted pendulum.
No external disturbances or uncertainties are considered in this basic model.

This optimal control problem aims to find a control policy that stabilizes the inverted pendulum while minimizing energy consumption, making it nontrivial and solvable through various control optimization techniques like LQR, MPC, or reinforcement learning.

