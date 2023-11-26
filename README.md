Matlab code for simulating and controling the inverted pendulum on a cart problem.

# Cart-Pole Control Problem

An inverted pendulum is a classic problem in control theory and robotics. It involves a
rigid pendulum that is mounted on a cart, where the objective is to keep the pendulum
balanced in the upright position by moving the cart back and forth. This
problem is challenging because it requires precise control of the cart’s position and velocity
to maintain the pendulum’s equilibrium.

<p align="center">
  <img src="cartpole_draw.png" width="250" title="Cart with inverted pendulum system">
</p>


The problem can be solved using various control strategies. The classical control approach
involves designing a controller based on mathematical models of the system, while more modern
control techniques use advanced control algorithms to achieve better performance. Optimal control methods aim to minimize a certain cost function while maintaining the pendulum’s
balance.

# Mathematical Statement

Given the Lagrangian of the system, $L(x,x',\theta,\theta')$, 
