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

Given the Lagrangian of the system, $L(x,x',\theta,\theta')$, which is given by the difference of the kinnetic and potential energy of the system:

$$L(x, x', \theta, \theta '):= \frac{1}{2}(M+m)(x')^2 + mlx'\theta ' \cos{\theta} + \frac{1}{2}ml^2 |\theta ' |^2 - mgl \cos{\theta} $$
Whose partial derivatives are given by:

$$\frac{\partial L}{\partial \theta} = glm \sin{\theta} - lm\theta ' x'\sin{\theta}$$

$$\frac{d}{dt}\left(\frac{\partial L}{\partial \theta '} \right) = -lm x' \theta '\sin{\theta} +lmx'' \cos{\theta}  + l^2m \theta ''$$

$$\frac{\partial L}{\partial x}=0$$

$$\frac{d}{dt}\left(\frac{\partial L}{\partial x '} \right) = -lm |\theta '|^2\sin{\theta}  + (m+M)x'' +lm \theta\cos{\theta} ''$$

Applying Lagrange's theorem we get:

$$\begin{cases}
\theta '' = \frac{1}{l} \left( g\sin{\theta}- x'' \cos{\theta} \right)\\
x'' =  \frac{ml}{(M+m)}\left(- \theta '' \cos{\theta} +|\theta ' |^2 \sin{\theta}\right)
\end{cases}$$

Now, having the system described by a second order differential equation, we will now 




