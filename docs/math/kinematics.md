# 1. System Kinematics

The first step in deriving the equations of motion for the inverted pendulum is to establish the coordinate system and determine the position and velocity of all masses in the system.

## 1.1 System Parameters and Coordinate Frame

We define our reference frame with the origin $(0,0)$ at the center of the cart's track when the cart is at rest. The horizontal axis is $X$ (positive to the right) and the vertical axis is $Y$ (positive upwards).

*   **$M$**: Mass of the cart (kg)
*   **$m$**: Mass of the pendulum (kg)
*   **$l$**: Distance from the cart's pivot point to the center of mass (CoM) of the pendulum (m)
*   **$x(t)$**: The horizontal position of the cart at time $t$ (m)
*   **$\theta(t)$**: The angle of the pendulum with respect to the vertical upward axis (rad). Thus, $\theta = 0$ corresponds to the pendulum pointing straight up, and $\theta = \pi$ corresponds to the pendulum hanging straight down.

## 1.2 Cart Kinematics

The cart is constrained to move only along the horizontal track. Therefore, its position coordinates are simply:
$$x_{cart} = x$$
$$y_{cart} = 0$$

Taking the time derivative, the velocity of the cart is:
$$v_{cart_x} = \dot{x}$$
$$v_{cart_y} = 0$$

## 1.3 Pendulum Kinematics

The pendulum's center of mass is attached to the cart at a pivot point. The position of this pivot point is exactly $(x, 0)$. 
Using trigonometry, the horizontal distance of the pendulum's CoM from the pivot is $l \sin(\theta)$, and the vertical height of the pendulum's CoM from the pivot is $l \cos(\theta)$.

Thus, the global coordinates of the pendulum's center of mass, $(x_p, y_p)$, are:
$$x_p = x + l \sin(\theta)$$
$$y_p = l \cos(\theta)$$

To find the velocity of the pendulum's center of mass, we differentiate the position coordinates with respect to time $t$. We must use the chain rule since $\theta$ is a function of time $\theta(t)$.

**Horizontal velocity of the pendulum ($v_{px}$):**
$$v_{px} = \frac{d}{dt} x_p = \frac{d}{dt} (x + l \sin(\theta))$$
$$v_{px} = \dot{x} + l \cos(\theta) \dot{\theta}$$

**Vertical velocity of the pendulum ($v_{py}$):**
$$v_{py} = \frac{d}{dt} y_p = \frac{d}{dt} (l \cos(\theta))$$
$$v_{py} = -l \sin(\theta) \dot{\theta}$$

The squared magnitude of the pendulum's velocity ($v_p^2 = v_{px}^2 + v_{py}^2$) will be essential for calculating kinetic energy in the next section:
$$v_p^2 = (\dot{x} + l \cos(\theta) \dot{\theta})^2 + (-l \sin(\theta) \dot{\theta})^2$$
$$v_p^2 = (\dot{x}^2 + 2 l \dot{x} \dot{\theta} \cos(\theta) + l^2 \dot{\theta}^2 \cos^2(\theta)) + (l^2 \dot{\theta}^2 \sin^2(\theta))$$

Factoring out $l^2 \dot{\theta}^2$ from the last two terms:
$$v_p^2 = \dot{x}^2 + 2 l \dot{x} \dot{\theta} \cos(\theta) + l^2 \dot{\theta}^2 (\cos^2(\theta) + \sin^2(\theta))$$

Using the Pythagorean identity $\cos^2(\theta) + \sin^2(\theta) = 1$:
$$v_p^2 = \dot{x}^2 + 2 l \dot{x} \dot{\theta} \cos(\theta) + l^2 \dot{\theta}^2$$

With the kinematics fully described, we can now formulate the energy equations for the system.
