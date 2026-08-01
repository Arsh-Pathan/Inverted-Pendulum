# 2. System Energy

To use the Euler-Lagrange method, we need the total kinetic energy ($T$) and the total potential energy ($V$) of the system.

## 2.1 Kinetic Energy ($T$)

The total kinetic energy of the system is the sum of the kinetic energy of the cart and the kinetic energy of the pendulum.
$$T = T_{cart} + T_{pendulum}$$

### Kinetic Energy of the Cart
The cart only undergoes translational motion. Its kinetic energy is:
$$T_{cart} = \frac{1}{2} M v_{cart}^2 = \frac{1}{2} M \dot{x}^2$$

### Kinetic Energy of the Pendulum
The pendulum undergoes both translational motion (its center of mass moves) and rotational motion (it rotates around its center of mass).

1.  **Translational Kinetic Energy:**
    $$T_{p, trans} = \frac{1}{2} m v_p^2$$
    From our kinematics derivation ($v_p^2 = \dot{x}^2 + 2 l \dot{x} \dot{\theta} \cos(\theta) + l^2 \dot{\theta}^2$), this is:
    $$T_{p, trans} = \frac{1}{2} m (\dot{x}^2 + 2 l \dot{x} \dot{\theta} \cos(\theta) + l^2 \dot{\theta}^2)$$

2.  **Rotational Kinetic Energy:**
    $$T_{p, rot} = \frac{1}{2} I \dot{\theta}^2$$
    Where $I$ is the moment of inertia of the pendulum about its center of mass. For a uniform rod of mass $m$ and total length $L = 2l$, the standard result about the centre is
    $$I = \frac{1}{12}mL^2 = \frac{1}{12}m(2l)^2 = \frac{1}{3}ml^2$$
    Combined with the parallel-axis shift to the pivot this gives $I + ml^2 = \frac{4}{3}ml^2$, the moment of inertia about the pivot used throughout.

Total kinetic energy of the pendulum:
$$T_{pendulum} = \frac{1}{2} m (\dot{x}^2 + 2 l \dot{x} \dot{\theta} \cos(\theta) + l^2 \dot{\theta}^2) + \frac{1}{2} I \dot{\theta}^2$$

### Total Kinetic Energy
Summing the cart and pendulum kinetic energies:
$$T = \frac{1}{2} M \dot{x}^2 + \frac{1}{2} m \dot{x}^2 + m l \dot{x} \dot{\theta} \cos(\theta) + \frac{1}{2} m l^2 \dot{\theta}^2 + \frac{1}{2} I \dot{\theta}^2$$
$$T = \frac{1}{2} (M + m) \dot{x}^2 + m l \dot{x} \dot{\theta} \cos(\theta) + \frac{1}{2} (m l^2 + I) \dot{\theta}^2$$

Since $I = \frac{1}{3} m l^2$, the term $(m l^2 + I)$ evaluates to $m l^2 + \frac{1}{3} m l^2 = \frac{4}{3} m l^2$. Therefore, the total kinetic energy is:
$$T = \frac{1}{2} (M + m) \dot{x}^2 + m l \dot{x} \dot{\theta} \cos(\theta) + \frac{2}{3} m l^2 \dot{\theta}^2$$

## 2.2 Potential Energy ($V$)

The potential energy of the system depends entirely on gravity (assuming no springs). Since the cart moves only horizontally, its height does not change, meaning its potential energy is constant. We can define the reference height ($V=0$) at the track level ($y=0$).

The potential energy of the pendulum depends on the vertical height of its center of mass ($y_p$).
From the kinematics section, $y_p = l \cos(\theta)$.

Therefore, the potential energy is:
$$V = m g y_p = m g l \cos(\theta)$$

## 2.3 The Lagrangian ($L$)

The Lagrangian $L$ is defined as the difference between kinetic and potential energy:
$$L = T - V$$

Substituting our expressions for $T$ and $V$:
$$L = \left[ \frac{1}{2} (M + m) \dot{x}^2 + m l \dot{x} \dot{\theta} \cos(\theta) + \frac{2}{3} m l^2 \dot{\theta}^2 \right] - \left[ m g l \cos(\theta) \right]$$

$$L = \frac{1}{2} (M + m) \dot{x}^2 + m l \dot{x} \dot{\theta} \cos(\theta) + \frac{2}{3} m l^2 \dot{\theta}^2 - m g l \cos(\theta)$$

This Lagrangian fully describes the dynamics of the conservative parts of our system. Next, we will apply the Euler-Lagrange equations to find the equations of motion.
