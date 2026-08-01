# 3. Euler-Lagrange Equations of Motion

We use the Euler-Lagrange equation to derive the equations of motion for the system's two generalized coordinates: cart position $x$, and pendulum angle $\theta$.

The Euler-Lagrange equation is:
$$\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{q}_i} \right) - \frac{\partial L}{\partial q_i} = Q_i$$
Where $q_i \in \{x, \theta\}$ and $Q_i$ represents the generalized non-conservative forces acting on that coordinate.

The Lagrangian derived in the previous section is:
$$L = \frac{1}{2} (M + m) \dot{x}^2 + m l \dot{x} \dot{\theta} \cos(\theta) + \frac{2}{3} m l^2 \dot{\theta}^2 - m g l \cos(\theta)$$

## 3.1 Equation for the Cart Position ($x$)

The generalized non-conservative force for the cart is the external motor force $F$ minus the viscous friction of the track $b_c \dot{x}$.
$$Q_x = F - b_c \dot{x}$$

**Step 1: Calculate $\frac{\partial L}{\partial \dot{x}}$**
We differentiate $L$ with respect to $\dot{x}$ (treating all other variables, including $x$, $\theta$, and $\dot{\theta}$, as constants):
$$\frac{\partial L}{\partial \dot{x}} = \frac{\partial}{\partial \dot{x}} \left( \frac{1}{2} (M + m) \dot{x}^2 + m l \dot{x} \dot{\theta} \cos(\theta) \right)$$
$$\frac{\partial L}{\partial \dot{x}} = (M + m) \dot{x} + m l \dot{\theta} \cos(\theta)$$
*(Physically, this represents the total linear momentum of the system in the $x$ direction).*

**Step 2: Calculate the time derivative $\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{x}} \right)$**
We take the full time derivative. We must use the product rule on the term $m l \dot{\theta} \cos(\theta)$, because both $\dot{\theta}$ and $\cos(\theta)$ depend on time $t$.
$$\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{x}} \right) = \frac{d}{dt} [ (M + m) \dot{x} ] + \frac{d}{dt} [ m l \dot{\theta} \cos(\theta) ]$$
$$\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{x}} \right) = (M + m) \ddot{x} + m l \ddot{\theta} \cos(\theta) + m l \dot{\theta} \left( \frac{d}{dt} \cos(\theta) \right)$$
Using the chain rule, $\frac{d}{dt} \cos(\theta) = -\sin(\theta) \dot{\theta}$:
$$\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{x}} \right) = (M + m) \ddot{x} + m l \ddot{\theta} \cos(\theta) - m l \dot{\theta}^2 \sin(\theta)$$

**Step 3: Calculate $\frac{\partial L}{\partial x}$**
Since $x$ does not explicitly appear in the Lagrangian $L$, this term is zero:
$$\frac{\partial L}{\partial x} = 0$$

**Step 4: Assemble the equation**
$$ \left( (M + m) \ddot{x} + m l \ddot{\theta} \cos(\theta) - m l \dot{\theta}^2 \sin(\theta) \right) - (0) = F - b_c \dot{x} $$

**Final Translational Equation of Motion:**
$$(M + m)\ddot{x} + ml\ddot{\theta}\cos(\theta) - ml\dot{\theta}^2\sin(\theta) = F - b_c\dot{x}$$

## 3.2 Equation for the Pendulum Angle ($\theta$)

The generalized non-conservative force for the pendulum is the viscous friction at the pivot joint $b_p \dot{\theta}$. Since friction opposes motion, it acts in the negative direction.
$$Q_\theta = -b_p \dot{\theta}$$

**Step 1: Calculate $\frac{\partial L}{\partial \dot{\theta}}$**
We differentiate $L$ with respect to $\dot{\theta}$:
$$\frac{\partial L}{\partial \dot{\theta}} = \frac{\partial}{\partial \dot{\theta}} \left( m l \dot{x} \dot{\theta} \cos(\theta) + \frac{2}{3} m l^2 \dot{\theta}^2 \right)$$
$$\frac{\partial L}{\partial \dot{\theta}} = m l \dot{x} \cos(\theta) + \frac{4}{3} m l^2 \dot{\theta}$$
*(Physically, this represents the angular momentum of the system).*

**Step 2: Calculate the time derivative $\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{\theta}} \right)$**
Taking the full time derivative, using the product rule on $m l \dot{x} \cos(\theta)$:
$$\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{\theta}} \right) = m l \ddot{x} \cos(\theta) + m l \dot{x} (-\sin(\theta) \dot{\theta}) + \frac{4}{3} m l^2 \ddot{\theta}$$
$$\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{\theta}} \right) = m l \ddot{x} \cos(\theta) - m l \dot{x} \dot{\theta} \sin(\theta) + \frac{4}{3} m l^2 \ddot{\theta}$$

**Step 3: Calculate $\frac{\partial L}{\partial \theta}$**
We differentiate $L$ with respect to $\theta$. The variable $\theta$ appears inside $\cos(\theta)$ in two terms:
$$\frac{\partial L}{\partial \theta} = \frac{\partial}{\partial \theta} \left( m l \dot{x} \dot{\theta} \cos(\theta) - m g l \cos(\theta) \right)$$
$$\frac{\partial L}{\partial \theta} = -m l \dot{x} \dot{\theta} \sin(\theta) + m g l \sin(\theta)$$

**Step 4: Assemble the equation**
$$ \left( m l \ddot{x} \cos(\theta) - m l \dot{x} \dot{\theta} \sin(\theta) + \frac{4}{3} m l^2 \ddot{\theta} \right) - \left( -m l \dot{x} \dot{\theta} \sin(\theta) + m g l \sin(\theta) \right) = -b_p \dot{\theta} $$

Notice that the Coriolis term $-m l \dot{x} \dot{\theta} \sin(\theta)$ cancels out beautifully:
$$m l \ddot{x} \cos(\theta) - m l \dot{x} \dot{\theta} \sin(\theta) + \frac{4}{3} m l^2 \ddot{\theta} + m l \dot{x} \dot{\theta} \sin(\theta) - m g l \sin(\theta) = -b_p \dot{\theta}$$
$$m l \ddot{x} \cos(\theta) + \frac{4}{3} m l^2 \ddot{\theta} - m g l \sin(\theta) = -b_p \dot{\theta}$$

**Final Rotational Equation of Motion:**
$$ml\ddot{x}\cos(\theta) + \frac{4}{3}ml^2\ddot{\theta} - mgl\sin(\theta) = -b_p\dot{\theta}$$

## 3.3 Summary of Non-Linear System Equations
1. **Translational:** $(M + m)\ddot{x} + ml\ddot{\theta}\cos(\theta) - ml\dot{\theta}^2\sin(\theta) = F - b_c\dot{x}$
2. **Rotational:** $ml\ddot{x}\cos(\theta) + \frac{4}{3}ml^2\ddot{\theta} - mgl\sin(\theta) = -b_p\dot{\theta}$
