# 4. State-Space Linearization

The non-linear equations derived in the previous section accurately model the real system. However, for classical control methods like the Linear Quadratic Regulator (LQR), we need a linear state-space model in the form:
$$\dot{\mathbf{x}} = A \mathbf{x} + B u$$

## 4.1 Small Angle Approximation
We linearize the system around its unstable equilibrium point, the upright position. At this point:
*   $\theta = 0$
*   $\dot{\theta} = 0$
*   $\ddot{\theta} = 0$
*   $\dot{x} = 0$
*   $\ddot{x} = 0$

For small deviations around $\theta = 0$, we use the Taylor series approximations (Small Angle Approximation):
*   $\cos(\theta) \approx 1$
*   $\sin(\theta) \approx \theta$
*   $\dot{\theta}^2 \approx 0$ (because the square of a small number is vanishingly small)
*   $\theta \dot{\theta} \approx 0$

## 4.2 Linearizing the Equations of Motion
Let's apply these approximations to our two non-linear equations.

**1. Translational Equation:**
$$(M + m)\ddot{x} + ml\ddot{\theta}\cos(\theta) - ml\dot{\theta}^2\sin(\theta) = F - b_c\dot{x}$$
Applying approximations ($\cos(\theta) \rightarrow 1$, $\dot{\theta}^2 \rightarrow 0$):
$$(M + m)\ddot{x} + ml\ddot{\theta}(1) - ml(0)(\theta) = F - b_c\dot{x}$$
$$(M + m)\ddot{x} + ml\ddot{\theta} = F - b_c\dot{x}$$
*(Equation A)*

**2. Rotational Equation:**
$$ml\ddot{x}\cos(\theta) + \frac{4}{3}ml^2\ddot{\theta} - mgl\sin(\theta) = -b_p\dot{\theta}$$
Applying approximations ($\cos(\theta) \rightarrow 1$, $\sin(\theta) \rightarrow \theta$):
$$ml\ddot{x}(1) + \frac{4}{3}ml^2\ddot{\theta} - mgl(\theta) = -b_p\dot{\theta}$$
$$ml\ddot{x} + \frac{4}{3}ml^2\ddot{\theta} - mgl\theta = -b_p\dot{\theta}$$
*(Equation B)*

## 4.3 Solving for Accelerations ($\ddot{x}$ and $\ddot{\theta}$)

To build a state space model, we need explicit equations for $\ddot{x}$ and $\ddot{\theta}$. We must solve the system of linear equations (Equation A and Equation B) algebraically.

From (Equation A), solve for $\ddot{x}$:
$$(M + m)\ddot{x} = F - b_c\dot{x} - ml\ddot{\theta}$$
$$\ddot{x} = \frac{1}{M + m} (F - b_c\dot{x} - ml\ddot{\theta})$$

Substitute this $\ddot{x}$ into (Equation B):
$$ml \left[ \frac{1}{M + m} (F - b_c\dot{x} - ml\ddot{\theta}) \right] + \frac{4}{3}ml^2\ddot{\theta} - mgl\theta = -b_p\dot{\theta}$$

Multiply everything by $(M+m)$ to remove the denominator:
$$ml(F - b_c\dot{x} - ml\ddot{\theta}) + \frac{4}{3}ml^2(M+m)\ddot{\theta} - mgl(M+m)\theta = -b_p(M+m)\dot{\theta}$$

Expand terms:
$$mlF - mlb_c\dot{x} - m^2l^2\ddot{\theta} + \frac{4}{3}ml^2(M+m)\ddot{\theta} - mgl(M+m)\theta = -b_p(M+m)\dot{\theta}$$

Group the $\ddot{\theta}$ terms together:
$$\left( \frac{4}{3}ml^2(M+m) - m^2l^2 \right) \ddot{\theta} = mgl(M+m)\theta - b_p(M+m)\dot{\theta} + mlb_c\dot{x} - mlF$$

Let the constant denominator term be denoted as $D = \frac{4}{3}ml^2(M+m) - m^2l^2$.
$$D = ml^2 \left( \frac{4}{3}(M+m) - m \right) = ml^2 \left( \frac{4}{3}M + \frac{1}{3}m \right)$$

Solving for $\ddot{\theta}$:
$$\ddot{\theta} = \frac{mgl(M+m)}{D} \theta - \frac{b_p(M+m)}{D} \dot{\theta} + \frac{mlb_c}{D} \dot{x} - \frac{ml}{D} F$$

Now we substitute $\ddot{\theta}$ back into Equation A to find $\ddot{x}$ independently of $\ddot{\theta}$. For brevity, one arrives at:
$$\ddot{x} = \frac{-m^2g l^2}{D} \theta + \frac{b_p m l}{D} \dot{\theta} - \frac{\frac{4}{3} m l^2 b_c}{D} \dot{x} + \frac{\frac{4}{3} m l^2}{D} F$$

## 4.4 State-Space Representation

Let our state vector be $\mathbf{x} = \begin{bmatrix} x \\ \dot{x} \\ \theta \\ \dot{\theta} \end{bmatrix}$ and our input $u = F$.
The state equation is $\dot{\mathbf{x}} = A \mathbf{x} + B u$.

$$
\begin{bmatrix}
\dot{x} \\
\ddot{x} \\
\dot{\theta} \\
\ddot{\theta}
\end{bmatrix}
=
\begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & \frac{-4 m l^2 b_c}{3D} & \frac{-m^2 g l^2}{D} & \frac{m l b_p}{D} \\
0 & 0 & 0 & 1 \\
0 & \frac{m l b_c}{D} & \frac{m g l (M+m)}{D} & \frac{-b_p (M+m)}{D}
\end{bmatrix}
\begin{bmatrix}
x \\
\dot{x} \\
\theta \\
\dot{\theta}
\end{bmatrix}
+
\begin{bmatrix}
0 \\
\frac{4 m l^2}{3D} \\
0 \\
\frac{-m l}{D}
\end{bmatrix}
u
$$

Where $D = ml^2 \left( \frac{4}{3}M + \frac{1}{3}m \right)$.
This constitutes the exact continuous-time linear model required to design optimal state-feedback controllers like LQR.
