# 6. Controller Implementations and Simulation Math

Beyond the foundational physical derivations, the project uses specific mathematical implementations for its controllers, filters, and software physics simulation. This document details the exact mathematics used in the `algorithm/math/controllers` and `algorithm/math/envs` modules.

## 6.1 LQR Balancer (`lqr_balancer.py`)

The Linear Quadratic Regulator (LQR) uses the linearized state-space matrices derived previously to find an optimal state-feedback gain matrix $K$.

### Control Law
The implemented control law calculates the required motor power from the signed tilt and angular velocity:
$$u = k_\theta \cdot \theta + k_\omega \cdot \dot{\theta}_{filtered}$$

**Sign convention (critical).** $\theta$ is `PendulumState.theta_from_upright`, defined as
$$\theta = \mathrm{wrap}(\texttt{angle\_dev} - 180^\circ) \in [-180^\circ, +180^\circ]$$
so $\theta = 0$ is upright and $\dot{\theta}$ (the reported angular velocity) is *exactly* the
time derivative of $\theta$. Both gains are **positive**: to arrest a fall toward $+\theta$ the
cart must accelerate toward $+\theta$ as well ("catch-the-fall"), because the pendulum is
driven only through the pivot reaction term $-m l \ddot{x}\cos\theta$.

> [!WARNING]
> The legacy property `error_from_upright` equals $180^\circ - \texttt{angle\_dev} = -\theta$.
> Mixing it with the raw `velocity` field combines two opposite sign conventions, which turns
> the $k_\omega$ term into *positive* velocity feedback and drives the cart away from the fall.
> Use `theta_from_upright` in all control laws.

### Cart-Position Observability Limit
The physical rig instruments the **pendulum pivot only** (a single AS5600); there is no cart
position encoder. The law above therefore feeds back just $[\theta, \dot{\theta}]$, which can
hold the pole vertical but cannot regulate where the cart sits. Any residual tilt bias makes
the cart drift steadily in one direction until it reaches the end of the rail. Empirically, on
the simulated 0.4 m track, angle-only feedback keeps the pole within $8^\circ$ yet hits the
end-stop in $\approx 0.9\ \text{s}$, whereas adding cart terms
$$u = k_\theta\theta + k_\omega\dot\theta + k_x x + k_v \dot{x}$$
holds both the pole and the cart indefinitely. Full four-state stabilization of this platform
requires adding a cart position sensor (belt encoder or motor-shaft odometry).

### Exponential Moving Average (EMA) Velocity Filter
To reduce noise from the sensor readings, the LQR controller applies an Exponential Moving Average filter (a first-order discrete low-pass filter) to the angular velocity before using it in the control law:
$$\dot{\theta}_{filtered, t} = \alpha \cdot \dot{\theta}_{raw, t} + (1 - \alpha) \cdot \dot{\theta}_{filtered, t-1}$$
Where $\alpha \in (0, 1]$ is the smoothing factor (e.g., $0.08$).

---

## 6.2 Energy Swing-Up Controller (`swing_up.py`)

When the pendulum is hanging down, linear controllers like LQR fail. Instead, we use a non-linear energy-pumping law based on the Åström-Furuta Lyapunov strategy to add kinetic energy until the pendulum enters the top "capture basin".

### Energy Pumping Law
Let $\varphi$ be measured from the **hanging** equilibrium ($\varphi = 0$ hanging, $\varphi = \pm\pi$
upright), and let $J = \frac{4}{3}ml^2$ be the moment of inertia about the pivot. The energy
relative to hanging rest and its upright target are
$$E = \tfrac{1}{2}J\dot{\varphi}^2 + mgl\left(1 - \cos\varphi\right), \qquad E_{top} = 2mgl$$

With the energy error $\tilde{E} = E - E_{top}$ and Lyapunov candidate $V = \tfrac{1}{2}\tilde{E}^2$,
the implemented law is
$$u = -k_E\,\tilde{E}\,\dot{\varphi}\cos\varphi$$
which yields $\dot{V} = -k_E\left(\tilde{E}\dot{\varphi}\cos\varphi\right)^2 \le 0$, so $E \to E_{top}$.

The $\tilde{E}$ factor is what makes this converge: it **pumps** energy while $E < E_{top}$,
**brakes** once $E > E_{top}$, and goes to zero at the target.

> [!WARNING]
> A bare $u = \mathrm{sign}(\dot{\varphi}\cos\varphi)\,P_{max}$ law (no $\tilde{E}$ term) has no
> notion of "enough energy" and pumps without bound, so the pendulum spins continuously instead
> of arriving at the top with $\dot{\varphi} \approx 0$.

Because the law is identically zero at exact rest ($\dot\varphi = 0, \varphi = 0$), a small
fixed kick is applied to break away from the hanging equilibrium.

---

## 6.3 Numerical Simulation Physics (`inverted_pendulum_env.py`)

For the RL environment and offline testing, the continuous non-linear differential equations are integrated over discrete time steps $\Delta t$ (e.g., 10ms) using the Euler integration method.

### State Update Equations
The simulator state is $[x, \dot{x}, \theta, \dot{\theta}]$, where $\theta$ is the deviation from
upright. The action $a \in [-1, 1]$ commands a cart acceleration
$a_{cart} = a\,\ddot{x}_{max}$, and the realised cart acceleration after rail drag is
$$\ddot{x} = a_{cart} - b_c \dot{x}$$

The pendulum is actuated **only** through the pivot, so its rotational equation about the pivot is
$$J\ddot{\theta} = mgl\sin\theta - ml\,\ddot{x}\cos\theta - b\,\dot{\theta},
\qquad J = \tfrac{4}{3}ml^2$$

Two corrections versus the earlier formulation:
1. **Inertia.** $J = \frac{4}{3}ml^2$ for a uniform rod whose COM is at $l$ (consistent with
   `energy.md`), not the point-mass $ml^2$ — a 33% error in the inertia.
2. **Actuation sign.** The cart enters as $-ml\ddot{x}\cos\theta$. Near upright
   ($\cos\theta \approx 1$) a *positive* cart acceleration *reduces* a positive tilt. Modelling
   the input as an additive $+\tau_{motor}$ inverts this coupling, so a controller tuned against
   that model pushes the cart the wrong way on real hardware.

### Semi-Implicit Euler Integration
Velocities are advanced first, then positions using the *updated* velocity (semi-implicit /
symplectic Euler), which is markedly more stable than explicit Euler for oscillatory systems:
$$\dot{x}_{t+1} = \dot{x}_t + \ddot{x}_t\Delta t, \qquad x_{t+1} = x_t + \dot{x}_{t+1}\Delta t$$
$$\dot{\theta}_{t+1} = \dot{\theta}_t + \ddot{\theta}_t \Delta t, \qquad \theta_{t+1} = \theta_t + \dot{\theta}_{t+1}\Delta t$$

Reaching a rail end-stop is treated as a fully inelastic collision ($\dot{x} \to 0$).

---

## 6.4 RL Environment Reward Function

The reward function $R_t$ calculated at every step in the `InvertedPendulumEnv` combines discrete conditional bonuses/penalties with continuous quadratic costs.

### 1. Upright Holding Bonus
If the pendulum is balanced within $\pm 1.0^{\circ}$ (the "upright buffer"), a time-scaling bonus is applied to encourage stability:
$$Bonus_t = 10.0 + \min(40.0, 0.1 \times \text{frames\_held})$$

### 2. High-Speed Spin Penalty
To prevent the agent from cheating the system by spinning the pendulum continuously, a severe penalty is applied if $|\dot{\theta}| > 360^{\circ}/s$:
$$Penalty_t = \begin{cases} 
      50.0 + 0.15 \times (|\dot{\theta}| - 360) & \text{if } |\dot{\theta}| > 360 \\
      0 & \text{otherwise}
   \end{cases}
$$

### 3. Dense Shaping Term
The $\pm 1^\circ$ holding bonus is far too narrow to be discovered by exploration on its own — a
randomly initialised policy essentially never lands inside it, so the bonus provides no gradient.
A dense term supplies a usable slope from anywhere in the state space:
$$Shaping_t = \cos\theta \in [-1, +1]$$
which is $+1$ upright, $0$ horizontal, and $-1$ hanging.

### 4. Continuous Cost Function
A quadratic cost penalises tilt, spin, control effort, and cart excursion toward the rail ends:
$$Cost_t = 2.0\,\theta^2 + 0.05\,\dot{\theta}^2 + 0.01\,u_{norm}^2 + 5.0\left(\max\left(0, |x| - 0.8\,x_{lim}\right)\right)^2$$
where $u_{norm} \in [-1, 1]$ is the normalised action, $\theta$ and $\dot\theta$ are in **radians**
and rad/s, and $x_{lim}$ is the rail half-length.

**Total Reward:**
$$R_t = Shaping_t + Bonus_t - Penalty_t - Cost_t$$

> [!IMPORTANT]
> The reward is evaluated on the state the action **produced** (post-step), not the state it was
> chosen in. Scoring the pre-step state credits every reward to the wrong transition and makes the
> value function learn a shifted objective.
