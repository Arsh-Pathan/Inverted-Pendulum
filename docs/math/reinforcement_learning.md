# 5. Reinforcement Learning Formulation

In addition to classical control methods (like LQR and PID), this project utilizes Reinforcement Learning (RL) to stabilize and swing up the inverted pendulum. Specifically, we employ Proximal Policy Optimization (PPO) and Soft Actor-Critic (SAC).

This document outlines the mathematical formulation of the RL environment and the core equations for these algorithms.

## 5.1 Markov Decision Process (MDP)

The inverted pendulum problem is formulated as a continuous-time Markov Decision Process (MDP) defined by the tuple $(\mathcal{S}, \mathcal{A}, \mathcal{P}, \mathcal{R}, \gamma)$.

### State Space ($\mathcal{S}$)
The state vector $s_t \in \mathcal{S}$ is continuous and contains the kinematic variables of the cart and pendulum:
$$s_t = \begin{bmatrix} x \\ \dot{x} \\ \theta \\ \dot{\theta} \end{bmatrix}$$
Where $x$ is cart position, $\dot{x}$ is cart velocity, $\theta$ is pendulum angle, and $\dot{\theta}$ is angular velocity. (Note: In some implementations, $\theta$ is represented as $[\cos(\theta), \sin(\theta)]$ to avoid discontinuity at $\pi \equiv -\pi$).

### Action Space ($\mathcal{A}$)
The action $a_t \in \mathcal{A}$ is continuous and represents the force or voltage applied to the cart motor:
$$a_t = F \in [F_{min}, F_{max}]$$

### Transition Dynamics ($\mathcal{P}$)
The transition probability density $P(s_{t+1} | s_t, a_t)$ is determined by the non-linear physics of the system (derived in previous sections) integrated over a discrete time step $\Delta t$. Using numerical integration (e.g., Euler or Runge-Kutta 4):
$$s_{t+1} = s_t + \int_{t}^{t+\Delta t} f_{physics}(s, a) dt$$

### Reward Function ($\mathcal{R}$)
The reward function $r_t = R(s_t, a_t)$ is designed to penalize deviation from the upright equilibrium ($\theta=0, x=0$) and penalize excessive control effort. A typical quadratic reward formulation is:
$$r_t = -(Q_{\theta} \theta^2 + Q_{x} x^2 + Q_{\dot{\theta}} \dot{\theta}^2 + Q_{\dot{x}} \dot{x}^2 + R_{u} a_t^2)$$
For swing-up tasks, additional non-linear terms like $\cos(\theta)$ may be used:
$$r_t = \cos(\theta) - 1 - \alpha a_t^2$$

### Discount Factor ($\gamma$)
The discount factor $\gamma \in (0, 1)$ balances immediate and future rewards in the return $R_t$:
$$R_t = \sum_{k=0}^{\infty} \gamma^k r_{t+k}$$

---

## 5.2 Proximal Policy Optimization (PPO) Math

PPO is an on-policy actor-critic algorithm that optimizes the policy by taking steps that are bounded to avoid destructively large updates.

### 1. Advantage Function
PPO uses the Advantage function $A^{\pi}(s, a)$, which measures how much better an action $a$ is compared to the average action in state $s$:
$$A_t = Q_t(s_t, a_t) - V_t(s_t)$$
In practice, this is estimated using Generalized Advantage Estimation (GAE).

### 2. Probability Ratio
Let $\pi_{\theta}(a_t|s_t)$ be the current policy network and $\pi_{\theta_{old}}(a_t|s_t)$ be the policy network before the update. The probability ratio is:
$$r_t(\theta) = \frac{\pi_{\theta}(a_t|s_t)}{\pi_{\theta_{old}}(a_t|s_t)}$$

### 3. Clipped Surrogate Objective
To prevent large policy updates, PPO clips this ratio. The objective function $L^{CLIP}(\theta)$ that PPO maximizes is:
$$L^{CLIP}(\theta) = \mathbb{E}_t \left[ \min \left( r_t(\theta) A_t, \text{clip}(r_t(\theta), 1-\epsilon, 1+\epsilon) A_t \right) \right]$$
Where $\epsilon$ is a hyperparameter (e.g., 0.2).
*   If $A_t > 0$ (action was good), the ratio is capped at $1+\epsilon$ to prevent over-updating.
*   If $A_t < 0$ (action was bad), the ratio is floored at $1-\epsilon$.

### 4. Value Function Loss
The critic network (value function $V_{\phi}(s_t)$) is updated by minimizing the Mean Squared Error against the targeted returns $R_t$:
$$L^{VF}(\phi) = \mathbb{E}_t \left[ (V_{\phi}(s_t) - R_t)^2 \right]$$

---

## 5.3 Soft Actor-Critic (SAC) Math

SAC is an off-policy algorithm based on the maximum entropy framework. It encourages exploration by maximizing both the expected return and the entropy of the policy.

### 1. Maximum Entropy Objective
The optimal policy $\pi^*$ in SAC maximizes the standard return plus an entropy term $\mathcal{H}$:
$$\pi^* = \arg\max_{\pi} \mathbb{E}_{\tau \sim \pi} \left[ \sum_{t=0}^{\infty} \gamma^t \left( r(s_t, a_t) + \alpha \mathcal{H}(\pi(\cdot | s_t)) \right) \right]$$
Where $\alpha$ is the temperature parameter that determines the relative importance of the entropy term.
The entropy is $\mathcal{H}(\pi(\cdot | s_t)) = -\mathbb{E}_{a \sim \pi} [\log \pi(a|s_t)]$.

### 2. Soft Q-Function Training
The Soft Q-function $Q_{\theta}(s, a)$ is trained to minimize the Soft Bellman Residual:
$$J_Q(\theta) = \mathbb{E}_{(s_t, a_t) \sim \mathcal{D}} \left[ \frac{1}{2} \left( Q_{\theta}(s_t, a_t) - y_t \right)^2 \right]$$
Where the target $y_t$ incorporates the entropy of the next state:
$$y_t = r(s_t, a_t) + \gamma \mathbb{E}_{a_{t+1} \sim \pi_{\phi}} \left[ Q_{\bar{\theta}}(s_{t+1}, a_{t+1}) - \alpha \log \pi_{\phi}(a_{t+1} | s_{t+1}) \right]$$
Here, $Q_{\bar{\theta}}$ is the target Q-network.

### 3. Policy (Actor) Training
The policy network $\pi_{\phi}$ is trained to maximize the Q-value while maintaining high entropy. By applying the reparameterization trick ($a_t = f_{\phi}(\epsilon_t ; s_t)$), the objective to minimize is:
$$J_{\pi}(\phi) = \mathbb{E}_{s_t \sim \mathcal{D}, \epsilon_t \sim \mathcal{N}} \left[ \alpha \log \pi_{\phi}(f_{\phi}(\epsilon_t; s_t) | s_t) - Q_{\theta}(s_t, f_{\phi}(\epsilon_t; s_t)) \right]$$

### 4. Temperature ($\alpha$) Auto-Tuning
In modern SAC, $\alpha$ is automatically adjusted to meet a target entropy $\bar{\mathcal{H}}$:
$$J(\alpha) = \mathbb{E}_{a_t \sim \pi} \left[ -\alpha \log \pi(a_t | s_t) - \alpha \bar{\mathcal{H}} \right]$$
