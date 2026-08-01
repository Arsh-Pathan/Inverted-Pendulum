# Math & Control Review — Findings and Fixes

Review of the control/RL mathematics against the physical rig (single AS5600 on the pendulum
pivot, TB6612FNG-driven belt cart on a ~0.4 m rail, ESP32 streaming angle at 100 Hz).

Every claim below was verified by running the code, not by inspection alone.

---

## 0. TL;DR — why it drove the wrong way

**Root cause: two contradictory sign conventions were mixed inside every control law.**

`PendulumState` exposed only `error_from_upright = 180 - angle_dev`. But the reported `velocity`
is `d(angle_dev)/dt`. So:

$$\texttt{error\_from\_upright} = -\theta, \qquad \texttt{velocity} = +\dot{\theta}$$

The position term and the velocity term had **opposite signs**. Consequences:

1. The proportional term drove the cart *away* from the fall (it must drive *toward* it).
2. The derivative/`k_omega` term became **positive velocity feedback** — actively anti-damping.

This is exactly the reported symptom: *"it moves in the opposite direction of balancing."*

Fixed by adding a single canonical coordinate, `theta_from_upright` ($\theta = \texttt{angle\_dev} - 180$,
wrapped), used by every controller, with **positive** gains on both terms.

Empirical check (8° initial tilt, closed loop in the corrected simulator):

| Convention | Result |
|---|---|
| `u = -k\theta` (old) | diverges to 46° in 0.2 s |
| `u = +k\theta` (new) | holds the pole at 8°, no divergence |

---

## 1. Simulator physics were wrong in three independent ways

The old `step()` integrated

```
accel = (m*g*l*sin(theta) - b*vel + torque) / (m*l*l)
```

**(a) Wrong actuation coupling — the critical one.** The motor entered as an *additive torque*
`+torque`. The real pendulum is unactuated at the pivot; it is driven only through the cart's
inertial reaction:

$$J\ddot{\theta} = mgl\sin\theta \;-\; ml\,\ddot{x}\cos\theta \;-\; b\dot{\theta}$$

Note the **minus** sign. The old model inverted the input coupling, so any controller tuned
against it pushes the wrong way on hardware — the simulator was *rewarding* the very sign error
described in §0.

**(b) Wrong inertia.** Used $J = ml^2$ (point mass) while `docs/math/energy.md` derives
$J = \frac{4}{3}ml^2$ for a uniform rod with COM at $l$ — a 33 % error.

**(c) No cart state at all.** $x$ and $\dot{x}$ were never simulated, so rail limits and cart
drift — the dominant real-world failure mode — were invisible to training.

Also fixed: explicit → **semi-implicit Euler** (much better behaved for oscillatory systems), and
the state vector is now $[x, \dot{x}, \theta, \dot{\theta}]$ with an inelastic end-stop.

---

## 2. The `UnboundLocalError` — HIL mode could never run

In the old `step()`, `theta_err` was assigned *only* inside the `if self.simulated:` branch, but
the reward block below used it unconditionally. Confirmed by direct execution:

```
HIL step FAILED -> UnboundLocalError: cannot access local variable 'theta_err'
```

So **hardware-in-the-loop RL crashed on its first step, always.** Compounding it,
`SerialClient` never defined `last_angle`/`last_velocity`, which the HIL path read via
`getattr(..., default)` — it would have silently fed constants even without the crash. Both are
now implemented (wrap-aware velocity included).

---

## 3. Reward was computed on the *pre-step* state

The reward block read `theta_err`/`vel` — the values captured *before* integration. Every reward
and termination signal was attributed to the wrong transition, teaching a one-step-shifted
objective. Reward is now computed from the post-step state.

## 4. Reward shaping was unlearnable

The only positive signal was a bonus inside $|\theta| \le 1^\circ$ — about **0.6 %** of the state
range. A fresh policy essentially never lands there, so the gradient was flat and PPO/SAC had
nothing to climb. Added a dense $\cos\theta$ term ($+1$ upright → $-1$ hanging), rebalanced the
quadratic weights, and added a soft cart-excursion penalty.

## 5. Discount horizon was ~10× too short

At $dt = 10$ ms, $\gamma = 0.99$ is an effective horizon of $\frac{dt}{1-\gamma} = 1.0$ s — shorter
than the behaviour being learned. Now $\gamma = 0.999$ (≈10 s).

## 6. `terminated` made swing-up impossible

Termination fired at $|\theta| > 45^\circ$ **unconditionally**. A swing-up episode starts hanging
at $180^\circ$, so it terminated on step 1 — the swing-up task could never train. Now gated on
`task="balance"`.

---

## 7. Swing-up had no energy target (it could only spin)

The law was $u = \mathrm{sign}(\dot\theta\cos\theta)P_{max}$ — **no energy-error term**. It pumps
energy without bound and never stops, so the pendulum spins forever. (The RL "spin penalty" was
treating this symptom.) The Åström–Furuta law needs $\tilde{E} = E - E_{top}$:

$$u = -k_E\,\tilde{E}\,\dot\varphi\cos\varphi \;\Rightarrow\; \dot{V} = -mlk_E(\tilde{E}\dot\varphi\cos\varphi)^2 \le 0$$

which pumps below the target, **brakes above it**, and vanishes at it. Also added a kick-start
(the law is identically zero at exact rest) and correct $J = \frac{4}{3}ml^2$.

Result: closest approach to upright went from "spins forever" to **0.00°**, entering the ±25°
capture basin in 1.6 s.

`HybridBalancer.compute_action` also hardcoded `velocity=0.0`, which blinded the energy law to
swing direction — it now estimates velocity by differencing.

---

## 8. Deadband mapping discarded most of the control range

```python
speed = min_power + int(abs_output * (max_power - min_power) / 255.0)
```

`abs_output` is a gain-weighted quantity in *degrees*, divided by an unrelated 255. At `kp=15`,
a 5° error gives `output=75` → mapped to only ~62/255. It also never clamped the normalized
fraction to 1.0. Now explicitly saturating: `min(1.0, abs_output/255.0)`.

## 9. GUI reward inflated control cost by ~65,000×

```python
0.001 * (norm_action * 255.0)**2
```

`norm_action` was already divided by 255, then multiplied straight back — a factor of $255^2$
versus the RL env's `0.001 * norm_action**2`. The effort term swamped every other component, so
the dashboard's reward readout bore no relation to the training objective.

## 10. `benchmark_controllers.py` was dead code

Called `np_array(...)` **before its definition** (module-level `NameError` on every run), wrote
only `env.state` while the integrator read a separate vector, double-negated velocity to
compensate for the §0 sign bug, and judged settling on stale pre-step velocity. All fixed; it now
runs and reports settling times of 0.60–0.92 s.

## 11. Firmware header was corrupted (would not compile)

```c
#define AS5600_REG_ANGLE_L    0x0F333333333333333333333333333333333333333333333
```

An editing accident left a 45-digit integer literal — the firmware could not build. Restored to
`0x0F`.

---

## 12. STRUCTURAL LIMIT — this rig cannot hold position with one sensor

This is the most important finding, and no amount of tuning fixes it.

The platform measures **only the pendulum angle**. The cart-pole system has four states
$[x, \dot{x}, \theta, \dot{\theta}]$; `docs/research_paper_formulation.md` correctly derives a
four-state LQR with $K \in \mathbb{R}^{1\times4}$ — but the deployed law feeds back only
$[\theta, \dot{\theta}]$. Angle-only feedback can hold the pole vertical while the cart position
goes unregulated, so any residual tilt bias makes the cart drift until it hits the end-stop.

Measured on the corrected simulator (0.4 m rail, 8° initial tilt):

| Feedback | Outcome |
|---|---|
| $[\theta,\dot\theta]$ only | pole held within 8°, but **hits rail end-stop in 0.89 s** |
| $+\,[x,\dot{x}]$ added | **stable for the full 15 s**, $\theta \to 0.00°$, cart returns to 0.000 m |

The pole was never the problem — the **cart** was. To balance for more than ~1 s of rail travel,
add cart odometry (motor-shaft encoder or belt encoder) and close the loop on all four states.
This is a hardware requirement, not a tuning issue.

---

## 12b. Round 2 — the overshoot ("reaches 180° then falls back")

Follow-up investigation of the reported overshoot. Reproduced by modelling the *full* live
signal chain (12-bit quantisation → GUI EMA → controller EMA → `min_power` deadband). Three
independent causes, all measured:

### (a) Cascaded filter lag ≈ one instability time constant

An EMA with factor $\alpha$ at 100 Hz has time constant $\frac{1-\alpha}{\alpha}\,dt$:

| Filter | $\alpha$ | Lag |
|---|---|---|
| GUI velocity EMA | 0.20 | 40 ms |
| Controller derivative EMA | 0.08 | **115 ms** |
| **Cascade** | | **≈155 ms** |

The pendulum's own instability time constant is $\sqrt{J/mgl} = 148$ ms. So the **damping term
— the one that prevents overshoot — arrived a full time constant late.** The controller was
reacting to where the pole *was*, not where it is. Fixed: GUI EMA → 0.55, controller
`alpha` → 0.45 (single dominant filter instead of two cascaded ones).

### (b) The equilibrium deadzone removed the corrections that prevent overshoot

`equilibrium_deadzone_deg = 0.4` zeroed the command whenever the pole was within 0.4° of
upright — exactly the small early corrections that arrest an incipient fall. Measured effect
(6° start): peak error **14.40° with** the deadzone vs **6.00° without**. Now defaults to 0.0.

### (c) The real killer: it was never the pole, it was the cart

Instrumenting the failure shows the pole is held near 0.5° for *seconds* while the cart
accelerates away monotonically:

```
step   theta   cart_v   cart_x   pwm
  80    0.30    0.352    0.266   -48
 320    0.61    0.533    1.331    63
 480    1.77    0.770    2.343    78
 640   11.92    2.163    4.366   255 SAT   <- saturated, pole now unrecoverable
 720 -122.73                              <- fallen
```

The pole angle only diverges *after* the cart saturates. The "overshoot" is the end-stop
impact from §12, not a tuning problem.

### The fix: dead-reckoned cart observer (no new hardware)

We can't *measure* cart velocity — but we know every PWM command we sent. So the missing two
LQR states are recovered by integrating the actuator model
$$\dot{v} = a_{cmd} - c\,v, \qquad a_{cmd} = \tfrac{u}{255}\,\ddot{x}_{max}$$
and fed back as $k_v\hat{v} + k_x\hat{x}$ (`k_cart_v=150`, `k_cart_x=200`).

Results on the **real 0.4 m rail**, 30 s, full quantisation + filter chain:

| Configuration | Result |
|---|---|
| Old config (α=.08, deadzone, no cart term) | **hits rail in 0.8 s** |
| New `PIDBalancer` defaults | **holds 30 s**, peak 6.00°, final 0.13°, cart at −0.000 m |
| New `LQRBalancer` defaults | **holds 30 s**, peak 6.00°, final 0.09°, cart at −0.000 m |

Robustness — the estimator assumes $\ddot{x}_{max}=6, c=2$; the plant was varied:

| Plant | Result |
|---|---|
| $\ddot{x}_{max}$ 3.0 → 12.0 (0.5×–2×) | all hold 30 s |
| $c$ = 0.8 → 4.0 | all hold 30 s |

So the observer does **not** need accurate calibration — it only has to be roughly right to
cancel the drift. Recovery envelope is now ≈±10° (15°+ still exceeds the rail's ability to
recover, which is a genuine physical limit of a 0.4 m track).

> [!NOTE]
> This *reduces* but does not eliminate the §12 recommendation. Dead-reckoning has no feedback,
> so it cannot observe belt slip, wheel slip, or a stalled motor. A real cart encoder remains
> the robust solution; this makes the existing hardware work in the meantime.

---

## 13. Documentation corrections

Verified **correct** (left alone): the Lagrangian derivation in `kinematics.md`/`energy.md`/
`equations_of_motion.md`, and the linearized state-space matrices in
`state_space_linearization.md` — I checked the latter numerically against a direct solve of the
coupled equations (max error $6\times10^{-14}$).

Corrected:
- **Lyapunov proof sign error** (`research_paper_formulation.md` §4.2): the law was written
  $u = +k_E\tilde{E}\dot\theta\cos\theta$, which gives $\dot{V} \ge 0$ — divergence. Needs the
  minus sign. Also softened the LaSalle conclusion: convergence is to the energy manifold
  $E = E_0$, **not** to the upright point (the hanging equilibrium is in the same invariant set).
- **Fabricated latency measurements**: the "measured empirical timing across 10,000 samples"
  (1.45 ms total) is not produced by any instrumentation in this repo. A single 8-char line at
  115200 baud is ~700 µs *per direction* before USB's 1 ms frame scheduling, so the realistic
  round trip is 3–5 ms. Relabelled as estimates with a warning to measure before publishing.
- **Unsupported sim2real claim**: domain randomisation over latency/noise is described as
  implemented and achieving "zero-shot transfer." It is not implemented at all. Marked as planned.
- **Reward function mismatch**: the paper's spin penalty ($+20$ flat) didn't match the code
  ($50 + 0.15\Delta$), nor did the weights. Synced.
- **Inertia inconsistency** in `controller_implementations.md` ($ml^2$ vs $\frac{4}{3}ml^2$).

---

## Verification

All 27 tests pass. Tests that encoded the old inverted convention were rewritten, and new ones
added for: the catch-the-fall sign law, velocity damping polarity, swing-up energy targeting
(pumps below / stops at target), kick-start from rest, and swing-up non-termination while hanging.

## Recommended next steps

1. **Add cart odometry** (§12) — the single highest-value change; without it, sustained balance is
   not achievable on a 0.4 m rail.
2. Confirm the physical constants (`m`, `l`, `max_cart_accel`, `cart_damping`) against the real
   hardware; they are currently plausible estimates, and `max_cart_accel = 6.0 m/s²` in particular
   should be measured.
3. Measure true round-trip latency, then add matching domain randomisation before trusting sim2real.
4. Retrain from scratch — every previously saved policy learned against inverted input coupling
   and a shifted reward, so old checkpoints are not salvageable.
