# Null-Space RL — Planar 4-DOF Proof of Concept Plan

**Goal:** Validate the core hypothesis — that a null-space-constrained RL policy learns better
secondary behaviours (manipulability, joint-limit avoidance) than any fixed analytical objective —
on a fully self-contained Python environment with matplotlib visualisation, before committing to
a full 3D mobile manipulator simulation.

---

## Why Planar 4-DOF First

A 4-DOF planar arm tracking a 2D end-effector position ($m = 2$) has a **2-dimensional null-space**
(`rank(N) = 4 - 2 = 2`). This is the minimal configuration where:

- The null-space is non-trivial (rank > 0) and has geometric intuition
- The Jacobian, pseudo-inverse, and null-space projector can be computed and visualised analytically
- An RL episode runs in milliseconds on CPU — no GPU, no simulator license
- matplotlib can animate the arm, joint angles, reward curves, and null-space vectors in real time
- The full experiment fits on a MacBook, runs in a single Python file

**All theoretical claims carry directly to 3D.** The structural decoupling proof (Section 5 of the
research plan) is dimension-agnostic. If it works here, extending to MuJoCo + mobile base is
an engineering task, not a research risk.

---

## Stack

| Tool | Version | Purpose | Install |
|---|---|---|---|
| Python | 3.11 | Runtime | `pyenv install 3.11.9` |
| NumPy | latest | Kinematics, linear algebra | `pip install numpy` |
| SciPy | latest | Damped least squares, integration | `pip install scipy` |
| matplotlib | latest | Arm animation + live plots | `pip install matplotlib` |
| Gymnasium | 0.29+ | RL environment interface | `pip install gymnasium` |
| Stable-Baselines3 | 2.x | SAC implementation (Mac native) | `pip install stable-baselines3` |
| Weights & Biases | latest | Experiment tracking | `pip install wandb` |
| pytest | latest | Unit tests | `pip install pytest` |

**Why Stable-Baselines3 over CleanRL here?**  
SB3 requires zero modification to run SAC — you just pass your custom `gymnasium.Env`.
CleanRL is better when you need to modify the algorithm internals (Phase 3 of the full plan).
For this PoC, SB3 lets you focus entirely on the environment design and the null-space projection.

---

## Repository Structure

```
planar_nullspace/
├── kinematics.py          # Forward kinematics, Jacobian, null-space projector
├── env.py                 # Gymnasium environment (NullSpacePlanarEnv)
├── baselines.py           # Classical null-space objectives (manipulability gradient, etc.)
├── train.py               # SAC training script with SB3
├── evaluate.py            # Load checkpoint, run episodes, generate plots
├── visualise.py           # matplotlib animation of arm + live metrics
├── tests/
│   ├── test_kinematics.py # Unit tests for FK, Jacobian, null-space identity
│   └── test_env.py        # Unit tests for env step, reward, observation
└── notebooks/
    └── exploration.ipynb  # Interactive prototyping
```

---

## Step 1 — Kinematics Module (`kinematics.py`)

### 1.1 Forward Kinematics

A planar 4-DOF arm with link lengths `L = [l1, l2, l3, l4]` and joint angles
`q = [q1, q2, q3, q4]`.

The end-effector position (2D) is:

```
x_e = l1*cos(q1) + l2*cos(q1+q2) + l3*cos(q1+q2+q3) + l4*cos(q1+q2+q3+q4)
y_e = l1*sin(q1) + l2*sin(q1+q2) + l3*sin(q1+q2+q3) + l4*sin(q1+q2+q3+q4)
```

Using cumulative angle sums `phi_k = sum(q_1..q_k)`:

```python
def forward_kinematics(q: np.ndarray, L: np.ndarray) -> np.ndarray:
    """
    Returns end-effector (x, y) for a planar n-DOF arm.
    q: joint angles (n,)
    L: link lengths (n,)
    """
    phi = np.cumsum(q)           # cumulative angle at each joint
    x = np.sum(L * np.cos(phi))
    y = np.sum(L * np.sin(phi))
    return np.array([x, y])
```

Also implement `all_joint_positions(q, L)` returning the (x, y) of every joint — needed for
visualisation and for obstacle-proximity rewards later.

### 1.2 Analytical Jacobian

For a planar arm, the Jacobian `J` is `(2 x n)`. Each column `k` is:

```
J[:, k] = sum_{i=k}^{n} [-L_i * sin(phi_i),  L_i * cos(phi_i)]
```

where `phi_i = sum(q_1 .. q_i)`.

```python
def jacobian(q: np.ndarray, L: np.ndarray) -> np.ndarray:
    """
    Returns the (2 x n) task-space Jacobian for a planar n-DOF arm.
    """
    n = len(q)
    phi = np.cumsum(q)
    J = np.zeros((2, n))
    for k in range(n):
        J[0, k] = -np.sum(L[k:] * np.sin(phi[k:]))
        J[1, k] = +np.sum(L[k:] * np.cos(phi[k:]))
    return J
```

### 1.3 Damped Pseudo-Inverse and Null-Space Projector

```python
def damped_pinv(J: np.ndarray, lam: float = 0.01) -> np.ndarray:
    """Damped Moore-Penrose pseudo-inverse: J^T (J J^T + lam*I)^-1"""
    m = J.shape[0]
    return J.T @ np.linalg.inv(J @ J.T + lam * np.eye(m))

def null_space_projector(J: np.ndarray, lam: float = 0.01) -> np.ndarray:
    """N = I - J_dag @ J.  Projects vectors into null-space of J."""
    n = J.shape[1]
    Jdag = damped_pinv(J, lam)
    return np.eye(n) - Jdag @ J
```

### 1.4 Manipulability

```python
def manipulability(J: np.ndarray) -> float:
    """Yoshikawa manipulability: sqrt(det(J J^T))"""
    return np.sqrt(max(np.linalg.det(J @ J.T), 0.0))
```

### 1.5 Unit Tests (`tests/test_kinematics.py`)

These must pass before any RL training begins:

```python
def test_null_space_identity():
    """J @ N @ v == 0 for all v, all q."""
    rng = np.random.default_rng(42)
    L = np.array([1.0, 0.8, 0.6, 0.4])
    for _ in range(1000):
        q = rng.uniform(-np.pi, np.pi, 4)
        v = rng.standard_normal(4)
        J = jacobian(q, L)
        N = null_space_projector(J)
        residual = np.linalg.norm(J @ N @ v)
        assert residual < 1e-9, f"Null-space identity violated: {residual}"

def test_null_space_idempotent():
    """N @ N == N."""
    q = np.array([0.3, -0.5, 0.8, -0.2])
    L = np.array([1.0, 0.8, 0.6, 0.4])
    J = jacobian(q, L)
    N = null_space_projector(J)
    assert np.allclose(N @ N, N, atol=1e-8)

def test_jacobian_finite_difference():
    """Analytical Jacobian matches finite-difference approximation."""
    q = np.array([0.3, -0.5, 0.8, -0.2])
    L = np.array([1.0, 0.8, 0.6, 0.4])
    J_analytical = jacobian(q, L)
    eps = 1e-6
    J_fd = np.zeros((2, 4))
    for k in range(4):
        dq = np.zeros(4); dq[k] = eps
        J_fd[:, k] = (forward_kinematics(q + dq, L) - forward_kinematics(q - dq, L)) / (2 * eps)
    assert np.allclose(J_analytical, J_fd, atol=1e-5)
```

Run with: `pytest tests/test_kinematics.py -v`

---

## Step 2 — Gymnasium Environment (`env.py`)

### 2.1 Environment Specification

```
State space (obs):  [q (4), q_dot (4), ee_error (2), task_phase (1)] = 11-dim
Action space:       [-1, 1]^4  (raw policy output, projected into null-space inside step())
Episode length:     200 steps
dt:                 0.05 s
Link lengths:       [1.0, 0.8, 0.6, 0.4]  (total reach = 2.8)
Joint limits:       q_i in [-pi, pi]
Task:               Track a fixed or slowly moving 2D target within reachable workspace
```

### 2.2 Step Function — Core Architecture

```python
def step(self, action: np.ndarray):
    J    = jacobian(self.q, self.L)           # (2 x 4)
    Jdag = damped_pinv(J, lam=self.lam)       # (4 x 2)
    N    = null_space_projector(J, lam=self.lam)  # (4 x 4)

    # --- Primary controller: CLIK ---
    ee   = forward_kinematics(self.q, self.L)
    e_ee = self.target - ee                   # (2,)
    q_dot_primary = Jdag @ (self.K * e_ee)    # CLIK gain K

    # --- Null-space RL action ---
    q_dot_null = N @ action                   # guaranteed: J @ q_dot_null == 0

    # --- Combine and integrate ---
    q_dot = q_dot_primary + self.alpha * q_dot_null
    self.q = self.q + q_dot * self.dt
    self.q = np.clip(self.q, -np.pi, np.pi)   # joint limit enforcement

    # --- Reward (null-space metrics ONLY) ---
    reward = self._null_space_reward()
    obs    = self._get_obs()
    done   = self._is_done()
    return obs, reward, done, False, {"ee_error": np.linalg.norm(e_ee)}
```

**Critical:** The EE error `e_ee` appears in the `info` dict for logging but **not** in the reward.
The RL agent is never rewarded for tracking — only for null-space quality. This is the central
architectural claim.

### 2.3 Reward Function

```python
def _null_space_reward(self) -> float:
    J    = jacobian(self.q, self.L)
    w    = manipulability(J)                       # Yoshikawa measure

    # Joint-limit avoidance: penalise q near limits (quartic well)
    q_mid   = 0.0                                  # symmetric limits around 0
    q_range = np.pi
    r_jlim  = -np.sum(((self.q - q_mid) / q_range) ** 4)

    # Smoothness: penalise large joint velocities
    r_smooth = -np.linalg.norm(self.q - self.q_prev) ** 2

    return (
        self.w1 * w
      + self.w2 * r_jlim
      + self.w3 * r_smooth
    )
```

Default weights: `w1=1.0, w2=0.3, w3=0.1`. These will be tuned during ablation.

### 2.4 Observation

```python
def _get_obs(self) -> np.ndarray:
    ee       = forward_kinematics(self.q, self.L)
    e_ee     = self.target - ee
    return np.concatenate([
        self.q / np.pi,          # normalised joint angles  (4,)
        self.q_dot_prev,         # previous joint velocity  (4,) -- for velocity awareness
        e_ee,                    # EE error for state info  (2,) -- not in reward
        [self.phase_encoding],   # task phase scalar        (1,)
    ])                           # total: 11-dim
```

### 2.5 Task Phases (For Task-Conditioning Ablation)

Define two phases toggling mid-episode:

| Phase | Encoding | Null-Space Priority |
|---|---|---|
| `APPROACH` | 0.0 | Manipulability (arm ready to grasp) |
| `HOLD` | 1.0 | Joint-limit avoidance (stable hold) |

Switch phase at step 100 of a 200-step episode. The task-conditioned policy should adapt its
null-space behaviour at the switch; the task-agnostic policy cannot.

---

## Step 3 — Classical Baselines (`baselines.py`)

These replace `action` in the `step()` call to benchmark against the learned policy:

```python
def manipulability_gradient(q, L, epsilon=1e-4):
    """Gradient of manipulability w.r.t. q — classical secondary objective."""
    grad = np.zeros(len(q))
    for k in range(len(q)):
        dq = np.zeros(len(q)); dq[k] = epsilon
        grad[k] = (manipulability(jacobian(q + dq, L))
                 - manipulability(jacobian(q - dq, L))) / (2 * epsilon)
    return grad

def joint_limit_gradient(q, q_mid=0.0, q_range=np.pi):
    """Gradient pushing joints toward centre of range."""
    return -(q - q_mid) / (q_range ** 2)

def zero_secondary(q, L):
    """Minimum-norm solution — no secondary objective."""
    return np.zeros(len(q))
```

All three baselines are evaluated over identical episode distributions and compared against the
learned policy using the same composite reward metric.

---

## Step 4 — Training Script (`train.py`)

```python
import gymnasium
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import EvalCallback
import wandb
from wandb.integration.sb3 import WandbCallback

from env import NullSpacePlanarEnv

def train(config: dict):
    wandb.init(project="nullspace-planar-poc", config=config)

    env      = NullSpacePlanarEnv(**config["env"])
    eval_env = NullSpacePlanarEnv(**config["env"])

    model = SAC(
        "MlpPolicy", env,
        learning_rate  = config["lr"],
        buffer_size    = 100_000,
        batch_size     = 256,
        tau            = 0.005,
        gamma          = 0.99,
        ent_coef       = "auto",   # entropy tuning — important for null-space exploration
        verbose        = 1,
        tensorboard_log= "./tb_logs/",
    )

    eval_cb = EvalCallback(
        eval_env,
        best_model_save_path="./checkpoints/",
        eval_freq=5000,
        n_eval_episodes=20,
        deterministic=True,
    )

    model.learn(
        total_timesteps=config["total_steps"],
        callback=[eval_cb, WandbCallback()],
    )
    model.save("nullspace_sac_final")
    wandb.finish()

if __name__ == "__main__":
    config = {
        "env": {"task_conditioned": True, "reward_weights": [1.0, 0.3, 0.1]},
        "lr": 3e-4,
        "total_steps": 300_000,
    }
    train(config)
```

**Expected training time on MacBook M2:** ~15 minutes for 300k steps. The environment is pure
numpy with no simulation overhead.

---

## Step 5 — Visualisation (`visualise.py`)

Build a matplotlib animation showing four panels simultaneously:

```
┌─────────────────────┬─────────────────────┐
│  Arm + workspace    │  Manipulability      │
│  (animated)         │  over episode        │
├─────────────────────┼─────────────────────┤
│  Joint angles       │  Null-space vector   │
│  vs. limits         │  magnitude vs. step  │
└─────────────────────┴─────────────────────┘
```

### Panel 1 — Arm Animation

```python
def draw_arm(ax, q, L, target, color='steelblue'):
    positions = all_joint_positions(q, L)   # (n+1, 2) including base
    xs = [p[0] for p in positions]
    ys = [p[1] for p in positions]
    ax.plot(xs, ys, '-o', color=color, linewidth=2, markersize=6)
    ax.plot(*target, 'r*', markersize=14, label='Target')
    ax.plot(*positions[-1], 'go', markersize=10, label='End-effector')
    ax.set_aspect('equal')
    ax.set_xlim(-3, 3); ax.set_ylim(-3, 3)
```

### Panel 4 — Null-Space Vector

Plot `||N @ action||` over the episode alongside `||q_dot_primary||` to visually confirm the
null-space contribution is non-trivial and varies with task phase.

### Running Comparisons

```python
# visualise.py entry point: run all policies side-by-side
python visualise.py --policies learned manipulability_grad zero --episodes 5
```

This renders a 3-column animation comparing the arm behaviour under each policy while tracking
the same sequence of targets.

---

## Step 6 — Experiment Sequence

Run experiments in this order. Each builds on the last and maps to a claim in the paper.

### Experiment 1 — Null-Space Identity Verification
**Before any RL.** Run the unit tests:
```
pytest tests/ -v
```
All tests must pass. Specifically, `test_null_space_identity` with 1000 random configurations.
This is proof that your architecture is correctly implemented.

### Experiment 2 — CLIK Primary Controller Validation
**Before any RL.** Run 50 episodes with `action = np.zeros(4)` (no null-space contribution).
Confirm that EE error converges to < 0.01 in every episode. Plot EE error vs. step.  
**Expected result:** Exponential convergence with rate `K * sigma_min(J @ J.T)`.

### Experiment 3 — Baseline Benchmark
Run 200 evaluation episodes for each classical null-space objective:
- Zero secondary objective (`action = 0`)
- Manipulability gradient
- Joint-limit gradient

Record per-episode: mean manipulability, mean joint-limit score, mean EE error.  
**Expected result:** Each classical baseline excels at one metric but not others. None dominates
on the composite reward from Eq. (4) of the research plan.

### Experiment 4 — Task-Agnostic RL (Core Ablation A)
Train SAC without task-phase conditioning (`phase_encoding = 0` always). Evaluate on the composite
null-space reward. Compare against all baselines from Experiment 3.  
**Expected result:** Learned policy outperforms all baselines on composite reward. EE error remains
low (same as baselines) because CLIK handles it regardless.

### Experiment 5 — Task-Conditioned RL (Core Ablation B)
Train SAC with task-phase conditioning (phase toggles at step 100). Evaluate:
- During `APPROACH` phase: compare manipulability vs. baselines
- During `HOLD` phase: compare joint-limit score vs. baselines
- At phase switch: measure latency for policy to adapt null-space behaviour

**Expected result:** Task-conditioned policy achieves higher per-phase metric than task-agnostic
policy, confirming that conditioning on `z` enables phase-specialised null-space behaviour.

### Experiment 6 — End-to-End RL Comparison (Sanity Check)
Train an SAC agent with full joint velocity control (no null-space projection, no CLIK).
The reward includes both EE error and null-space metrics.  
**Expected result:** End-to-end RL takes 5-10x more steps to converge and has worse EE tracking
during early training. This motivates the structured approach.

### Experiment 7 — Moving Target (Robustness)
Repeat Experiments 4 and 5 with a smoothly moving target (circular trajectory in workspace).
**Expected result:** The CLIK layer maintains tracking; the null-space policy adapts secondary
behaviour without retraining.

---

## Step 7 — Evaluation Metrics

Record these metrics for every experiment:

| Metric | Formula | Measures |
|---|---|---|
| Mean manipulability | `mean(sqrt(det(J J^T)))` over episode | Dexterity |
| EE error | `mean(‖x_e - x_d‖)` over episode | Primary task performance |
| Joint-limit score | `mean(-sum(((q-q_mid)/q_range)^4))` | Safety/comfort |
| Composite reward | Eq. (4) of research plan | Overall null-space quality |
| Sample efficiency | Steps to reach 90% of peak composite reward | Training cost |
| Phase-adaptation latency | Steps until metric improves after phase switch | Task-conditioning quality |

---

## Step 8 — Plotting (`evaluate.py`)

Generate publication-quality figures for the eventual paper. Key plots to produce:

```
Figure 1:  Learning curve — composite reward vs. timesteps (all methods)
Figure 2:  Per-metric comparison bar chart (manipulability, jlim, EE error)
Figure 3:  Phase-conditioned behaviour — null-space metric vs. step with phase indicator
Figure 4:  Arm animation snapshots — 3 columns (zero, manip_grad, learned) at steps 0/50/100/150
Figure 5:  Sample efficiency curve — steps to 90% peak reward, each method
```

All figures use a consistent style:

```python
import matplotlib.pyplot as plt
plt.rcParams.update({
    "font.family": "serif",
    "font.size": 11,
    "axes.spines.right": False,
    "axes.spines.top": False,
    "figure.dpi": 150,
})
```

---

## Implementation Order (Day-by-Day)

| Day | Task | Done When |
|---|---|---|
| 1 | `kinematics.py` + all unit tests passing | `pytest tests/test_kinematics.py` green |
| 2 | `env.py` step function + `test_env.py` | Null-space identity test passes inside env |
| 3 | Experiment 2 (CLIK validation) + visualise panel 1 | EE error < 0.01 in 50 episodes |
| 4 | Experiments 3 (baselines) | Baseline numbers recorded in CSV |
| 5 | Training run: Experiment 4 (task-agnostic SAC) | Training completes; reward curves look reasonable |
| 6 | Training run: Experiment 5 (task-conditioned SAC) | Phase-adaptation visible in panel 4 |
| 7 | Experiment 6 (end-to-end comparison) | Sample efficiency gap confirmed |
| 8 | Generate all paper figures, clean up code | Publication-ready plots |

---

## Known Pitfalls and How to Avoid Them

### Pitfall 1 — Null-space gain `alpha` too large
If the null-space contribution dominates, CLIK can't converge fast enough. Start with `alpha = 0.2`
and increase only if EE error stays below 0.05.

### Pitfall 2 — Singular configurations
At full extension or zero-angle configurations, `det(J J^T) ≈ 0` and `Jdag` becomes numerically
unstable. Always use the damped pseudo-inverse. Also add a small random perturbation to the initial
joint configuration at episode reset to avoid starting near singularities:
```python
self.q = rng.uniform(-0.8*np.pi, 0.8*np.pi, 4)
```

### Pitfall 3 — Policy outputs large vectors that move joints to limits
Even though `N @ action` has no EE effect, it can slam joints into limits rapidly. Add a gradient
penalty term `r_smooth` (already in the reward) and clip joint velocities:
```python
q_dot = np.clip(q_dot, -self.max_qdot, self.max_qdot)
```

### Pitfall 4 — SAC entropy term kills null-space exploration
The entropy coefficient `ent_coef = "auto"` usually works, but if the policy collapses to near-zero
null-space actions, force a minimum: `ent_coef = 0.1`. The null-space is genuinely under-constrained
and needs entropy to explore it.

### Pitfall 5 — Task-phase not visible enough to the policy
If the phase scalar `[0, 1]` is too weak a signal, switch to a one-hot encoding `[1,0]` vs `[0,1]`
or use a sinusoidal encoding. The phase difference must be salient in the observation.

### Pitfall 6 — Comparing policies on different EE targets
Always evaluate all policies on the same fixed sequence of targets (seeded evaluation). Use
`eval_env = NullSpacePlanarEnv(seed=42)` to ensure fair comparison.

---

## Success Criteria

The PoC is successful if all of the following hold:

1. **Null-space identity holds:** `‖J @ N @ v‖ < 1e-9` for 1000 random configurations ✓
2. **CLIK tracks independently:** EE error stays < 0.05 regardless of null-space policy ✓
3. **Learned policy beats baselines:** composite reward > all classical baselines by ≥ 10% ✓
4. **Task-conditioning works:** per-phase metric is better with conditioning than without ✓
5. **End-to-end RL is less efficient:** needs ≥ 5x more timesteps to match null-space RL ✓

If (1) and (2) are confirmed, the theoretical architecture is correct.  
If (3) is confirmed, there is a publishable result.  
If (4) is confirmed, the task-conditioning contribution is defensible.  
If (5) is confirmed, the sample-efficiency argument is quantified.

---

## Next Step After This PoC

Once all 5 success criteria are met, the path to the full paper is:

```
Planar 4-DOF (this plan)
    ↓  replace matplotlib with MuJoCo MJCF
3D Fixed-Base 6-DOF Arm
    ↓  add differential-drive base + nonholonomic projector N_nonhol
Full Mobile Manipulator (RoArm-M2-S)
    ↓  real hardware validation
Paper submission (RA-L)
```

The planar PoC de-risks every component. If the null-space RL concept doesn't work here,
it won't work in 3D — and you will have found out in days, not months.