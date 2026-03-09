# Theory Primer: Kinematics, Null-Space & RL
## From Basics to Research — Through the Lens of a 4-DOF Planar Arm

> **How to use this document.**  
> Read it in order the first time. Come back to individual sections as reference while coding.
> Every concept is first explained in plain language, then stated precisely in math.
> The running example is always the same 4-DOF planar arm — so by the end, you will have
> built up the entire theoretical stack on a single concrete object.

---

## Part I — Kinematics

---

### 1. Configuration Space and Joint Space

**Plain language first.**  
A robot arm is just a chain of rigid sticks connected by hinges. To know where the arm *is*,
you only need to know the angle at each hinge. Everything else — where the tip is, where each
elbow is — can be computed from those angles. The collection of all joint angles is called the
**configuration** of the robot.

**More precisely.**  
For a robot with $n$ joints, the configuration space (also called **C-space** or **joint space**)
is the set of all possible joint angle vectors:

$$\boldsymbol{q} = [q_1,\; q_2,\; \ldots,\; q_n]^\top \in \mathcal{C} \subseteq \mathbb{R}^n$$

For our 4-DOF planar arm: $n = 4$, so $\boldsymbol{q} \in \mathbb{R}^4$. Each $q_i$ is constrained
to $[-\pi, \pi]$, making $\mathcal{C}$ a 4-dimensional torus $(\mathbb{T}^4)$.

**Why this matters.**  
The robot does not "know" about Cartesian space. It only moves its motors. Every concept from
here on is about the relationship between what the motor angles do and what happens to the
end-effector in Cartesian space.

---

### 2. Forward Kinematics (FK)

**Plain language.**  
FK answers: *given joint angles $\boldsymbol{q}$, where is the end-effector?*  
This is always uniquely solvable — one configuration gives exactly one end-effector pose.

**The 4-DOF planar case.**  
Let link lengths be $\boldsymbol{L} = [l_1, l_2, l_3, l_4]$. Define the **cumulative angle** at
joint $k$:

$$\phi_k = \sum_{i=1}^{k} q_i \qquad (\phi_0 = 0)$$

This is the angle the $k$-th link makes with the global $x$-axis. The end-effector position is:

$$\boxed{
\begin{aligned}
x_e &= \sum_{k=1}^{4} l_k \cos\phi_k \\
y_e &= \sum_{k=1}^{4} l_k \sin\phi_k
\end{aligned}
}$$

Define the FK map as:

$$\boldsymbol{f} : \mathbb{R}^4 \to \mathbb{R}^2, \qquad \boldsymbol{x}_e = \boldsymbol{f}(\boldsymbol{q})$$

**Homogeneous transforms (for reference).**  
In the general 3D case, FK is built from **homogeneous transformation matrices**
$\mathbf{T}_i \in \mathrm{SE}(3)$ chained as $\mathbf{T}_{0\to n} = \mathbf{T}_1 \mathbf{T}_2 \cdots \mathbf{T}_n$.
The planar case uses $\mathrm{SE}(2)$ transforms — same idea, 3×3 matrices instead of 4×4.

**Denavit-Hartenberg (DH) convention.**  
A systematic way to define these transforms for any serial chain using 4 parameters per joint
$(a_i, d_i, \alpha_i, \theta_i)$. For pure planar revolute joints: $d_i = 0$, $\alpha_i = 0$,
$a_i = l_i$. You do not need DH for the planar PoC, but you will for 3D.

---

### 3. Task Space and the Workspace

**Task space** (or **operational space**) is the space in which you specify what the robot should
*do* — usually the position (and/or orientation) of the end-effector. Dimension $m$.

For the 4-DOF planar arm tracking a 2D point: $m = 2$, so $\boldsymbol{x}_e \in \mathbb{R}^2$.

**Workspace** is the set of all end-effector positions reachable by some configuration:

$$\mathcal{W} = \{\boldsymbol{f}(\boldsymbol{q}) \mid \boldsymbol{q} \in \mathcal{C}\}$$

For our arm with total reach $L_\text{tot} = l_1 + l_2 + l_3 + l_4$, the workspace is an annulus:

$$\mathcal{W} = \{\boldsymbol{x} \in \mathbb{R}^2 : |L_\text{tot} - 2\max(l_k)| \leq \|\boldsymbol{x}\| \leq L_\text{tot}\}$$

**Key dimension relationship:**
- $n = 4$: degrees of freedom (DOF) in joint space
- $m = 2$: degrees of freedom in task space
- $n - m = 2$: **degrees of redundancy** — how many "spare" DOF the arm has

This leftover is the null-space. More on this in Part II.

---

### 4. The Jacobian — The Bridge Between Joint Space and Task Space

**Plain language.**  
The Jacobian answers: *if joint angles change at rate $\dot{\boldsymbol{q}}$, how fast does the
end-effector move?* It is the instantaneous linear map from joint velocities to task-space
velocities.

**Formal definition.**  
The Jacobian $\mathbf{J}(\boldsymbol{q})$ is the matrix of partial derivatives of the FK map:

$$\mathbf{J}(\boldsymbol{q}) = \frac{\partial \boldsymbol{f}(\boldsymbol{q})}{\partial \boldsymbol{q}} \in \mathbb{R}^{m \times n}$$

giving the velocity relationship:

$$\dot{\boldsymbol{x}}_e = \mathbf{J}(\boldsymbol{q})\,\dot{\boldsymbol{q}}$$

For our 4-DOF planar arm, $\mathbf{J} \in \mathbb{R}^{2 \times 4}$. Each column $k$ is the
contribution of joint $k$'s velocity to the end-effector velocity:

$$\mathbf{J}_{:,k} = \frac{\partial \boldsymbol{f}}{\partial q_k} = \begin{bmatrix} -\sum_{i=k}^{4} l_i \sin\phi_i \\ +\sum_{i=k}^{4} l_i \cos\phi_i \end{bmatrix}$$

**Intuition for the column formula.**  
When joint $k$ rotates, it sweeps all links $k, k+1, \ldots, n$ through an arc.
The Jacobian column for joint $k$ is the tangential velocity at the tip due to that sweep.

**Geometric interpretation.**  
Plot all four columns of $\mathbf{J}$ as 2D arrows from the origin. The end-effector velocity
is a linear combination of these arrows weighted by $\dot{\boldsymbol{q}}$. The Jacobian tells you
which joints are "effective" at moving the tip in a given direction.

---

### 5. Manipulability — How Dexterous is the Arm?

**Plain language.**  
Near some configurations, the arm can move its end-effector easily in any direction.
Near others (like being fully stretched out), it can only move in one direction no matter
how you spin the joints. **Manipulability** quantifies this dexterity.

**Yoshikawa manipulability measure:**

$$w(\boldsymbol{q}) = \sqrt{\det\!\left(\mathbf{J}(\boldsymbol{q})\,\mathbf{J}(\boldsymbol{q})^\top\right)}$$

- $w > 0$: arm is away from singular configurations; can move freely
- $w = 0$: **singular configuration** — the arm has "lost" a degree of freedom
  (e.g., fully stretched, or two links folded back on each other)

**Geometric meaning via the SVD.**  
Write $\mathbf{J} = \mathbf{U}\,\mathbf{\Sigma}\,\mathbf{V}^\top$ (singular value decomposition). Then:

$$\mathbf{J}\mathbf{J}^\top = \mathbf{U}\,\mathbf{\Sigma}^2\,\mathbf{U}^\top, \qquad \det(\mathbf{J}\mathbf{J}^\top) = \prod_{i=1}^{m} \sigma_i^2$$

So $w = \prod_i \sigma_i$. The singular values $\sigma_i$ are the "radii" of the
**manipulability ellipsoid** — the set of unit-norm task-space velocities achievable by
unit-norm joint velocities. When one $\sigma_i \to 0$, the ellipsoid collapses to a line:
singularity.

**Why it matters for your research.**  
The classical null-space secondary objective is often $\nabla_{\boldsymbol{q}} w(\boldsymbol{q})$ —
gradient ascent on manipulability. Your paper's claim is that an RL policy can do better
than this gradient, especially when multiple objectives compete simultaneously.

---

### 6. Inverse Kinematics — The Hard Direction

**Plain language.**  
FK is easy — one $\boldsymbol{q}$ gives one $\boldsymbol{x}_e$. But if you want the arm to go to a
*target* position $\boldsymbol{x}_d$, you need to invert this: find $\boldsymbol{q}$ such that
$\boldsymbol{f}(\boldsymbol{q}) = \boldsymbol{x}_d$. This is **Inverse Kinematics (IK)**, and it is hard for
three reasons:

1. The map $\boldsymbol{f}$ is nonlinear — no analytic inverse in general
2. It can have **multiple solutions** (for redundant arms, infinitely many)
3. It can have **no solution** (target outside workspace)

**The velocity-level IK.**  
Rather than solving position IK directly, work at the velocity level. You want:

$$\dot{\boldsymbol{x}}_e = \dot{\boldsymbol{x}}_d \quad\Rightarrow\quad \mathbf{J}\,\dot{\boldsymbol{q}} = \dot{\boldsymbol{x}}_d$$

This is an underdetermined linear system ($m < n$). The solution is not unique — there are
infinitely many $\dot{\boldsymbol{q}}$ satisfying it. This is exactly where the null-space lives.

---

### 7. The Pseudo-Inverse — Minimum-Norm IK

**Plain language.**  
Of all the infinite joint velocity solutions to $\mathbf{J}\dot{\boldsymbol{q}} = \dot{\boldsymbol{x}}_d$,
the **pseudo-inverse** picks the one with the smallest $\|\dot{\boldsymbol{q}}\|^2$ — the
minimum-energy solution.

**Moore-Penrose pseudo-inverse** (for full row rank $\mathbf{J}$, i.e., $m < n$ and away from
singularities):

$$\mathbf{J}^\dagger = \mathbf{J}^\top (\mathbf{J}\mathbf{J}^\top)^{-1} \in \mathbb{R}^{n \times m}$$

Properties:
- $\mathbf{J}\,\mathbf{J}^\dagger = \mathbf{I}_m$ (left inverse in task space)
- $\mathbf{J}^\dagger\,\mathbf{J} \neq \mathbf{I}_n$ (not a full inverse — this is the key)
- $\mathbf{J}^\dagger\,\dot{\boldsymbol{x}}_d$ is the minimum-norm solution

**Damped pseudo-inverse** (numerically stable near singularities):

$$\mathbf{J}^\dagger_\lambda = \mathbf{J}^\top (\mathbf{J}\mathbf{J}^\top + \lambda\mathbf{I}_m)^{-1}$$

$\lambda = 0.01$ is a typical value. This trades a small amount of tracking accuracy
for numerical stability — essential when implementing on a real system.

---

## Part II — The Null Space

---

### 8. What the Null Space Is

**Plain language.**  
The null space of $\mathbf{J}$ is the set of joint velocities that produce *zero* end-effector
velocity. Moving in the null space is like rearranging the arm's elbows without moving the
tip. The tip stays fixed; the arm changes shape.

**Formal definition:**

$$\mathcal{N}(\mathbf{J}) = \{\boldsymbol{v} \in \mathbb{R}^n \mid \mathbf{J}\,\boldsymbol{v} = \boldsymbol{0}\}$$

For $\mathbf{J} \in \mathbb{R}^{m \times n}$ with full row rank ($\text{rank} = m$):

$$\dim\!\left(\mathcal{N}(\mathbf{J})\right) = n - m$$

For our arm: $\dim(\mathcal{N}) = 4 - 2 = 2$. There is a 2-dimensional family of joint motions
that do absolutely nothing to the end-effector.

**Intuition.**  
Think of it this way: the arm has 4 motors. It only needs 2 to control $(x_e, y_e)$.
The remaining 2 "motor directions" (the null space) are free — you can use them for anything.

---

### 9. The Null-Space Projector

**The projector matrix:**

$$\mathbf{N}(\boldsymbol{q}) = \mathbf{I}_n - \mathbf{J}^\dagger\,\mathbf{J} \in \mathbb{R}^{n \times n}$$

This matrix takes any vector $\boldsymbol{v} \in \mathbb{R}^n$ and projects it onto the null space of $\mathbf{J}$. After projection:

$$\mathbf{J}\,\underbrace{(\mathbf{I} - \mathbf{J}^\dagger\,\mathbf{J})}_{\mathbf{N}}\,\boldsymbol{v} = \mathbf{J}\boldsymbol{v} - \mathbf{J}\mathbf{J}^\dagger\,\mathbf{J}\boldsymbol{v} = \mathbf{J}\boldsymbol{v} - \mathbf{J}\boldsymbol{v} = \boldsymbol{0}$$

So $\mathbf{J}\,\mathbf{N} = \mathbf{0}$ **always** — this is the fundamental identity of null-space control.

**Key properties to verify in code:**

| Property | Formula | What it means |
|---|---|---|
| Annihilation | $\mathbf{J}\,\mathbf{N} = \mathbf{0}$ | Null-space actions have zero EE effect |
| Idempotency | $\mathbf{N}^2 = \mathbf{N}$ | Projecting twice = projecting once |
| Symmetry | $\mathbf{N}^\top = \mathbf{N}$ | Orthogonal projection |
| Rank | $\text{rank}(\mathbf{N}) = n - m$ | 2 free dimensions for our arm |

**What $\mathbf{N}$ looks like geometrically.**  
$\mathbb{R}^n$ is split into two orthogonal subspaces:
- **Row space of $\mathbf{J}$**: vectors that *do* move the EE — dimension $m = 2$
- **Null space of $\mathbf{J}$**: vectors that *don't* move the EE — dimension $n - m = 2$

$\mathbf{J}^\dagger\mathbf{J}$ projects onto the row space. $\mathbf{N} = \mathbf{I} - \mathbf{J}^\dagger\mathbf{J}$ projects
onto the null space. They are complementary projectors that sum to the identity.

---

### 10. The General IK Solution and Why It's Beautiful

**The complete solution** to $\mathbf{J}\,\dot{\boldsymbol{q}} = \dot{\boldsymbol{x}}_d$ is:

$$\boxed{\dot{\boldsymbol{q}} = \underbrace{\mathbf{J}^\dagger\,\dot{\boldsymbol{x}}_d}_{\text{minimum-norm particular solution}} + \underbrace{\mathbf{N}\,\boldsymbol{z}}_{\text{null-space homogeneous component}}}$$

where $\boldsymbol{z} \in \mathbb{R}^n$ is **completely arbitrary**.

**Why this is beautiful:**
- The first term handles the task — getting the EE where it needs to go
- The second term handles everything else — how the arm's "body" moves
- These two are orthogonal: they do not interfere with each other
- $\boldsymbol{z}$ is a free variable you can use for *any* secondary purpose

**Verify it satisfies the constraint:**

$$\mathbf{J}\,\dot{\boldsymbol{q}} = \mathbf{J}\mathbf{J}^\dagger\,\dot{\boldsymbol{x}}_d + \mathbf{J}\mathbf{N}\,\boldsymbol{z} = \dot{\boldsymbol{x}}_d + \boldsymbol{0} = \dot{\boldsymbol{x}}_d \;\checkmark$$

**Classical choices for $\boldsymbol{z}$:**
- $\boldsymbol{z} = \alpha\,\nabla_{\boldsymbol{q}} w(\boldsymbol{q})$: move toward higher manipulability
- $\boldsymbol{z} = \alpha\,\nabla_{\boldsymbol{q}} H_\text{jlim}(\boldsymbol{q})$: move joints toward midrange
- $\boldsymbol{z} = -\alpha\,\boldsymbol{q}$: return joints to home configuration

**Your research:** Replace all of these with $\boldsymbol{z} = \pi_\theta(\boldsymbol{s}, \boldsymbol{z}_\text{task})$ — a learned policy.

---

### 11. Singularities — When the Null Space Breaks Down

**What happens at a singularity.**  
At a singular configuration, $\det(\mathbf{J}\mathbf{J}^\top) = 0$, meaning $\mathbf{J}\mathbf{J}^\top$ is not
invertible. The pseudo-inverse becomes numerically unstable. The arm has effectively "lost"
a degree of freedom — it cannot move the EE in some direction no matter how it moves its joints.

**Types of singularities for a planar arm:**
- **Boundary singularity**: arm fully extended ($w = 0$, tip at maximum reach)
- **Interior singularity**: two links are collinear (fold-back configuration)

**Effect on the null space.**  
At a boundary singularity: the null space *grows* (from dimension 2 to dimension 3 for our arm),
because the arm has fewer ways to affect the EE. At an interior singularity: some task-space
directions become unreachable, but the null space structure depends on which singular values collapse.

**Practical fix — Damped Least Squares (DLS):**
The damped pseudo-inverse introduces a small $\lambda > 0$ that regularises the inverse:

$$\mathbf{J}^\dagger_\lambda = \mathbf{J}^\top(\mathbf{J}\mathbf{J}^\top + \lambda\mathbf{I})^{-1}$$

This is equivalent to solving the modified problem:
$\min \|\dot{\boldsymbol{x}}_d - \mathbf{J}\dot{\boldsymbol{q}}\|^2 + \lambda\|\dot{\boldsymbol{q}}\|^2$, which
always has a unique solution and degrades gracefully near singularities.

---

### 12. Closed-Loop Inverse Kinematics (CLIK)

**The problem with open-loop velocity IK.**  
If you integrate $\dot{\boldsymbol{q}} = \mathbf{J}^\dagger\,\dot{\boldsymbol{x}}_d$ over time, numerical errors
accumulate. The actual EE drifts from the desired trajectory.

**CLIK adds a feedback correction term:**

$$\dot{\boldsymbol{q}} = \mathbf{J}^\dagger\,(\dot{\boldsymbol{x}}_d + K\,\boldsymbol{e}), \qquad \boldsymbol{e} = \boldsymbol{x}_d - \boldsymbol{x}_e$$

where $K > 0$ is the proportional gain. The term $K\,\boldsymbol{e}$ drives the EE back to the desired
position whenever it drifts. This is why it's "closed-loop."

**Stability analysis.**  
Task-space error dynamics:

$$\dot{\boldsymbol{e}} = \dot{\boldsymbol{x}}_d - \dot{\boldsymbol{x}}_e = \dot{\boldsymbol{x}}_d - \mathbf{J}\dot{\boldsymbol{q}}$$

Substituting the CLIK law and assuming $\mathbf{J}\mathbf{J}^\dagger \approx \mathbf{I}_m$ (away from
singularities):

$$\dot{\boldsymbol{e}} = \dot{\boldsymbol{x}}_d - (\dot{\boldsymbol{x}}_d + K\,\boldsymbol{e}) = -K\,\boldsymbol{e}$$

This gives exponential convergence: $\|\boldsymbol{e}(t)\| = \|\boldsymbol{e}(0)\|\,e^{-Kt}$.

**The combined controller (your system):**

$$\dot{\boldsymbol{q}} = \underbrace{\mathbf{J}^\dagger(\dot{\boldsymbol{x}}_d + K\,\boldsymbol{e})}_{\text{CLIK: guarantees tracking}} + \underbrace{\mathbf{N}\,\pi_\theta(\boldsymbol{s}, \boldsymbol{z})}_{\text{RL: uses null-space freely}}$$

The null-space term adds nothing to $\dot{\boldsymbol{e}}$ because $\mathbf{J}\mathbf{N} = \mathbf{0}$.
Error convergence is unchanged. This is the key theorem of your paper.

---

### 13. Why Classical Null-Space Objectives Fall Short

The classical approach picks a scalar "comfort" function $g(\boldsymbol{q})$ and uses
$\boldsymbol{z} = \nabla_{\boldsymbol{q}} g(\boldsymbol{q})$ as the null-space velocity. Problems:

**Problem 1 — Single objective, multiple needs.**  
Maximising manipulability helps dexterity but may drive joints to limits.
Avoiding joint limits improves safety but may reduce manipulability.
These objectives compete. A scalar gradient can only optimise one at a time.

**Problem 2 — Task-agnostic.**  
During a *reaching* phase, manipulability matters most — prepare a good grasp pose.
During a *holding* phase, joint-limit avoidance matters most — don't strain the motors.
A fixed gradient has no concept of phase.

**Problem 3 — Local, greedy.**  
The gradient $\nabla g(\boldsymbol{q})$ only knows about the current configuration.
It can get stuck in local optima of $g$, or make choices now that are bad for the next 10 steps.

**What RL can do differently:**  
- Optimise a *composite* reward over a *horizon* of steps
- Condition on task phase via the embedding $\boldsymbol{z}$
- Implicitly model the trade-offs between objectives through the value function

---

## Part III — Reinforcement Learning

---

### 14. The Markov Decision Process (MDP)

**Plain language.**  
RL is the framework for learning from trial and error. At each step, an agent observes the
world, takes an action, receives a reward, and the world changes. The goal is to choose actions
that maximise the total reward over time. The formal model for this is the MDP.

**An MDP is a tuple** $(\mathcal{S}, \mathcal{A}, P, r, \gamma)$:

| Symbol | Name | Meaning |
|---|---|---|
| $\mathcal{S}$ | State space | Everything the agent can observe |
| $\mathcal{A}$ | Action space | Everything the agent can do |
| $P(s' \mid s, a)$ | Transition dynamics | How the world changes |
| $r(s, a)$ | Reward function | Immediate feedback signal |
| $\gamma \in [0, 1)$ | Discount factor | How much future rewards are worth today |

**For your null-space environment:**
- $\mathcal{S} \subset \mathbb{R}^{11}$: joint angles, velocities, EE error, task phase
- $\mathcal{A} \subset \mathbb{R}^4$: raw policy output (projected into null-space inside the env)
- $P$: deterministic (your forward kinematics + CLIK + null-space projection)
- $r$: composite null-space reward (manipulability + joint-limit + smoothness)
- $\gamma = 0.99$: standard for continuous-control tasks with 200-step episodes

---

### 15. Policy, Value Function, and the RL Objective

**Policy** $\pi : \mathcal{S} \to \Delta(\mathcal{A})$:  
A mapping from states to probability distributions over actions.
A **deterministic policy** returns a single action: $a = \pi(s)$.
A **stochastic policy** samples: $a \sim \pi(\cdot \mid s)$.

**Return** $G_t$: the discounted sum of future rewards from step $t$:

$$G_t = r_t + \gamma r_{t+1} + \gamma^2 r_{t+2} + \cdots = \sum_{k=0}^{\infty} \gamma^k r_{t+k}$$

**State value function** $V^\pi(s)$: expected return starting from state $s$ under policy $\pi$:

$$V^\pi(s) = \mathbb{E}_\pi[G_t \mid s_t = s]$$

**Action-value (Q) function** $Q^\pi(s, a)$: expected return taking action $a$ in state $s$,
then following $\pi$:

$$Q^\pi(s, a) = \mathbb{E}_\pi[G_t \mid s_t = s, a_t = a]$$

**The RL objective**: find policy $\pi^*$ maximising expected return from any starting state:

$$\pi^* = \arg\max_\pi \mathbb{E}_{s_0 \sim \rho_0}\left[V^\pi(s_0)\right]$$

**Why $\gamma < 1$?** Two reasons: (1) makes the infinite sum converge; (2) encodes preference
for rewards sooner rather than later. For $\gamma = 0.99$ and a 200-step episode, the reward
at the last step is worth $0.99^{200} \approx 0.13$ compared to the first step.

---

### 16. Policy Gradient — How RL Learns

**The core idea.**  
Most modern deep RL methods are policy gradient methods. They directly parameterise the policy
$\pi_\theta$ with a neural network and take gradient steps on the RL objective:

$$\nabla_\theta J(\theta) = \mathbb{E}_{\pi_\theta}\left[\sum_t \nabla_\theta \log \pi_\theta(a_t \mid s_t) \cdot G_t\right]$$

This is the **REINFORCE** / policy gradient theorem. Intuitively: increase the probability of
actions that led to high return; decrease the probability of actions that led to low return.

**The variance problem.**  
The gradient estimator above has very high variance — the return $G_t$ depends on many future
random actions. This makes learning slow and unstable. The solution is the **Actor-Critic** architecture.

---

### 17. Actor-Critic Architecture

**Two networks, two roles:**

- **Actor** $\pi_\theta(a \mid s)$: decides what to do (the policy)
- **Critic** $Q_\phi(s, a)$ or $V_\phi(s)$: evaluates how good the action was (the value function)

**The advantage function** $A^\pi(s, a)$: how much better is action $a$ than the average?

$$A^\pi(s, a) = Q^\pi(s, a) - V^\pi(s)$$

Using the advantage instead of raw return dramatically reduces variance. The policy gradient becomes:

$$\nabla_\theta J(\theta) \approx \mathbb{E}\left[\nabla_\theta \log \pi_\theta(a_t \mid s_t) \cdot A^\pi(s_t, a_t)\right]$$

**How to estimate the advantage.**  
Using the **TD error** as a low-variance advantage estimator:

$$\delta_t = r_t + \gamma\,V_\phi(s_{t+1}) - V_\phi(s_t)$$

This is the "surprise" — how much better was the actual outcome than the critic predicted.
The critic is trained to minimise $\mathbb{E}[\delta_t^2]$.

---

### 18. Soft Actor-Critic (SAC) — The Algorithm You Will Use

SAC is the current gold standard for **continuous control** tasks (actions are real-valued vectors,
not discrete choices). Understanding it deeply will serve you well.

**The entropy-augmented objective.**  
SAC adds an entropy bonus $\mathcal{H}(\pi(\cdot \mid s))$ to the reward:

$$J^\text{SAC}(\theta) = \mathbb{E}_\pi\left[\sum_t r(s_t, a_t) + \alpha\,\mathcal{H}(\pi(\cdot \mid s_t))\right]$$

where $\alpha > 0$ is the **temperature** and $\mathcal{H}(p) = -\mathbb{E}[\log p]$ is entropy.

**Why entropy matters for your task.**  
The null-space is genuinely under-constrained — many different null-space vectors produce
similar rewards. Without entropy regularisation, the policy could collapse to a single
deterministic null-space direction, missing better solutions nearby. The entropy term
keeps the policy stochastic and exploratory throughout training.

**Automatic temperature tuning.**  
SAC sets $\alpha$ automatically to maintain a target entropy level $\mathcal{H}_\text{target}$.
For a $d$-dimensional action space, a good default is $\mathcal{H}_\text{target} = -d$.
The temperature adapts via:

$$\alpha \leftarrow \alpha - \beta_\alpha \nabla_\alpha \mathbb{E}[\log \pi_\alpha(a \mid s) + \mathcal{H}_\text{target}]$$

You don't need to tune $\alpha$ manually — set `ent_coef='auto'` in Stable-Baselines3.

**The SAC update loop (simplified):**

```
Collect transition (s, a, r, s') using current policy
Store in replay buffer D

For each training step:
  Sample batch from D
  Update critic: minimise  (Q(s,a) - (r + γ * V(s')))²
  Update actor:  maximise  Q(s, π(s)) + α * H(π(·|s))
  Update target networks: soft update θ_target ← τ θ + (1-τ) θ_target
  Update temperature α: to match target entropy
```

**Off-policy learning.**  
SAC stores transitions in a **replay buffer** and learns from them repeatedly — not just from
the most recent episode. This is why it is sample-efficient. The replay buffer breaks the
temporal correlation between samples, which stabilises training.

---

### 19. The Replay Buffer and Sample Efficiency

**Why a replay buffer?**  
On-policy methods (like PPO) throw away data after each update — each experience is used
exactly once. Off-policy methods (like SAC) keep all past transitions in a buffer and reuse them.

**Buffer mechanics:**
- Fixed capacity $N$ (e.g., $10^5$ transitions)
- When full: overwrite the oldest entry (FIFO)
- Sample uniformly at random for each update

**Why this helps for your task.**  
Null-space exploration is sparse — most random actions produce similar mediocre rewards.
Good null-space behaviours (high manipulability + good joint angles simultaneously) are
discovered rarely. The replay buffer ensures these rare good experiences are used many times.

---

### 20. Neural Network Policy and Value Function

**Policy network (Actor).**  
For SAC with continuous actions, the actor is a **Gaussian policy**:

$$\pi_\theta(a \mid s) = \mathcal{N}(\mu_\theta(s),\, \sigma_\theta(s))$$

The network outputs mean $\mu_\theta(s)$ and log standard deviation $\log\sigma_\theta(s)$.
Actions are sampled and **squashed through $\tanh$** to bound them to $[-1, 1]^4$.

**Critic network (Q-function).**  
SAC uses **two Q-networks** $Q_{\phi_1}(s,a)$ and $Q_{\phi_2}(s,a)$ and takes the minimum:

$$Q_\text{target}(s, a) = \min(Q_{\phi_1}(s,a),\, Q_{\phi_2}(s,a))$$

This **clipped double-Q** trick prevents overestimation of Q-values, which is a major source
of instability in actor-critic methods.

**Architecture for your PoC:**

```
Actor:  MLP [11] → [256] → [256] → [8]    (4 means + 4 log-stds)
Critic: MLP [11+4] → [256] → [256] → [1]  (state-action value)
```

Both networks use ReLU activations. The 11-dim observation is your state; the 4-dim action
is the raw output before null-space projection.

---

### 21. The Null-Space Projection as a Differentiable Layer

**Where projection happens.**  
The null-space projection $\boldsymbol{a}_\text{env} = \mathbf{N}(\boldsymbol{q})\,\pi_\theta(s)$ happens
**inside the environment's step function**, not inside the neural network.

This means:
- The actor outputs an unconstrained 4-vector
- The environment projects it before applying it to the robot
- The Q-critic is trained on the *unprojected* action (what the policy output)
- The environment stores the *projected* action in the replay buffer

**Why this is clean.**  
The actor never "sees" the projection — it just learns to output vectors that are useful
after projection. The null-space structure is baked into the physics, not the learning.

**Could you put the projection inside the network?**  
Yes — you could add $\mathbf{N}$ as a differentiable PyTorch layer. The gradients would flow
through it correctly (since $\mathbf{N}$ is a smooth function of $\boldsymbol{q}$). This is an
advanced variant worth exploring in Phase 3.

---

### 22. MDP Formulation for Your Specific System

Bringing together all the concepts:

**State** $\boldsymbol{s}_t \in \mathbb{R}^{11}$:

$$\boldsymbol{s}_t = \left[\frac{\boldsymbol{q}}{\pi},\; \dot{\boldsymbol{q}}_\text{prev},\; \boldsymbol{e}_\text{ee},\; z_\text{phase}\right]$$

- $\boldsymbol{q}/\pi$: normalised joint angles (4-dim)
- $\dot{\boldsymbol{q}}_\text{prev}$: previous joint velocity — gives the policy velocity awareness (4-dim)
- $\boldsymbol{e}_\text{ee} = \boldsymbol{x}_d - \boldsymbol{x}_e$: EE error for situational awareness only, not in reward (2-dim)
- $z_\text{phase} \in \{0, 1\}$: task phase scalar (1-dim)

**Why include EE error in state but not reward?**  
The policy needs to *know* where the EE is relative to the target (e.g., near-singular
configurations arise near the target). But if the EE error is in the reward, the RL agent
might learn to move the EE instead of relying on CLIK. Including it in state but excluding
it from reward maintains the separation of concerns.

**Action** $\boldsymbol{a}_t \in [-1, 1]^4$: raw actor output, projected to null-space in env.

**Reward** $r_t$:

$$r_t = \underbrace{w_1\,\sqrt{\det(\mathbf{J}\mathbf{J}^\top)}}_{\text{manipulability}} + \underbrace{w_2\,\left(-\textstyle\sum_i \left(\frac{q_i}{q_\text{range}}\right)^4\right)}_{\text{joint-limit avoidance}} + \underbrace{w_3\,(-\|\Delta\boldsymbol{q}\|^2)}_{\text{smoothness}}$$

Default weights: $w_1 = 1.0$, $w_2 = 0.3$, $w_3 = 0.1$.

**Transition**: deterministic CLIK + null-space integration (no noise for PoC).

**Discount**: $\gamma = 0.99$.

**Episode length**: 200 steps, $\Delta t = 0.05\,\text{s}$ → 10 seconds of robot time.

---

### 23. The Stability Theorem — Formal Statement

This is the theoretical backbone of the paper. State it clearly.

**Theorem (EE convergence decoupling).**  
Let $\pi_\theta : \mathcal{S} \to \mathbb{R}^n$ be any bounded policy with
$\|\pi_\theta(s)\| \leq M < \infty$ for all $s$. Under the combined controller:

$$\dot{\boldsymbol{q}} = \mathbf{J}^\dagger(\dot{\boldsymbol{x}}_d + K\,\boldsymbol{e}) + \mathbf{N}\,\pi_\theta(\boldsymbol{s}, z)$$

the task-space error $\boldsymbol{e}(t) = \boldsymbol{x}_d(t) - \boldsymbol{x}_e(t)$ satisfies:

$$\|\boldsymbol{e}(t)\| \leq \|\boldsymbol{e}(0)\|\,e^{-K\sigma_\min t} \qquad \text{away from singularities}$$

where $\sigma_\min = \sigma_\min(\mathbf{J}\mathbf{J}^\dagger)$, **independent of** $\pi_\theta$.

**Proof.** Define $V = \frac{1}{2}\|\boldsymbol{e}\|^2$. Then:

$$\dot{V} = \boldsymbol{e}^\top\dot{\boldsymbol{e}} = \boldsymbol{e}^\top(\dot{\boldsymbol{x}}_d - \dot{\boldsymbol{x}}_e)$$

$$\dot{\boldsymbol{x}}_e = \mathbf{J}\dot{\boldsymbol{q}} = \underbrace{\mathbf{J}\mathbf{J}^\dagger(\dot{\boldsymbol{x}}_d + K\boldsymbol{e})}_{\approx\, \dot{\boldsymbol{x}}_d + K\boldsymbol{e}} + \underbrace{\mathbf{J}\mathbf{N}\pi_\theta}_{=\,\boldsymbol{0}}$$

Therefore: $\dot{\boldsymbol{e}} = -K\,\mathbf{J}\mathbf{J}^\dagger\,\boldsymbol{e}$, and:

$$\dot{V} = -K\,\boldsymbol{e}^\top\mathbf{J}\mathbf{J}^\dagger\,\boldsymbol{e} \leq -K\sigma_\min\,\|\boldsymbol{e}\|^2 = -2K\sigma_\min\,V$$

By Gronwall's inequality: $V(t) \leq V(0)\,e^{-2K\sigma_\min t}$, giving $\|\boldsymbol{e}(t)\| \leq \|\boldsymbol{e}(0)\|\,e^{-K\sigma_\min t}$. $\blacksquare$

**What this means for your paper:**  
You can train any RL policy you like in the null-space — even a terrible one — and the
end-effector will still converge. The RL agent cannot break the primary controller.

---

## Part IV — Connecting It All

---

### 24. The Information Flow Diagram

```
                    ┌─────────────────────────────────────────┐
                    │           Environment (step)             │
                    │                                          │
  s_t ──────────►  │   CLIK: q_dot_1 = J†(x_d_dot + K*e)    │
                    │   NULL:  q_dot_2 = N @ a_t              │
  a_t ──────────►  │   TOTAL: q_dot  = q_dot_1 + q_dot_2     │
  (from actor)     │   INTEGRATE: q ← q + q_dot * dt         │
                    │   REWARD: r = f(q)  [null-space only]   │
                    │   OBS: s_{t+1}                          │
                    └─────────────────────────────────────────┘
                              │           │
                           r_t, s_{t+1}  EE error (info only)
                              │
                    ┌─────────▼───────────┐
                    │   Replay Buffer D   │
                    │   (s, a, r, s')     │
                    └─────────┬───────────┘
                              │  sample batch
                    ┌─────────▼───────────┐
                    │   SAC Update        │
                    │   Critic: min TD²   │
                    │   Actor:  max Q+αH  │
                    │   Temp:   match H*  │
                    └─────────────────────┘
```

The key partition: **everything above** the replay buffer is the environment (your domain);
**everything below** is the RL algorithm (standard SAC, unmodified).

---

### 25. What You Are Actually Testing

At its core, the PoC tests one empirical claim:

> **A policy trained to maximise only null-space metrics, constrained to act only in the null-space,
> outperforms all fixed analytical null-space objectives on a composite metric — while
> producing identical end-effector tracking performance.**

This claim decomposes into testable hypotheses:

| Hypothesis | How to test it | Expected outcome |
|---|---|---|
| H1: Structural decoupling holds | Unit test: $\|\mathbf{J}\mathbf{N}\boldsymbol{v}\| < 10^{-9}$ | Pass for all configs |
| H2: CLIK converges independently | EE error with zero null-space action | $\leq 0.01$ in all episodes |
| H3: Learned > fixed (composite) | SAC reward vs. all baselines | ≥ 10% improvement |
| H4: Task-conditioning helps | Phase-conditioned vs. agnostic | Higher per-phase metric |
| H5: Sample efficiency advantage | End-to-end RL steps to match null-space RL | ≥ 5× more steps |

H1 and H2 are theoretical checks — they should always pass.  
H3, H4, H5 are empirical claims — they are the actual paper contributions.

---

### 26. Concepts to Learn Next (After This PoC)

When you move to 3D and mobile manipulators, you will need:

**From kinematics:**
- Rotation representations: rotation matrices, Euler angles, quaternions, axis-angle
- Velocity kinematics in 3D: the full 6D Jacobian (3 translational + 3 rotational rows)
- Denavit-Hartenberg parameters for 3D serial chains
- Nonholonomic constraints: Pfaffian form $\mathbf{A}(\boldsymbol{q})\dot{\boldsymbol{q}} = \boldsymbol{0}$,
  the extended null-space projector $\mathbf{N}_\text{nonhol}$

**From dynamics:**
- Newton-Euler and Lagrangian equations of motion: $\mathbf{M}(\boldsymbol{q})\ddot{\boldsymbol{q}} + \boldsymbol{C}(\boldsymbol{q},\dot{\boldsymbol{q}}) + \boldsymbol{g}(\boldsymbol{q}) = \boldsymbol{\tau}$
- Operational space dynamics (Khatib 1987): how to control in task space with force/torque

**From RL:**
- Constrained MDPs (CMDPs) and Lagrangian RL — if you want formal safety guarantees
- Hierarchical RL and task priority — for multi-task mobile manipulation
- Domain randomisation for sim-to-real transfer
- Hindsight Experience Replay (HER) — useful if you reformulate as a goal-conditioned problem

**From control theory:**
- Lyapunov stability for nonlinear systems
- Control Barrier Functions (CBFs) — formal safety guarantees that complement RL
- Model Predictive Control (MPC) — an alternative to RL for the secondary objective

---

### 27. Quick Reference — Formulas You Will Use Every Day

| Formula | What it computes |
|---|---|
| $\boldsymbol{x}_e = \sum_k l_k[\cos\phi_k, \sin\phi_k]^\top$ | End-effector position (planar FK) |
| $\mathbf{J}_{:,k} = [-\sum_{i\geq k} l_i\sin\phi_i,\; \sum_{i\geq k} l_i\cos\phi_i]^\top$ | Jacobian column $k$ |
| $\mathbf{J}^\dagger = \mathbf{J}^\top(\mathbf{J}\mathbf{J}^\top + \lambda\mathbf{I})^{-1}$ | Damped pseudo-inverse |
| $\mathbf{N} = \mathbf{I} - \mathbf{J}^\dagger\mathbf{J}$ | Null-space projector |
| $\mathbf{J}\mathbf{N} = \mathbf{0}$ | Fundamental null-space identity |
| $w = \sqrt{\det(\mathbf{J}\mathbf{J}^\top)}$ | Yoshikawa manipulability |
| $\dot{\boldsymbol{q}} = \mathbf{J}^\dagger\dot{\boldsymbol{x}}_d + \mathbf{N}\boldsymbol{z}$ | General IK velocity solution |
| $\dot{\boldsymbol{q}}_\text{CLIK} = \mathbf{J}^\dagger(\dot{\boldsymbol{x}}_d + K\boldsymbol{e})$ | CLIK primary controller |
| $G_t = \sum_{k\geq 0} \gamma^k r_{t+k}$ | Discounted return |
| $Q^\pi(s,a) = \mathbb{E}_\pi[G_t \mid s_t{=}s, a_t{=}a]$ | Action-value function |
| $A^\pi(s,a) = Q^\pi(s,a) - V^\pi(s)$ | Advantage function |
| $\delta_t = r_t + \gamma V(s_{t+1}) - V(s_t)$ | TD error |

---

*End of Primer. Build `kinematics.py` next — run the unit tests before anything else.*