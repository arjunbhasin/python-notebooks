# Kinematic Analysis of a Mobile Manipulator
## Manipulability · Velocity Kinematics · Null Space
### System: Differential Drive Platform + 3R PUMA-like Arm

> **Scope.** This document builds the complete kinematic theory for a mobile manipulator
> consisting of a nonholonomic differential drive base carrying a 3-revolute (3R) PUMA-like arm.
> Every concept is developed from scratch, with every matrix, every constraint, and every
> projector derived explicitly. The goal is to have a single reference that connects the
> platform mechanics, arm kinematics, augmented system, manipulability theory, velocity
> transformation ratios, null-space structure, and null-space projector — all for this
> specific system.

---

## Part I — System Description and Degrees of Freedom

---

### 1. The Two Subsystems

**Differential Drive Base**  
A rigid platform mounted on two independently driven wheels of radius $r$, separated by
axle width $2d$. The platform can translate and rotate on a flat plane. Its configuration
is described by the pose in the world frame:

$$\boldsymbol{q}_b = [x,\; y,\; \theta]^\top \in \mathbb{R}^2 \times \mathbb{S}^1$$

where $(x, y)$ is the position of the platform centre and $\theta$ is the heading angle.

**3R PUMA-like Arm**  
A serial chain of three revolute joints mounted on the platform. "PUMA-like" means:
- Joint 1 ($q_1$): rotation about a vertical axis (shoulder pan) — axis $\hat{z}$
- Joint 2 ($q_2$): rotation about a horizontal axis (shoulder tilt) — axis $\hat{y}$
- Joint 3 ($q_3$): rotation about a horizontal axis (elbow) — axis $\hat{y}$

Link lengths: $l_1$ (shoulder-to-elbow), $l_2$ (elbow-to-wrist), with $l_0$ being the
height of the shoulder joint above the platform centre.

Arm joint vector: $\boldsymbol{q}_a = [q_1,\; q_2,\; q_3]^\top \in \mathbb{T}^3$

**Full Configuration Vector**

$$\boldsymbol{q} = [x,\; y,\; \theta,\; q_1,\; q_2,\; q_3]^\top \in \mathbb{R}^n, \quad n = 6$$

---

### 2. Degrees of Freedom Analysis

This is where mobile manipulators get interesting. Counting DOF naively gives $n = 6$.
But the platform has a **nonholonomic constraint** — it cannot move sideways.
This constraint is on *velocities*, not on positions, and it reduces the **controllable
velocity space** but not the **reachable configuration space**.

**Task space dimension.** For end-effector position tracking in 3D: $m = 3$.
For full pose (position + orientation): $m = 6$ (but the 3R arm can only set 3 of these
independently).

For position-only task: $m = 3$, $n = 6$, **degrees of redundancy** = $n - m = 3$.

**Effective DOF with nonholonomic constraint.**  
The differential drive has 2 controllable inputs (left and right wheel velocities $[\omega_L, \omega_R]^\top$),
but can only produce 2 independent base velocities: forward speed $v$ and angular rate $\dot\theta$.
Lateral velocity is constrained to zero. This does not reduce the configuration space (the base
can eventually reach any pose) but it does reduce the *instantaneous* velocity space by one.
More on this in Section 6.

---

## Part II — Forward Kinematics of the Combined System

---

### 3. Arm Forward Kinematics Using DH Parameters

The 3R PUMA arm is described using the **Denavit-Hartenberg (DH) convention**.
Each frame is related to the previous by four parameters $(a_i, d_i, \alpha_i, \theta_i)$:

$$\mathbf{T}_{i-1}^i = \begin{bmatrix}
c\theta_i & -s\theta_i c\alpha_i & s\theta_i s\alpha_i & a_i c\theta_i \\
s\theta_i & c\theta_i c\alpha_i & -c\theta_i s\alpha_i & a_i s\theta_i \\
0 & s\alpha_i & c\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

where $c\theta_i = \cos\theta_i$, $s\theta_i = \sin\theta_i$, and similarly for $\alpha_i$.

**DH parameters for the 3R PUMA arm:**

| Joint $i$ | $a_i$ | $d_i$ | $\alpha_i$ | $\theta_i$ |
|---|---|---|---|---|
| 1 (shoulder pan) | 0 | $d_1 = l_0$ | $-\pi/2$ | $q_1$ |
| 2 (shoulder tilt) | $a_2 = l_1$ | 0 | 0 | $q_2$ |
| 3 (elbow) | $a_3 = l_2$ | 0 | 0 | $q_3$ |

The overall arm transformation from the shoulder mount frame $\{S\}$ to the end-effector frame $\{E\}$:

$$\mathbf{T}_S^E(\boldsymbol{q}_a) = \mathbf{T}_S^1 \cdot \mathbf{T}_1^2 \cdot \mathbf{T}_2^3$$

Expanding:

$$\mathbf{T}_S^1 = \begin{bmatrix}
c_1 & 0 & -s_1 & 0 \\
s_1 & 0 & c_1 & 0 \\
0 & -1 & 0 & l_0 \\
0 & 0 & 0 & 1
\end{bmatrix}, \quad
\mathbf{T}_1^2 = \begin{bmatrix}
c_2 & -s_2 & 0 & l_1 c_2 \\
s_2 & c_2 & 0 & l_1 s_2 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}, \quad
\mathbf{T}_2^3 = \begin{bmatrix}
c_3 & -s_3 & 0 & l_2 c_3 \\
s_3 & c_3 & 0 & l_2 s_3 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}$$

where $c_i = \cos q_i$, $s_i = \sin q_i$.

**End-effector position in the shoulder frame** (the translational part of $\mathbf{T}_S^E$):

$$\boldsymbol{p}_E^S = \begin{bmatrix}
c_1(l_1 c_2 + l_2 c_{23}) \\
s_1(l_1 c_2 + l_2 c_{23}) \\
l_0 - l_1 s_2 - l_2 s_{23}
\end{bmatrix}$$

where $c_{23} = \cos(q_2 + q_3)$, $s_{23} = \sin(q_2 + q_3)$.

---

### 4. Base-to-World Transform and Full System FK

The platform pose defines a rigid-body transformation from the body frame to the world frame:

$$\mathbf{T}_W^B = \begin{bmatrix}
\cos\theta & -\sin\theta & 0 & x \\
\sin\theta & \cos\theta & 0 & y \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}$$

The shoulder mount is at a fixed offset $\boldsymbol{p}_{BS}$ from the platform centre in the body frame (typically directly above the platform centre: $\boldsymbol{p}_{BS} = [0, 0, h]^\top$).

**Full end-effector position in the world frame:**

$$\boldsymbol{x}_e = \mathbf{R}(\theta)\,\boldsymbol{p}_E^S + \begin{bmatrix} x \\ y \\ 0 \end{bmatrix} + \mathbf{R}(\theta)\,\boldsymbol{p}_{BS}$$

where $\mathbf{R}(\theta) = \begin{bmatrix} c\theta & -s\theta & 0 \\ s\theta & c\theta & 0 \\ 0 & 0 & 1 \end{bmatrix}$ is the rotation from body to world frame.

Substituting $\boldsymbol{p}_E^S$ and $\boldsymbol{p}_{BS} = [0, 0, h]^\top$:

$$\boldsymbol{x}_e = \begin{bmatrix}
x + c\theta\cdot c_1(l_1 c_2 + l_2 c_{23}) - s\theta\cdot s_1(l_1 c_2 + l_2 c_{23}) \\
y + s\theta\cdot c_1(l_1 c_2 + l_2 c_{23}) + c\theta\cdot s_1(l_1 c_2 + l_2 c_{23}) \\
h + l_0 - l_1 s_2 - l_2 s_{23}
\end{bmatrix}$$

Notice the $z$-component depends only on arm joints — the planar base motion does not affect height. This is a useful decoupling property of this architecture.

---

## Part III — The Jacobian of the Mobile Manipulator

---

### 5. The Jacobian — Decomposed View

The full Jacobian $\mathbf{J}(\boldsymbol{q}) \in \mathbb{R}^{m \times n}$ maps generalised velocity
$\dot{\boldsymbol{q}}$ to end-effector velocity $\dot{\boldsymbol{x}}_e$:

$$\dot{\boldsymbol{x}}_e = \mathbf{J}(\boldsymbol{q})\,\dot{\boldsymbol{q}}, \qquad
\mathbf{J} = \begin{bmatrix} \mathbf{J}_b & \mathbf{J}_a \end{bmatrix}$$

where:
- $\mathbf{J}_b \in \mathbb{R}^{m \times 3}$ is the **base Jacobian** — how platform motion moves the EE
- $\mathbf{J}_a \in \mathbb{R}^{m \times 3}$ is the **arm Jacobian** — how joint motion moves the EE

**Why split them?** Because they have fundamentally different mathematical structure:
$\mathbf{J}_a$ is a classical serial-chain Jacobian (trigonometric). $\mathbf{J}_b$ comes from
rigid-body kinematics (the arm is rigidly attached to the base, so platform motion translates
and rotates every point on the arm).

---

### 6. The Base Jacobian $\mathbf{J}_b$

The end-effector position in world coordinates is $\boldsymbol{x}_e = [x_e, y_e, z_e]^\top$.

**Effect of $\dot{x}$** (base translates in world $x$): the EE moves with the base, so $\partial \boldsymbol{x}_e / \partial x = [1, 0, 0]^\top$.

**Effect of $\dot{y}$**: similarly $\partial \boldsymbol{x}_e / \partial y = [0, 1, 0]^\top$.

**Effect of $\dot\theta$** (base rotates): the EE undergoes a rotation about the world-$z$
axis through the base centre. For a point at position $\boldsymbol{p} = [p_x, p_y, p_z]^\top$
relative to the base centre in world coordinates, a rotation $\dot\theta$ causes velocity:

$$\frac{\partial \boldsymbol{x}_e}{\partial \theta} = \hat{z} \times (\boldsymbol{x}_e - \boldsymbol{x}_\text{base}) = \begin{bmatrix} -(x_e - x)\sin\theta - (y_e - y)\cos\theta \\ (x_e - x)\cos\theta - (y_e - y)\sin\theta \\ 0 \end{bmatrix}$$

Wait — let's derive this cleanly. Define $\Delta_x = x_e - x$ and $\Delta_y = y_e - y$ as the EE offset from base in world frame. Then $\dot\theta$ induces:

$$\frac{\partial}{\partial\theta}\begin{bmatrix} x_e \\ y_e \\ z_e \end{bmatrix} = \begin{bmatrix} -\Delta_y \\ +\Delta_x \\ 0 \end{bmatrix}$$

This is $\hat{z} \times \Delta\boldsymbol{p}$ — the standard formula for angular velocity contribution.

**Assembled base Jacobian** (for $m = 3$, position-only task):

$$\mathbf{J}_b = \begin{bmatrix}
1 & 0 & -\Delta_y \\
0 & 1 & +\Delta_x \\
0 & 0 & 0
\end{bmatrix}$$

Note the third row is zero because base motion is planar — it cannot change the EE height.
Also note $\mathbf{J}_b$ depends on $\theta$ and the current arm configuration (through $\Delta_x$, $\Delta_y$).

---

### 7. The Arm Jacobian $\mathbf{J}_a$ — Geometric Method

The geometric Jacobian is computed joint by joint. For each revolute joint $i$ with axis
$\hat{\boldsymbol{z}}_i$ and origin $\boldsymbol{o}_i$, the column of the positional Jacobian is:

$$\mathbf{J}_{a,i} = \hat{\boldsymbol{z}}_i \times (\boldsymbol{x}_e - \boldsymbol{o}_i)$$

All vectors are expressed in the **world frame**, making the cross products straightforward.

**Joint 1 (shoulder pan, axis $= \mathbf{R}(\theta)\hat{z}$):**

$$\hat{\boldsymbol{z}}_1 = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}, \qquad
\boldsymbol{o}_1 = \begin{bmatrix} x \\ y \\ h + l_0 \end{bmatrix} \quad\text{(shoulder position in world)}$$

Define $\boldsymbol{r}_1 = \boldsymbol{x}_e - \boldsymbol{o}_1 = [x_e - x,\; y_e - y,\; z_e - (h+l_0)]^\top$.

$$\mathbf{J}_{a,1} = \hat{z} \times \boldsymbol{r}_1 = \begin{bmatrix} -(y_e - y) \\ x_e - x \\ 0 \end{bmatrix}$$

**Joint 2 (shoulder tilt, axis $= \mathbf{R}(\theta)[-\sin q_1, \cos q_1, 0]^\top$):**

Joint 2 rotates about the $y$-axis of frame $\{1\}$, which in the world frame is:

$$\hat{\boldsymbol{z}}_2 = \mathbf{R}(\theta)\begin{bmatrix} -s_1 \\ c_1 \\ 0 \end{bmatrix} = \begin{bmatrix} -c\theta\, s_1 - s\theta\, c_1 \\ -s\theta\, s_1 + c\theta\, c_1 \\ 0 \end{bmatrix}$$

Wait — more carefully: from the DH table, Joint 2's axis is the $z$-axis of frame $\{1\}$.
From $\mathbf{T}_S^1$, frame $\{1\}$'s $z$-axis in the shoulder frame is $[-s_1, c_1, 0]^\top$
(the third column of the rotation part). In the world frame:

$$\hat{\boldsymbol{z}}_2 = \mathbf{R}(\theta)\begin{bmatrix} -s_1 \\ c_1 \\ 0 \end{bmatrix}$$

Let $\boldsymbol{o}_2 = \boldsymbol{o}_1$ (joint 2 is co-located with joint 1, offset only by $d_1 = l_0$ along $z$ which is already included in $\boldsymbol{o}_1$). Then:

$$\mathbf{J}_{a,2} = \hat{\boldsymbol{z}}_2 \times (\boldsymbol{x}_e - \boldsymbol{o}_2)$$

**Joint 3 (elbow, axis aligned with joint 2's axis):**

$$\hat{\boldsymbol{z}}_3 = \hat{\boldsymbol{z}}_2 \quad \text{(parallel axes for joints 2 and 3)}$$

$$\boldsymbol{o}_3 = \boldsymbol{o}_1 + l_1 \mathbf{R}(\theta)\begin{bmatrix} c_1 c_2 \\ s_1 c_2 \\ -s_2 \end{bmatrix}$$

$$\mathbf{J}_{a,3} = \hat{\boldsymbol{z}}_3 \times (\boldsymbol{x}_e - \boldsymbol{o}_3)$$

**Assembled arm Jacobian:**

$$\mathbf{J}_a = \begin{bmatrix} \mathbf{J}_{a,1} & \mathbf{J}_{a,2} & \mathbf{J}_{a,3} \end{bmatrix} \in \mathbb{R}^{3 \times 3}$$

**Full Jacobian:**

$$\boxed{\mathbf{J} = \begin{bmatrix} \mathbf{J}_b & \mathbf{J}_a \end{bmatrix} \in \mathbb{R}^{3 \times 6}}$$

With $m = 3$ and $n = 6$: the system has 3 degrees of redundancy for position-only control.

---

### 8. The Nonholonomic Constraint — A Velocity Restriction That Changes Everything

The differential drive cannot move sideways. In the body frame, the velocity in the
$y$-body direction must be zero at all times. This is expressed as a **Pfaffian constraint**:

$$\boldsymbol{a}(\boldsymbol{q})^\top \dot{\boldsymbol{q}} = 0$$

For the lateral velocity constraint, in terms of the generalised velocities $[\dot{x}, \dot{y}, \dot\theta]$:

$$\boxed{-\sin\theta\;\dot{x} + \cos\theta\;\dot{y} = 0}$$

This says: the component of platform velocity perpendicular to the heading direction is always zero. In matrix form:

$$\underbrace{\begin{bmatrix} -s\theta & c\theta & 0 & 0 & 0 & 0 \end{bmatrix}}_{\boldsymbol{a}^\top(\boldsymbol{q})} \dot{\boldsymbol{q}} = 0$$

**This constraint is nonholonomic** — it cannot be integrated to a position-level constraint
$g(\boldsymbol{q}) = 0$. The platform *can* reach any $(x, y, \theta)$ by manoeuvring, but *cannot*
instantaneously move in the $y$-body direction. It is a kinematic constraint on velocity, not position.

**Admissible velocities** for the differential drive are parameterised by two inputs:

$$\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot\theta \end{bmatrix} = v\begin{bmatrix} c\theta \\ s\theta \\ 0 \end{bmatrix} + \dot\theta\begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} = \mathbf{G}(\theta)\begin{bmatrix} v \\ \dot\theta \end{bmatrix}$$

where $\mathbf{G}(\theta) = \begin{bmatrix} c\theta & 0 \\ s\theta & 0 \\ 0 & 1 \end{bmatrix}$ is the
**input matrix** (columns are the motion directions the platform can actually take).

**Connecting to wheel velocities.** Let $\omega_R, \omega_L$ be the right and left wheel angular velocities. Then:

$$v = \frac{r}{2}(\omega_R + \omega_L), \qquad \dot\theta = \frac{r}{2d}(\omega_R - \omega_L)$$

The actual inputs are $[\omega_R, \omega_L]^\top$, but we typically work with $[v, \dot\theta]^\top$.

---

## Part IV — Velocity Transformation and Manipulability

---

### 9. Velocity Transformation — The Map From Inputs to Task Space

The velocity relationship for the full system is:

$$\dot{\boldsymbol{x}}_e = \mathbf{J}(\boldsymbol{q})\,\dot{\boldsymbol{q}} = \mathbf{J}_b\,\dot{\boldsymbol{q}}_b + \mathbf{J}_a\,\dot{\boldsymbol{q}}_a$$

But because of the nonholonomic constraint, $\dot{\boldsymbol{q}}_b$ is restricted to the column space of
$\mathbf{G}(\theta)$: $\dot{\boldsymbol{q}}_b = \mathbf{G}(\theta)\begin{bmatrix} v \\ \dot\theta \end{bmatrix}$.

Define the **reduced generalised velocity** $\boldsymbol{u} = [v,\; \dot\theta,\; \dot{q}_1,\; \dot{q}_2,\; \dot{q}_3]^\top \in \mathbb{R}^5$.

The velocity transformation becomes:

$$\dot{\boldsymbol{x}}_e = \mathbf{J}(\boldsymbol{q})\begin{bmatrix} \mathbf{G}(\theta) & \mathbf{0} \\ \mathbf{0} & \mathbf{I}_3 \end{bmatrix}\boldsymbol{u} = \underbrace{\mathbf{J}_b\,\mathbf{G}(\theta)}_{\tilde{\mathbf{J}}_b} \begin{bmatrix} v \\ \dot\theta \end{bmatrix} + \mathbf{J}_a\,\dot{\boldsymbol{q}}_a$$

$$= \underbrace{\begin{bmatrix} \tilde{\mathbf{J}}_b & \mathbf{J}_a \end{bmatrix}}_{\tilde{\mathbf{J}}(\boldsymbol{q})} \boldsymbol{u}$$

where $\tilde{\mathbf{J}} \in \mathbb{R}^{3 \times 5}$ is the **reduced Jacobian** after incorporating the nonholonomic constraint.

**Computing $\tilde{\mathbf{J}}_b = \mathbf{J}_b\,\mathbf{G}(\theta)$:**

$$\tilde{\mathbf{J}}_b = \begin{bmatrix}
1 & 0 & -\Delta_y \\
0 & 1 & \Delta_x \\
0 & 0 & 0
\end{bmatrix}
\begin{bmatrix} c\theta & 0 \\ s\theta & 0 \\ 0 & 1 \end{bmatrix}
= \begin{bmatrix}
c\theta & -\Delta_y \\
s\theta & \Delta_x \\
0 & 0
\end{bmatrix}$$

This has a clean interpretation: the first column ($[c\theta, s\theta, 0]^\top$) is the EE velocity from pure forward drive; the second column ($[-\Delta_y, \Delta_x, 0]^\top$) is the EE velocity from pure rotation.

---

### 10. The Velocity Transmission Ratio (VTR)

The **Velocity Transmission Ratio** (also called velocity ratio or velocity transformation factor)
quantifies how efficiently a unit velocity in the input space produces velocity in the task space.
It is a scalar measure that varies with configuration and task-space direction.

**For a single joint $i$**, the VTR in direction $\hat{\boldsymbol{d}} \in \mathbb{R}^m$ (unit vector in task space) is:

$$\text{VTR}_i(\hat{\boldsymbol{d}}) = |\hat{\boldsymbol{d}}^\top \mathbf{J}_{:,i}|$$

This measures how much of joint $i$'s velocity reaches the end-effector in direction $\hat{\boldsymbol{d}}$.
A high VTR means joint $i$ is effective at moving the EE in that direction.

**The VTR ellipsoid.** For the full system, the set of EE velocities achievable by unit-norm
input velocity $\|\boldsymbol{u}\| = 1$ forms an ellipsoid in task space:

$$\mathcal{E} = \{\tilde{\mathbf{J}}\,\boldsymbol{u} \mid \|\boldsymbol{u}\| \leq 1\} = \{\boldsymbol{v} \in \mathbb{R}^m \mid \boldsymbol{v}^\top (\tilde{\mathbf{J}}\tilde{\mathbf{J}}^\top)^{-1}\boldsymbol{v} \leq 1\}$$

The shape of this ellipsoid is determined by the singular value decomposition:

$$\tilde{\mathbf{J}} = \mathbf{U}\,\mathbf{\Sigma}\,\mathbf{V}^\top$$

where $\mathbf{U} \in \mathbb{R}^{m \times m}$ contains the task-space principal directions,
$\mathbf{\Sigma} = \text{diag}(\sigma_1, \ldots, \sigma_m)$ with $\sigma_1 \geq \sigma_2 \geq \cdots \geq \sigma_m \geq 0$,
and $\mathbf{V} \in \mathbb{R}^{5 \times 5}$ contains the input-space principal directions.

The ellipsoid semi-axes are $\sigma_1, \sigma_2, \sigma_3$ along directions $\boldsymbol{u}_1, \boldsymbol{u}_2, \boldsymbol{u}_3$
(columns of $\mathbf{U}$).

**Interpretation:**

- In direction $\boldsymbol{u}_1$: a unit input velocity produces EE speed $\sigma_1$ (most efficient direction)
- In direction $\boldsymbol{u}_3$: a unit input velocity produces EE speed $\sigma_3$ (least efficient direction)
- When $\sigma_3 \to 0$: the ellipsoid flattens to a disk/line — the arm cannot move the EE in
  direction $\boldsymbol{u}_3$ regardless of how fast the joints move. **This is a singularity.**

**VTR for individual axes.** For axis-aligned task directions (e.g., pure $x$-motion):

$$\text{VTR}_x = \sqrt{e_x^\top \tilde{\mathbf{J}}\tilde{\mathbf{J}}^\top e_x} = \sqrt{\sum_i \tilde{J}_{1i}^2}$$

This is simply the norm of the first row of $\tilde{\mathbf{J}}$.

---

### 11. Manipulability Measures — A Full Survey

Manipulability is a scalar summary of the velocity ellipsoid. Multiple measures exist,
each emphasising different aspects.

#### 11.1 Yoshikawa Manipulability Index

The most widely used measure (Yoshikawa, 1985):

$$\boxed{w(\boldsymbol{q}) = \sqrt{\det\!\left(\tilde{\mathbf{J}}(\boldsymbol{q})\,\tilde{\mathbf{J}}(\boldsymbol{q})^\top\right)}}$$

Since $\tilde{\mathbf{J}}\tilde{\mathbf{J}}^\top = \mathbf{U}\mathbf{\Sigma}^2\mathbf{U}^\top$:

$$w = \sqrt{\prod_{i=1}^m \sigma_i^2} = \prod_{i=1}^m \sigma_i = \sigma_1\,\sigma_2\,\sigma_3$$

**Properties:**
- $w = 0$ iff the system is at a singularity (at least one $\sigma_i = 0$)
- $w$ is proportional to the volume of the manipulability ellipsoid
- Dimensionally: units of $(\text{m/s per rad/s})^m$ — not normalised

**Gradient of Yoshikawa manipulability** (needed for classical null-space control):

$$\nabla_{\boldsymbol{q}_a} w = w \cdot \text{tr}\!\left[(\tilde{\mathbf{J}}\tilde{\mathbf{J}}^\top)^{-1}\frac{\partial(\tilde{\mathbf{J}}\tilde{\mathbf{J}}^\top)}{\partial q_k}\right] \cdot \frac{1}{2}$$

Using the matrix identity $\nabla_{q_k}\det(\mathbf{A}) = \det(\mathbf{A})\text{tr}(\mathbf{A}^{-1}\nabla_{q_k}\mathbf{A})$:

$$\frac{\partial w}{\partial q_k} = \frac{w}{2}\,\text{tr}\!\left[(\tilde{\mathbf{J}}\tilde{\mathbf{J}}^\top)^{-1} \frac{\partial(\tilde{\mathbf{J}}\tilde{\mathbf{J}}^\top)}{\partial q_k}\right]$$

In practice, this is computed numerically via finite differences or via the formula involving the arm Jacobian's derivatives.

#### 11.2 Condition Number Manipulability

$$\kappa(\boldsymbol{q}) = \frac{\sigma_1}{\sigma_m}$$

**Properties:**
- $\kappa = 1$ means the ellipsoid is a sphere — perfectly isotropic (best possible)
- $\kappa \to \infty$ at singularities
- Not dependent on the overall "size" of the ellipsoid, only its shape
- More numerically robust than Yoshikawa for control purposes

The **inverse condition number** $1/\kappa \in [0, 1]$ is easier to maximise:

$$\mu(\boldsymbol{q}) = \frac{\sigma_m}{\sigma_1} \in [0, 1]$$

#### 11.3 Minimum Singular Value

$$w_\text{min}(\boldsymbol{q}) = \sigma_m(\tilde{\mathbf{J}})$$

This is the most conservative measure — it captures the worst-case velocity transmission.
Moving away from zero ensures the arm can always move the EE in at least the minimum amount
in every direction. Used when you specifically want to avoid near-singular configurations.

#### 11.4 Comparison of Measures for Your System

| Situation | Yoshikawa $w$ | Condition $1/\kappa$ | Min singular $\sigma_m$ |
|---|---|---|---|
| Optimal (near elbow-90°) | High | Near 1 | High |
| Fully extended | Near 0 | Near 0 | Near 0 |
| Elbow-aligned (one direction lost) | Low | Low | Near 0 |
| Large $\sigma_1$, small $\sigma_3$ | Can be moderate | Near 0 | Near 0 |

For your research, the **Yoshikawa index** is the standard choice — it appears in most related work and has a clear geometric interpretation (volume of ellipsoid). The RL policy is evaluated on its ability to maintain or improve $w$.

#### 11.5 Extended Manipulability for the Mobile Manipulator

When the base is mobile, the manipulability should account for base motion. Using $\tilde{\mathbf{J}}$ instead of $\mathbf{J}_a$ gives the **whole-body manipulability**:

$$w_\text{WB}(\boldsymbol{q}) = \sqrt{\det(\tilde{\mathbf{J}}\tilde{\mathbf{J}}^\top)}$$

This is always $\geq w_\text{arm}$ because base motion adds more reachable velocity directions.
However, it is not the most useful metric if the base is slow (e.g., heavy platform) — in that
case, a weighted manipulability using $\mathbf{W}_{\boldsymbol{u}} = \text{diag}(w_v, w_{\dot\theta}, w_{\dot{q}_1}, w_{\dot{q}_2}, w_{\dot{q}_3})$ accounts for the relative speed of each DOF:

$$w_\text{weighted} = \sqrt{\det(\tilde{\mathbf{J}}\,\mathbf{W}_{\boldsymbol{u}}^{-1}\,\tilde{\mathbf{J}}^\top)}$$

---

### 12. Manipulability Ellipsoid — Detailed Geometric Picture

Let's visualise exactly what the manipulability ellipsoid represents for your 3D system.

The SVD of $\tilde{\mathbf{J}}$ gives $\tilde{\mathbf{J}} = \mathbf{U}\mathbf{\Sigma}\mathbf{V}^\top$. Consider two spaces:

**Input ellipsoid** in $\mathbb{R}^5$ (the $\boldsymbol{u}$-space): the unit ball $\|\boldsymbol{u}\|^2 \leq 1$.

**Output ellipsoid** in $\mathbb{R}^3$ (task space): the image of the input unit ball under $\tilde{\mathbf{J}}$.

The output ellipsoid has:
- Principal axes aligned with the columns of $\mathbf{U}$: $\boldsymbol{u}_1, \boldsymbol{u}_2, \boldsymbol{u}_3$
- Semi-axis lengths: $\sigma_1, \sigma_2, \sigma_3$

The arm is most dexterous in direction $\boldsymbol{u}_1$ (longest axis) and least dexterous in $\boldsymbol{u}_3$ (shortest axis).

**Concrete example.** Suppose for some configuration:
$$\sigma_1 = 0.8\,\text{m/s/(rad/s)}, \quad \sigma_2 = 0.5, \quad \sigma_3 = 0.1$$

The system can move the EE at up to 0.8 m/s in direction $\boldsymbol{u}_1$ with unit input, but only 0.1 m/s in direction $\boldsymbol{u}_3$. The Yoshikawa index is $w = 0.8 \times 0.5 \times 0.1 = 0.04$. If a trajectory requires high velocity in direction $\boldsymbol{u}_3$, the controller will demand very large joint velocities — this is near-singular behaviour.

**What good manipulability looks like.** For the 3R arm in the PUMA configuration, maximum manipulability occurs near the "elbow-up" configurations where $q_2 \approx -\pi/3$ and $q_3 \approx \pi/2$ — the arm makes roughly a Z-shape. Avoid: arm fully extended ($q_2 \approx 0, q_3 \approx 0$) or folded back on itself.

---

## Part V — The Null Space in Full

---

### 13. Null Space of the Reduced Jacobian

Working with the reduced Jacobian $\tilde{\mathbf{J}} \in \mathbb{R}^{3 \times 5}$ (the nonholonomically constrained system):

**Null space definition:**

$$\mathcal{N}(\tilde{\mathbf{J}}) = \{\boldsymbol{u} \in \mathbb{R}^5 \mid \tilde{\mathbf{J}}\,\boldsymbol{u} = \boldsymbol{0}\}$$

**Dimension:**

$$\dim(\mathcal{N}(\tilde{\mathbf{J}})) = 5 - \text{rank}(\tilde{\mathbf{J}}) = 5 - 3 = 2$$

(assuming full rank, i.e., away from singularities)

**What null-space motions mean physically.** Any $\boldsymbol{u} \in \mathcal{N}(\tilde{\mathbf{J}})$ produces
zero end-effector velocity. This means you can simultaneously drive the base (forward/rotation)
and move the arm joints in a coordinated way such that the end-effector stays completely still.
These motions reconfigure the system — changing the arm posture, platform pose — without disturbing the task.

**Examples of null-space motions for your system:**
- Driving the platform forward while extending the arm backward: EE stays fixed, the "body" changes shape
- Rotating the shoulder joint while rotating the platform in the opposite direction: EE stays fixed in world frame
- Changing $q_2$ and $q_3$ in opposite directions such that the EE height is maintained

These are the motions your RL policy exploits.

---

### 14. The Null-Space Projector for the Constrained System

The **null-space projector** for $\tilde{\mathbf{J}}$ maps any input velocity to its null-space component:

$$\mathbf{N}(\boldsymbol{q}) = \mathbf{I}_5 - \tilde{\mathbf{J}}^\dagger\,\tilde{\mathbf{J}} \in \mathbb{R}^{5 \times 5}$$

where the pseudoinverse (damped, for numerical stability) is:

$$\tilde{\mathbf{J}}^\dagger = \tilde{\mathbf{J}}^\top\left(\tilde{\mathbf{J}}\tilde{\mathbf{J}}^\top + \lambda\mathbf{I}_3\right)^{-1} \in \mathbb{R}^{5 \times 3}, \quad \lambda = 0.01$$

**Fundamental identity** (the core of all null-space control):

$$\tilde{\mathbf{J}}\,\mathbf{N} = \tilde{\mathbf{J}}(\mathbf{I}_5 - \tilde{\mathbf{J}}^\dagger\tilde{\mathbf{J}}) = \tilde{\mathbf{J}} - \underbrace{\tilde{\mathbf{J}}\tilde{\mathbf{J}}^\dagger\tilde{\mathbf{J}}}_{= \tilde{\mathbf{J}}} = \mathbf{0}$$

This is verified using the pseudoinverse property $\tilde{\mathbf{J}}\tilde{\mathbf{J}}^\dagger\tilde{\mathbf{J}} = \tilde{\mathbf{J}}$.

**Complete properties of $\mathbf{N}$:**

$$\mathbf{N}^2 = \mathbf{N} \quad (\text{idempotent}) \qquad \mathbf{N}^\top = \mathbf{N} \quad (\text{symmetric}) \qquad \tilde{\mathbf{J}}\mathbf{N} = \mathbf{0} \quad (\text{annihilating})$$

$$\text{rank}(\mathbf{N}) = 5 - 3 = 2 \qquad \text{range}(\mathbf{N}) = \mathcal{N}(\tilde{\mathbf{J}})$$

**The complementary projector** $\mathbf{P} = \mathbf{I}_5 - \mathbf{N} = \tilde{\mathbf{J}}^\dagger\tilde{\mathbf{J}}$
projects onto the row space of $\tilde{\mathbf{J}}$:

$$\mathbf{P} + \mathbf{N} = \mathbf{I}_5 \qquad \text{(partition of identity)}$$

Any input velocity $\boldsymbol{u}$ is uniquely decomposed as:

$$\boldsymbol{u} = \underbrace{\mathbf{P}\,\boldsymbol{u}}_{\text{task-space component}} + \underbrace{\mathbf{N}\,\boldsymbol{u}}_{\text{null-space component}}$$

The task-space component is the minimum-norm part that achieves the task; the null-space component is the residual that changes arm posture without moving the EE.

---

### 15. The Complete IK Solution With Null Space

For the velocity-level IK problem $\tilde{\mathbf{J}}\,\boldsymbol{u} = \dot{\boldsymbol{x}}_d$:

$$\boxed{\boldsymbol{u} = \underbrace{\tilde{\mathbf{J}}^\dagger\,\dot{\boldsymbol{x}}_d}_{\text{minimum-norm particular solution}} + \underbrace{(\mathbf{I}_5 - \tilde{\mathbf{J}}^\dagger\tilde{\mathbf{J}})\,\boldsymbol{u}_0}_{\text{null-space component for secondary task}}}$$

where $\boldsymbol{u}_0 \in \mathbb{R}^5$ is any vector — the projection $\mathbf{N}\boldsymbol{u}_0$ picks out its
null-space component. Different choices of $\boldsymbol{u}_0$ implement different secondary objectives.

**Recovering base velocities and joint velocities.** After computing $\boldsymbol{u}^* \in \mathbb{R}^5$:

$$\dot{\boldsymbol{q}}_b^* = \mathbf{G}(\theta)\begin{bmatrix} u_1^* \\ u_2^* \end{bmatrix}, \qquad \dot{\boldsymbol{q}}_a^* = \begin{bmatrix} u_3^*, u_4^*, u_5^* \end{bmatrix}^\top$$

Then the actual wheel velocities:

$$\omega_R = \frac{1}{r}(v^* + d\dot\theta^*), \qquad \omega_L = \frac{1}{r}(v^* - d\dot\theta^*)$$

---

### 16. The Nonholonomic Constraint in the Null-Space Framework

Here is a subtlety specific to mobile manipulators that is often glossed over: the nonholonomic
constraint and the null-space projector are *not* the same object, and they must be handled carefully.

**Two different constraint sets:**

1. **Task constraint**: $\tilde{\mathbf{J}}\,\boldsymbol{u} = \dot{\boldsymbol{x}}_d$ — the EE must track the desired trajectory.
   This lives in the **reduced** input space $\boldsymbol{u} \in \mathbb{R}^5$.

2. **Nonholonomic constraint**: $\boldsymbol{a}^\top \dot{\boldsymbol{q}} = 0$ — the platform cannot move sideways.
   This lives in the **full** configuration velocity space $\dot{\boldsymbol{q}} \in \mathbb{R}^6$.

We incorporated (2) into (1) by substituting $\dot{\boldsymbol{q}}_b = \mathbf{G}(\theta)\begin{bmatrix}v\\\dot\theta\end{bmatrix}$.
The reduced Jacobian $\tilde{\mathbf{J}}$ already "knows" the nonholonomic constraint.

**The augmented constraint matrix** (if you want to work in the full $\dot{\boldsymbol{q}} \in \mathbb{R}^6$ space and handle both constraints simultaneously):

$$\mathbf{A}_\text{aug} = \begin{bmatrix} \mathbf{J} \\ \boldsymbol{a}^\top \end{bmatrix} \in \mathbb{R}^{(m+1) \times n}$$

For $m = 3$, $n = 6$: $\mathbf{A}_\text{aug} \in \mathbb{R}^{4 \times 6}$.

**The extended null-space projector** in the full space:

$$\mathbf{N}_\text{aug} = \mathbf{I}_6 - \mathbf{A}_\text{aug}^\dagger\,\mathbf{A}_\text{aug} \in \mathbb{R}^{6 \times 6}$$

$$\dim(\mathcal{N}(\mathbf{A}_\text{aug})) = 6 - \text{rank}(\mathbf{A}_\text{aug}) = 6 - 4 = 2$$

Any velocity $\dot{\boldsymbol{q}} = \mathbf{N}_\text{aug}\,\boldsymbol{z}$ simultaneously satisfies both the task
constraint and the nonholonomic constraint. This is the most general form of the null-space projector for a mobile manipulator, and it is what appears in the theoretical formulation in your paper.

**Equivalence.** For motion planning purposes, using $\tilde{\mathbf{J}}$ and $\mathbf{N}$ in the reduced
space $\boldsymbol{u}$ is equivalent to using $\mathbf{A}_\text{aug}$ and $\mathbf{N}_\text{aug}$ in the full
space $\dot{\boldsymbol{q}}$, since $\mathbf{G}(\theta)$ maps the admissible velocities bijectively. The
reduced-space formulation is computationally cleaner.

---

### 17. Degrees of Redundancy — A Careful Count

Let's count precisely for your system under different task definitions:

| Task | $m$ | $n$ | Nonhol. constraints | Effective DOF | Null-space dim |
|---|---|---|---|---|---|
| EE position (3D) | 3 | 6 | 1 | 5 | 2 |
| EE position + heading | 4 | 6 | 1 | 5 | 1 |
| EE position + full orientation | 6 | 6 | 1 | 5 | −1 (over-constrained) |
| EE position (2D, flat surface) | 2 | 6 | 1 | 5 | 3 |

For your paper's 3D position task: the null space is **2-dimensional**, meaning the RL policy
must output a 5-dimensional $\boldsymbol{u}$ that is then projected onto a 2D subspace. In practice,
the policy outputs a 5-vector and the projection $\mathbf{N}\boldsymbol{u}_0$ picks the 2D null-space component.

---

## Part VI — Task-Priority Control

---

### 18. The Hierarchy of Tasks

A mobile manipulator often has multiple simultaneous objectives that must be prioritised:

1. **Primary task** (highest priority): End-effector tracking — must be achieved
2. **Secondary task**: Manipulability maximisation — achieved as much as possible
3. **Tertiary task**: Joint limit avoidance — achieved in remaining DOF

Task-priority control (Siciliano & Slotine, 1991) achieves this hierarchy through nested null-space projections.

**Two-task hierarchy.** Let the primary task Jacobian be $\tilde{\mathbf{J}}_1$ and the secondary task
Jacobian be $\tilde{\mathbf{J}}_2$. The combined controller:

$$\boldsymbol{u} = \tilde{\mathbf{J}}_1^\dagger\,\boldsymbol{e}_1 + (\mathbf{I} - \tilde{\mathbf{J}}_1^\dagger\tilde{\mathbf{J}}_1)\,\tilde{\mathbf{J}}_2^\dagger\,\boldsymbol{e}_2$$

The second term $(\mathbf{I} - \tilde{\mathbf{J}}_1^\dagger\tilde{\mathbf{J}}_1)\,\tilde{\mathbf{J}}_2^\dagger$ is the **augmented Jacobian** — it projects the secondary task solution into the null space of the primary task. The primary task is never disturbed by the secondary task.

**Three-task hierarchy:**

$$\boldsymbol{u} = \tilde{\mathbf{J}}_1^\dagger\,\boldsymbol{e}_1 + \mathbf{N}_1\tilde{\mathbf{J}}_2^\dagger\,\boldsymbol{e}_2 + \mathbf{N}_1\mathbf{N}_2^*\tilde{\mathbf{J}}_3^\dagger\,\boldsymbol{e}_3$$

where $\mathbf{N}_1 = \mathbf{I} - \tilde{\mathbf{J}}_1^\dagger\tilde{\mathbf{J}}_1$ and $\mathbf{N}_2^* = \mathbf{I} - (\tilde{\mathbf{J}}_2\mathbf{N}_1)^\dagger(\tilde{\mathbf{J}}_2\mathbf{N}_1)$.

**The key principle:** Each successive task is executed in the null space of all higher-priority tasks. Task $k$ can only use whatever DOF are not consumed by tasks $1, \ldots, k-1$.

**For your system with 2D null-space:** You can have at most one secondary task with one DOF (e.g., manipulability gradient is 1D). The second null-space dimension is what your RL policy uses.

---

### 19. CLIK for the Mobile Manipulator

Extending CLIK from a fixed arm to the mobile manipulator requires care because:
1. The Jacobian $\tilde{\mathbf{J}}$ depends on platform pose as well as arm configuration
2. The base moves via a velocity input, not direct position control
3. Numerical integration of the nonholonomic constraint must be done correctly

**The CLIK controller for the mobile manipulator:**

$$\boldsymbol{u}^* = \tilde{\mathbf{J}}^\dagger(\boldsymbol{q})\left(\dot{\boldsymbol{x}}_d + K_p\,\boldsymbol{e}\right) + \mathbf{N}(\boldsymbol{q})\,\pi_\theta(\boldsymbol{s}, z)$$

where:
- $\boldsymbol{e} = \boldsymbol{x}_d - \boldsymbol{x}_e$ is the task-space error
- $K_p > 0$ is the proportional gain
- $\pi_\theta$ is the RL policy outputting a null-space velocity in $\mathbb{R}^5$
- $\mathbf{N}(\boldsymbol{q})\,\pi_\theta$ is the projected null-space component

**Stability.** As in the fixed-base case, the task-space error dynamics under CLIK with the null-space RL term:

$$\dot{\boldsymbol{e}} = -K_p\,\tilde{\mathbf{J}}\tilde{\mathbf{J}}^\dagger\,\boldsymbol{e} + \underbrace{\tilde{\mathbf{J}}\mathbf{N}\pi_\theta}_{= \boldsymbol{0}}$$

$$\Rightarrow \|\boldsymbol{e}(t)\| \leq \|\boldsymbol{e}(0)\|\,e^{-K_p\sigma_\text{min}(\tilde{\mathbf{J}}\tilde{\mathbf{J}}^\dagger)t}$$

The null-space RL action has zero effect on EE convergence — regardless of what the policy does.

**Integration on the platform.** The platform pose is updated by integrating the nonholonomic model:

$$\begin{bmatrix} \dot{x} \\ \dot{y} \\ \dot\theta \end{bmatrix} = \begin{bmatrix} c\theta & 0 \\ s\theta & 0 \\ 0 & 1 \end{bmatrix}\begin{bmatrix} v^* \\ \dot\theta^* \end{bmatrix}$$

This must be integrated carefully (e.g., Runge-Kutta 4, not Euler for large $dt$) because the platform heading $\theta$ changes the meaning of forward velocity. A simple Euler integrator can accumulate significant errors for large $\dot\theta^*$.

---

## Part VII — Singularity Analysis for the Mobile Manipulator

---

### 20. Types of Singularities

The mobile manipulator can be singular in qualitatively different ways, and it is important to distinguish them.

**Arm singularities.** Configurations where $\text{rank}(\mathbf{J}_a) < m$:

1. **Shoulder singularity**: When $q_2 = 0$ or $q_2 = \pi$ — the elbow and shoulder are aligned,
   reducing the effective reach plane. $\sigma_\text{min}(\mathbf{J}_a) \to 0$.

2. **Elbow singularity**: When $q_3 = 0$ — arm fully extended. All three joints lie in a plane
   perpendicular to the $\hat{z}_2$ axis, losing one direction of motion.

3. **Wrist singularity**: Not present in 3R arm (requires 5+ DOF with spherical wrist).

**Task-space singularities.** Even if $\mathbf{J}_a$ is full rank, the full Jacobian $\tilde{\mathbf{J}}$
can lose rank if the base motion becomes perfectly aligned with arm motion (redundant columns).

**Nonholonomic singularities.** The differential drive loses the ability to turn on the spot
if one wheel loses contact. These are mechanical, not kinematic — we ignore them here.

**Effect on manipulability.** All three arm singularities drive $w \to 0$. The RL policy should
learn to avoid these configurations by using the null space to maintain the arm in the interior
of the workspace, away from boundary (arm-extended) and interior (arm-folded) singularities.

---

### 21. Manipulability Gradient for Null-Space Control

The classical null-space control law uses $\boldsymbol{u}_0 = \nabla_{\boldsymbol{u}} w$ to maximise
manipulability. Let's derive this explicitly for the mobile manipulator.

Since $w = \sqrt{\det(\tilde{\mathbf{J}}\tilde{\mathbf{J}}^\top)}$ and $\tilde{\mathbf{J}}$ depends on
$\boldsymbol{q}_a = [q_1, q_2, q_3]^\top$ (and also on $\theta$ through $\tilde{\mathbf{J}}_b$, but
this is treated as fixed at each step):

$$\frac{\partial w}{\partial q_k} = \frac{1}{2w} \cdot \frac{\partial}{\partial q_k}\det(\tilde{\mathbf{J}}\tilde{\mathbf{J}}^\top)$$

Using Jacobi's formula $\frac{d}{dt}\det(\mathbf{A}) = \det(\mathbf{A})\,\text{tr}\!\left(\mathbf{A}^{-1}\frac{d\mathbf{A}}{dt}\right)$:

$$\frac{\partial w}{\partial q_k} = \frac{w}{2}\,\text{tr}\!\left[(\tilde{\mathbf{J}}\tilde{\mathbf{J}}^\top)^{-1}\left(\frac{\partial \tilde{\mathbf{J}}}{\partial q_k}\tilde{\mathbf{J}}^\top + \tilde{\mathbf{J}}\frac{\partial \tilde{\mathbf{J}}^\top}{\partial q_k}\right)\right]$$

$$= w\,\text{tr}\!\left[(\tilde{\mathbf{J}}\tilde{\mathbf{J}}^\top)^{-1}\frac{\partial \tilde{\mathbf{J}}}{\partial q_k}\tilde{\mathbf{J}}^\top\right]$$

The null-space velocity is then $\boldsymbol{u}_0 = [0, 0, \partial w/\partial q_1, \partial w/\partial q_2, \partial w/\partial q_3]^\top$
(base components set to zero if the base is not used for redundancy resolution).

After null-space projection: $\boldsymbol{u}_\text{null} = \mathbf{N}\,\boldsymbol{u}_0$. This ensures the
manipulability-gradient term does not disturb the EE tracking.

**Why this gradient is insufficient.** The gradient $\nabla w$ is:
- A local, greedy direction (no lookahead)
- Single-objective (manipulability only, ignoring joint limits)
- Prone to local maxima of $w$ that are not globally optimal
- Cannot adapt to task phase (approach vs. hold)

This is precisely the gap your RL policy fills.

---

## Part VIII — Complete System Summary

---

### 22. All the Key Matrices — Quick Reference

For the system $\boldsymbol{q} = [x, y, \theta, q_1, q_2, q_3]^\top$, $\boldsymbol{u} = [v, \dot\theta, \dot{q}_1, \dot{q}_2, \dot{q}_3]^\top$:

**Nonholonomic input matrix:**
$$\mathbf{G}(\theta) = \begin{bmatrix} c\theta & 0 \\ s\theta & 0 \\ 0 & 1 \end{bmatrix} \in \mathbb{R}^{3 \times 2}$$

**Base Jacobian:**
$$\mathbf{J}_b = \begin{bmatrix} 1 & 0 & -(y_e - y) \\ 0 & 1 & (x_e - x) \\ 0 & 0 & 0 \end{bmatrix} \in \mathbb{R}^{3 \times 3}$$

**Reduced base Jacobian:**
$$\tilde{\mathbf{J}}_b = \mathbf{J}_b\mathbf{G}(\theta) = \begin{bmatrix} c\theta & -(y_e - y) \\ s\theta & (x_e - x) \\ 0 & 0 \end{bmatrix} \in \mathbb{R}^{3 \times 2}$$

**Arm Jacobian** (geometric, world frame):
$$\mathbf{J}_a = \begin{bmatrix} \hat{z}_1 \times (\boldsymbol{x}_e - \boldsymbol{o}_1) & \hat{z}_2 \times (\boldsymbol{x}_e - \boldsymbol{o}_2) & \hat{z}_2 \times (\boldsymbol{x}_e - \boldsymbol{o}_3) \end{bmatrix} \in \mathbb{R}^{3 \times 3}$$

**Reduced Jacobian (full system, nonholonomic constraint incorporated):**
$$\tilde{\mathbf{J}} = \begin{bmatrix} \tilde{\mathbf{J}}_b & \mathbf{J}_a \end{bmatrix} \in \mathbb{R}^{3 \times 5}$$

**Damped pseudoinverse:**
$$\tilde{\mathbf{J}}^\dagger = \tilde{\mathbf{J}}^\top(\tilde{\mathbf{J}}\tilde{\mathbf{J}}^\top + \lambda\mathbf{I}_3)^{-1} \in \mathbb{R}^{5 \times 3}$$

**Null-space projector:**
$$\mathbf{N} = \mathbf{I}_5 - \tilde{\mathbf{J}}^\dagger\tilde{\mathbf{J}} \in \mathbb{R}^{5 \times 5}, \qquad \text{rank}(\mathbf{N}) = 2$$

**Yoshikawa manipulability:**
$$w = \sqrt{\det(\tilde{\mathbf{J}}\tilde{\mathbf{J}}^\top)} = \sigma_1\sigma_2\sigma_3$$

**CLIK + RL combined controller:**
$$\boldsymbol{u}^* = \tilde{\mathbf{J}}^\dagger(\dot{\boldsymbol{x}}_d + K_p\boldsymbol{e}) + \mathbf{N}\,\pi_\theta(\boldsymbol{s}, z) \in \mathbb{R}^5$$

---

### 23. The Null-Space Geometry — Visual Summary

```
  ℝ⁵  (input velocity space, u)
  ┌─────────────────────────────────────────┐
  │                                         │
  │   Row space of J̃ (dim 3)               │
  │   ┌───────────────────┐                 │
  │   │ J̃† x_dot: task   │◄── P = J̃†J̃   │
  │   │ primary motion    │                 │
  │   └───────────────────┘                 │
  │                                         │
  │   Null space of J̃ (dim 2)              │
  │   ┌───────────────────┐                 │
  │   │ N π_θ: RL policy  │◄── N = I - P   │
  │   │ secondary motion  │                 │
  │   └───────────────────┘                 │
  │                                         │
  └─────────────────────────────────────────┘
                   │ J̃·u*
                   ▼
  ℝ³  (task space velocity, x_dot)
  ┌─────────────────────────────────────────┐
  │  x_dot = J̃ (J̃†ẋ_d) + J̃ (Nπ_θ)      │
  │        = ẋ_d         + 0               │
  │                                         │
  │  EE tracking is UNCHANGED by RL term    │
  └─────────────────────────────────────────┘
```

---

### 24. Configuration Design for Maximum Manipulability — Practical Guidance

Given your 3R PUMA arm and differential drive base, here is practical guidance on what configurations to aim for.

**Arm posture.** The elbow-up configuration ($q_2 \approx -60°$, $q_3 \approx 90°$) consistently gives the highest Yoshikawa manipulability for PUMA-type arms. The "L-shape" — arm making a roughly 90° bend — is the isotropic ideal. The RL policy should learn to steer toward and maintain this family of configurations.

**Platform placement.** The base should be positioned such that the target lies within the "golden zone" of the arm — roughly $0.4L_\text{tot}$ to $0.8L_\text{tot}$ from the shoulder, where $L_\text{tot} = l_1 + l_2$. Too close: arm folds (interior singularity). Too far: arm extends (boundary singularity).

**Coordinated base-arm motion.** The null space of dimension 2 means there is a one-parameter family (at any instant) of ways to move the base and arm together without moving the EE. The RL policy learns to use this one-parameter freedom simultaneously for manipulability and joint-limit avoidance — something a single analytical gradient cannot do.

**Joint limit margins.** For the 3R arm with typical limits $q_1 \in [-\pi, \pi]$, $q_2 \in [-\pi/2, \pi/2]$, $q_3 \in [-2\pi/3, 2\pi/3]$: the joint-limit potential function to avoid is:

$$H_\text{jlim}(\boldsymbol{q}_a) = -\sum_{k=1}^3 \left(\frac{q_k - \bar{q}_k}{q_k^\text{range}}\right)^4$$

where $\bar{q}_k = (q_k^\text{max} + q_k^\text{min})/2$ is the joint midpoint and $q_k^\text{range} = q_k^\text{max} - q_k^\text{min}$. Maximising $H_\text{jlim}$ keeps joints away from limits.

---

### 25. Formula Reference Sheet

| Symbol | Dimension | Description |
|---|---|---|
| $\boldsymbol{q} = [x, y, \theta, q_1, q_2, q_3]^\top$ | $\mathbb{R}^6$ | Full configuration vector |
| $\boldsymbol{u} = [v, \dot\theta, \dot{q}_1, \dot{q}_2, \dot{q}_3]^\top$ | $\mathbb{R}^5$ | Reduced input velocity (post-constraint) |
| $\boldsymbol{x}_e \in \mathbb{R}^3$ | — | End-effector position (world frame) |
| $\mathbf{J}_b \in \mathbb{R}^{3 \times 3}$ | — | Base Jacobian (unconstrained) |
| $\tilde{\mathbf{J}}_b = \mathbf{J}_b\mathbf{G} \in \mathbb{R}^{3 \times 2}$ | — | Base Jacobian (nonholonomic) |
| $\mathbf{J}_a \in \mathbb{R}^{3 \times 3}$ | — | Arm Jacobian |
| $\tilde{\mathbf{J}} = [\tilde{\mathbf{J}}_b \; \mathbf{J}_a] \in \mathbb{R}^{3 \times 5}$ | — | Reduced full-system Jacobian |
| $\tilde{\mathbf{J}}^\dagger = \tilde{\mathbf{J}}^\top(\tilde{\mathbf{J}}\tilde{\mathbf{J}}^\top + \lambda\mathbf{I})^{-1}$ | $\mathbb{R}^{5 \times 3}$ | Damped pseudoinverse |
| $\mathbf{N} = \mathbf{I}_5 - \tilde{\mathbf{J}}^\dagger\tilde{\mathbf{J}}$ | $\mathbb{R}^{5 \times 5}$, rank 2 | Null-space projector |
| $\mathbf{A}_\text{aug} = [\mathbf{J}^\top \; \boldsymbol{a}]^\top$ | $\mathbb{R}^{4 \times 6}$ | Augmented constraint matrix |
| $\mathbf{N}_\text{aug} = \mathbf{I}_6 - \mathbf{A}_\text{aug}^\dagger\mathbf{A}_\text{aug}$ | $\mathbb{R}^{6 \times 6}$, rank 2 | Extended null-space projector |
| $w = \sqrt{\det(\tilde{\mathbf{J}}\tilde{\mathbf{J}}^\top)}$ | scalar $\geq 0$ | Yoshikawa manipulability |
| $\kappa = \sigma_1/\sigma_m$ | scalar $\geq 1$ | Condition number |
| $\mu = \sigma_m/\sigma_1 \in [0,1]$ | scalar | Inverse condition number |
| $\boldsymbol{u}^* = \tilde{\mathbf{J}}^\dagger(\dot{\boldsymbol{x}}_d + K_p\boldsymbol{e}) + \mathbf{N}\pi_\theta$ | $\mathbb{R}^5$ | CLIK + RL combined controller |
| $\tilde{\mathbf{J}}\mathbf{N} = \mathbf{0}$ | — | Fundamental null-space identity |

---

*End of Primer. The next step after reading this document is to implement the full Jacobian computation in Python, verify $\tilde{\mathbf{J}}\mathbf{N} = \mathbf{0}$ numerically for 1000 random configurations, and plot the manipulability ellipsoid across a sweep of arm configurations to build geometric intuition about good vs. bad postures.*
