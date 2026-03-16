# Robotics Mathematics: Advanced Topics

> **Prerequisite**: The 2-week core roadmap (Days 1–14).
> **Audience**: Researchers targeting geometric control and Lie-group methods for redundant manipulators.
> **Scope**: Three deep-dive modules that bridge the core roadmap to research-level geometric robotics.

---

# Module A — Quaternion Algebra and SLERP

---

## A.1 Why Quaternions?

The 2-week roadmap established rotation matrices ($SO(3)$) and exponential coordinates as the primary representations. Quaternions offer a **complementary** representation with specific computational advantages:

- **4 numbers, 1 constraint** vs 9 numbers, 6 constraints (matrices) or 3 numbers with singularities (Euler angles)
- **Composition** is quaternion multiplication — 16 multiplies vs 27 for matrix multiplication
- **Interpolation** has a clean, closed-form geodesic (SLERP) — no need for log/exp
- **Double cover** of $SO(3)$ — topologically $S^3$, which is simply connected, eliminating certain path-planning ambiguities
- **Numerically stable** — renormalising a quaternion to unit length is trivial (divide by its norm); re-orthogonalising a rotation matrix is expensive (SVD or Gram-Schmidt)

The trade-off: quaternions obscure the Lie-algebraic structure that makes Jacobians, twists, and null-space projections clean. In practice, modern robotics systems often use quaternions for **state representation and integration** but switch to Lie algebra (exponential coordinates) for **control and optimisation**.

---

## A.2 Quaternion Fundamentals

### A.2.1 Definition

A **quaternion** is a 4-tuple:

$$q = q_0 + q_1 i + q_2 j + q_3 k = (s, \mathbf{v})$$

where $s = q_0$ is the **scalar part** and $\mathbf{v} = [q_1, q_2, q_3]^\top$ is the **vector part**. The basis elements satisfy the Hamilton relations:

$$i^2 = j^2 = k^2 = ijk = -1$$

$$ij = k, \quad jk = i, \quad ki = j$$

$$ji = -k, \quad kj = -i, \quad ik = -j$$

These relations encode the non-commutativity of 3D rotations at the algebraic level. The fact that $ij \neq ji$ is not a quirk — it is the quaternion manifestation of the same phenomenon as $R_x(90°)R_z(90°) \neq R_z(90°)R_x(90°)$.

### A.2.2 Quaternion Multiplication

Given $p = (p_0, \mathbf{p}_v)$ and $q = (q_0, \mathbf{q}_v)$:

$$pq = (p_0 q_0 - \mathbf{p}_v \cdot \mathbf{q}_v, \; p_0 \mathbf{q}_v + q_0 \mathbf{p}_v + \mathbf{p}_v \times \mathbf{q}_v)$$

The scalar part involves a dot product (measuring alignment). The vector part involves both scaling (the $p_0 \mathbf{q}_v$ and $q_0 \mathbf{p}_v$ terms) and a cross product (encoding rotational composition). The cross product term is what makes quaternion multiplication non-commutative.

### A.2.3 Conjugate, Norm, and Inverse

**Conjugate**: $q^* = (s, -\mathbf{v})$. Geometrically: same rotation axis, opposite angle.

**Norm**: $\|q\| = \sqrt{q_0^2 + q_1^2 + q_2^2 + q_3^2}$

**Inverse**: $q^{-1} = q^* / \|q\|^2$

For **unit quaternions** ($\|q\| = 1$): $q^{-1} = q^*$. This parallels $R^{-1} = R^\top$ for rotation matrices — inversion is trivially cheap.

### A.2.4 Unit Quaternions and Rotations

A **unit quaternion** represents a rotation via:

$$q = (\cos(\theta/2), \; \sin(\theta/2) \hat{\omega})$$

where $\hat{\omega}$ is the unit rotation axis and $\theta$ is the rotation angle. The set of all unit quaternions forms $S^3$ (the 3-sphere in $\mathbb{R}^4$).

**The rotation action**: to rotate a vector $\mathbf{v} \in \mathbb{R}^3$ by quaternion $q$, embed $\mathbf{v}$ as a pure quaternion $\bar{v} = (0, \mathbf{v})$, then:

$$v_{\text{rotated}} = q \bar{v} q^*$$

The result is a pure quaternion whose vector part is the rotated vector.

### A.2.5 The Double Cover

The quaternions $q$ and $-q$ represent the **same rotation**:

$$q \bar{v} q^* = (-q) \bar{v} (-q)^*$$

This is because negating both $q$ and $q^*$ cancels out in the sandwich product. Geometrically, $(\hat{\omega}, \theta)$ and $(-\hat{\omega}, \theta + 2\pi)$ or equivalently $(\hat{\omega}, \theta)$ and $(-\hat{\omega}, -(2\pi - \theta))$ produce the same rotation. The quaternion captures both with $q$ and $-q$.

This double cover has a topological consequence: $S^3$ is the **universal cover** of $SO(3)$. Every loop in $SO(3)$ that cannot be contracted to a point *can* be contracted in $S^3$. This is the mathematical content of the Dirac belt trick — a $2\pi$ rotation in $SO(3)$ is a non-contractible loop, but its lift to $S^3$ is only half a great circle, which *can* be extended to a contractible loop ($4\pi$ rotation = full great circle = contractible).

**Practical consequence**: When interpolating or tracking quaternion trajectories, you must ensure **quaternion consistency** — always pick the sign of $q$ that is "closer" to the previous quaternion (i.e., ensure $q_{\text{prev}} \cdot q_{\text{current}} \geq 0$). Otherwise the interpolation may take the long way around $S^3$.

### Questions — Section A.2

**QA.2.1** Compute the quaternion for a 90° rotation about the z-axis. Then compute the quaternion for a 90° rotation about the x-axis. Multiply them (in both orders) and convert back to axis-angle. Verify the results match those from multiplying $R_z(90°)R_x(90°)$ and $R_x(90°)R_z(90°)$.

**QA.2.2** Show that $q$ and $-q$ produce the same rotation by explicitly expanding $q\bar{v}q^*$ and $(-q)\bar{v}(-q)^*$ for an arbitrary pure quaternion $\bar{v}$.

**QA.2.3** Given $q = (\cos(\pi/6), \sin(\pi/6)[0, 0, 1]^\top)$, compute $q^*$ and verify that $qq^* = (1, 0, 0, 0)$ (the identity quaternion).

**QA.2.4** Derive the quaternion multiplication formula from the Hamilton relations. Start with $(p_0 + p_1 i + p_2 j + p_3 k)(q_0 + q_1 i + q_2 j + q_3 k)$, expand all 16 terms, and group into scalar and vector parts.

**QA.2.5** Why does the half-angle $\theta/2$ appear in the quaternion representation rather than the full angle $\theta$? (Hint: consider what happens under composition — if you compose two rotations by $\theta$, the quaternion angle adds as $\theta/2 + \theta/2 = \theta$, but what does this correspond to on $SO(3)$?)

---

## A.3 Quaternion $\leftrightarrow$ Rotation Matrix Conversions

### A.3.1 Quaternion $\rightarrow$ Matrix

Given unit quaternion $q = (q_0, q_1, q_2, q_3)$:

$$R = \begin{bmatrix} 1-2(q_2^2+q_3^2) & 2(q_1 q_2 - q_0 q_3) & 2(q_1 q_3 + q_0 q_2) \\ 2(q_1 q_2 + q_0 q_3) & 1-2(q_1^2+q_3^2) & 2(q_2 q_3 - q_0 q_1) \\ 2(q_1 q_3 - q_0 q_2) & 2(q_2 q_3 + q_0 q_1) & 1-2(q_1^2+q_2^2) \end{bmatrix}$$

This can be derived by expanding the sandwich product $q\bar{v}q^*$ in components.

### A.3.2 Matrix $\rightarrow$ Quaternion (Shepperd's Method)

The naive approach (compute $\theta$ from trace, then axis from $R - R^\top$) has numerical issues near $\theta = 0$ and $\theta = \pi$. **Shepperd's method** avoids these by choosing the largest diagonal element:

1. Compute: $t = \text{tr}(R) = R_{11} + R_{22} + R_{33}$
2. Find the largest of $\{t, R_{11}, R_{22}, R_{33}\}$
3. Use the corresponding formula:

If $t$ is largest:

$$s = 2\sqrt{1 + t}$$

$$q_0 = s/4$$

$$q_1 = (R_{32} - R_{23})/s$$

$$q_2 = (R_{13} - R_{31})/s$$

$$q_3 = (R_{21} - R_{12})/s$$

If $R_{11}$ is largest:

$$s = 2\sqrt{1 + R_{11} - R_{22} - R_{33}}$$

$$q_1 = s/4$$

$$q_0 = (R_{32} - R_{23})/s$$

$$q_2 = (R_{21} + R_{12})/s$$

$$q_3 = (R_{13} + R_{31})/s$$

(Analogous formulas for $R_{22}$ and $R_{33}$ largest.)

The key insight: each formula divides by a different quantity, so you always pick the one with the largest denominator, avoiding division-by-small-number instabilities.

### A.3.3 Quaternion $\leftrightarrow$ Axis-Angle

These are the most direct conversions:

**Axis-angle $\rightarrow$ Quaternion**:

$$q = (\cos(\theta/2), \; \sin(\theta/2) \hat{\omega})$$

**Quaternion $\rightarrow$ Axis-angle**:

$$\theta = 2 \arccos(q_0) \quad [\text{or equivalently: } \theta = 2 \operatorname{arctan2}(\|q_v\|, q_0)]$$

$$\hat{\omega} = q_v / \|q_v\| \quad [\text{undefined when } \theta = 0, \text{ i.e., } q_v = 0]$$

The arctan2 form is numerically preferable because arccos has poor sensitivity near $q_0 = \pm 1$.

### Questions — Section A.3

**QA.3.1** Convert the quaternion $q = (\sqrt{2}/2, 0, 0, \sqrt{2}/2)$ to a rotation matrix using the formula. Verify the result is $R_z(90°)$.

**QA.3.2** Take $R = R_x(180°)$. Convert to a quaternion using Shepperd's method. Why would the naive trace-based approach ($\theta = \arccos((\text{tr}(R)-1)/2)$) be problematic here?

**QA.3.3** Implement (mentally or on paper) the full Shepperd algorithm for an arbitrary $R$. Count the number of arithmetic operations. Compare to extracting axis-angle via the log map (Day 7). Which is cheaper? Which is more numerically robust?

---

## A.4 SLERP — Spherical Linear Interpolation

### A.4.1 The Problem

Given two orientations $q_0$ and $q_1$ (as unit quaternions), find the "shortest path" interpolation $q(t)$ for $t \in [0, 1]$ such that:
- $q(0) = q_0$
- $q(1) = q_1$
- The path is a **geodesic** on $S^3$ (constant angular velocity)

### A.4.2 Why Linear Interpolation Fails

The naive approach — LERP (linear interpolation):

$$q_{\text{lerp}}(t) = (1-t)q_0 + tq_1 \quad \leftarrow \text{NOT unit length!}$$

The result does not lie on $S^3$. You could renormalise:

$$q_{\text{nlerp}}(t) = \text{normalize}((1-t)q_0 + tq_1)$$

This is called **NLERP** (normalised linear interpolation). It does produce valid rotations, but the angular velocity is **not constant** — the interpolation speeds up in the middle and slows down near the endpoints. For many robotics applications (trajectory generation, motion planning), constant angular velocity is essential.

### A.4.3 The SLERP Formula

**SLERP** (Spherical Linear Interpolation) follows the great circle arc on $S^3$:

$$\text{SLERP}(q_0, q_1, t) = q_0 \cdot \frac{\sin((1-t)\Omega)}{\sin(\Omega)} + q_1 \cdot \frac{\sin(t\Omega)}{\sin(\Omega)}$$

where:

$$\Omega = \arccos(q_0 \cdot q_1)$$

Here $q_0 \cdot q_1$ is the 4D dot product. The angle $\Omega$ is the angle between the two quaternions on $S^3$ (which is **half** the rotation angle between the two orientations due to the double cover).

### A.4.4 Derivation from Geodesics on $S^3$

$S^3$ is a Riemannian manifold with constant positive curvature (it's a sphere). Geodesics on spheres are great circles. The parametric equation of a great circle through two points on a unit sphere is:

$$q(t) = q_0 \cdot \frac{\sin((1-t)\Omega)}{\sin(\Omega)} + q_1 \cdot \frac{\sin(t\Omega)}{\sin(\Omega)}$$

This is the spherical analogue of the linear combination $(1-t)a + tb$ on a flat space. The sin functions ensure $q(t)$ stays on $S^3$ with constant angular speed.

**Proof that $\|q(t)\| = 1$**:

$$\|q(t)\|^2 = \frac{\sin^2((1-t)\Omega)}{\sin^2(\Omega)} + \frac{\sin^2(t\Omega)}{\sin^2(\Omega)} + \frac{2 \cos(\Omega) \sin((1-t)\Omega)\sin(t\Omega)}{\sin^2(\Omega)}$$

Using the identity $\sin(A)\sin(B) = \frac{1}{2}[\cos(A-B) - \cos(A+B)]$ and the fact that $q_0 \cdot q_1 = \cos \Omega$, this simplifies to 1. The algebra is tedious but the geometric reason is simple: we're parameterising a great circle, which lies on the unit sphere by definition.

### A.4.5 Important Implementation Details

**1. Hemisphere check**: Before interpolating, ensure $q_0 \cdot q_1 \geq 0$. If not, negate one quaternion:

$$\text{if } q_0 \cdot q_1 < 0:$$

$$q_1 \leftarrow -q_1 \quad \text{(same rotation, opposite hemisphere)}$$

$$\Omega \leftarrow \arccos(-(q_0 \cdot q_1)) \quad \text{[now takes the short path]}$$

Without this, SLERP may interpolate the "long way around" (rotating 270° instead of 90°, for example).

**2. Small angle fallback**: When $\Omega \approx 0$ ($q_0 \approx q_1$), $\sin \Omega \approx 0$ and the formula degenerates. Fall back to NLERP:

$$\text{if } \Omega < \varepsilon:$$

$$\text{return normalize}((1-t)q_0 + tq_1)$$

For small angles, NLERP and SLERP are virtually identical, so this is safe.

**3. Constant angular velocity**: SLERP traverses the geodesic at constant speed. The instantaneous angular velocity at any $t$ is:

$$|d\theta/dt| = 2\Omega / 1 = 2\Omega \quad \text{(constant)}$$

This makes SLERP directly suitable for trajectory generation where you want smooth, predictable angular motion.

### A.4.6 SLERP via the Exponential Map

There is an equivalent formulation using the group structure:

$$\text{SLERP}(q_0, q_1, t) = q_0 \cdot (q_0^{-1} q_1)^t$$

where $q^t$ means "raise the quaternion to the power $t$". For a unit quaternion $q = (\cos \alpha, \sin \alpha \cdot \hat{\omega})$:

$$q^t = (\cos(t\alpha), \sin(t\alpha) \cdot \hat{\omega})$$

**Interpretation**: $q_0^{-1}q_1$ is the "difference rotation" from $q_0$ to $q_1$. Taking it to the power $t$ scales that rotation to fraction $t$. Then left-multiplying by $q_0$ applies this partial rotation starting from $q_0$.

This is exactly analogous to the Lie group interpolation:

$$R(t) = R_0 \cdot \exp(t \cdot \log(R_0^\top R_1))$$

In fact, SLERP on quaternions and geodesic interpolation on $SO(3)$ via exp/log produce **identical rotation trajectories**. They are two computational paths to the same geometric object.

### A.4.7 SLERP vs Lie Group Interpolation

| Aspect | Quaternion SLERP | $SO(3)$ exp/log |
|---|---|---|
| Formula | $q_0(q_0^{-1}q_1)^t$ | $R_0 \exp(t \log(R_0^\top R_1))$ |
| Computational cost | ~20 multiplies + 1 arccos + 2 sin | Rodrigues log + exp (~50 multiplies) |
| Singularity | $\Omega \to 0$ (trivial fallback) | $\theta \to 0$ or $\pi$ (needs careful handling) |
| Double cover | Must handle $q$ vs $-q$ | No ambiguity |
| Extends to $SE(3)$ | Not natural | Directly via $\mathfrak{se}(3)$ |
| Control/Jacobian use | Must convert to axis-angle | Native |

**Bottom line**: Use SLERP for interpolation and integration. Use exp/log for control laws and Jacobians.

### A.4.8 Beyond SLERP: SQUAD for Multi-Point Interpolation

SLERP interpolates between two orientations. For a sequence of orientations $q_0, q_1, \ldots, q_n$, you need smooth transitions at the knot points. **SQUAD** (Spherical Quadrangle interpolation) provides $C^1$-continuous (continuous angular velocity) interpolation:

$$\text{SQUAD}(q_i, q_{i+1}, s_i, s_{i+1}, t) = \text{SLERP}(\text{SLERP}(q_i, q_{i+1}, t), \text{SLERP}(s_i, s_{i+1}, t), 2t(1-t))$$

where $s_i$ are intermediate control quaternions constructed from the neighbouring orientations:

$$s_i = q_i \cdot \exp\!\left(-\frac{\log(q_i^{-1}q_{i+1}) + \log(q_i^{-1}q_{i-1})}{4}\right)$$

This is the spherical analogue of cubic spline interpolation with Hermite-like tangent matching. The construction ensures that the angular velocity is continuous at each knot point, which is essential for smooth robot trajectories.

### Questions — Section A.4

**QA.4.1** Compute $\text{SLERP}(q_0, q_1, 0.5)$ where $q_0 = (1, 0, 0, 0)$ (identity) and $q_1 = (\sqrt{2}/2, 0, 0, \sqrt{2}/2)$ (90° about z). Verify the result corresponds to a 45° rotation about z.

**QA.4.2** Show that $\text{SLERP}(q_0, q_1, t) = q_0(q_0^{-1}q_1)^t$ is equivalent to the sin-based formula. (Hint: write $q_0^{-1}q_1$ in axis-angle form, take it to the power $t$, left-multiply by $q_0$, and simplify using trig identities.)

**QA.4.3** Explain why NLERP does not produce constant angular velocity. Consider three equally spaced parameter values $t = 0, 0.5, 1$ and compute the rotation angle at each for $q_0$ = identity, $q_1$ = 90° about z. Is the angle at $t = 0.5$ exactly 45°?

**QA.4.4** You are generating a trajectory for a robot end-effector that must pass through orientations $q_0, q_1, q_2$. Using piecewise SLERP (SLERP from $q_0$ to $q_1$, then SLERP from $q_1$ to $q_2$), is the angular velocity continuous at $q_1$? Why or why not? How does SQUAD fix this?

**QA.4.5** Prove that for unit quaternions, $\|\text{SLERP}(q_0, q_1, t)\| = 1$ for all $t \in [0,1]$. (Use the sin-based formula and trig identities.)

---

## A.5 Quaternion Angular Velocity and Integration

### A.5.1 Quaternion Kinematics Equation

The analogue of $\dot{R} = [\omega]_\times R$ for quaternions is:

$$\dot{q} = \tfrac{1}{2} q \otimes \bar{\omega}$$

where $\bar{\omega} = (0, \omega)$ is the body-frame angular velocity embedded as a pure quaternion, and $\otimes$ denotes quaternion multiplication. In matrix form:

$$\dot{q} = \tfrac{1}{2} \Omega(\omega) q$$

where $\Omega(\omega)$ is the $4 \times 4$ matrix:

$$\Omega(\omega) = \begin{bmatrix} 0 & -\omega_1 & -\omega_2 & -\omega_3 \\ \omega_1 & 0 & \omega_3 & -\omega_2 \\ \omega_2 & -\omega_3 & 0 & \omega_1 \\ \omega_3 & \omega_2 & -\omega_1 & 0 \end{bmatrix}$$

This is a **linear** ODE in $q$ (given $\omega$). This linearity is a significant advantage for integration — you can use standard integrators (RK4, etc.) directly, then renormalise.

### A.5.2 Integration and Unit Constraint

After each integration step, the quaternion may drift from unit length due to numerical error. The fix is trivial:

$$q \leftarrow q / \|q\|$$

Compare this to rotation matrices, where maintaining orthogonality requires SVD decomposition or iterative Gram-Schmidt — vastly more expensive. This is the primary reason many simulators (MuJoCo, Bullet, Drake) use quaternions for state representation.

### A.5.3 Quaternion Error for Control

The quaternion orientation error between desired $q_d$ and current $q$ is:

$$q_e = q_d \otimes q^{-1} = q_d \otimes q^*$$

The vector part of $q_e$ gives a 3D error signal:

$$e_{\text{quat}} = q_{e,\text{vec}} = \sin(\theta_e/2) \hat{\omega}_e$$

For small errors, $\sin(\theta_e/2) \approx \theta_e/2$, so $e_{\text{quat}} \approx \frac{1}{2} \theta_e \hat{\omega}_e$ — proportional to the axis-angle error. Many PD controllers on $SO(3)$ use this directly:

$$\tau = -K_p \cdot e_{\text{quat}} - K_d \cdot \omega_e$$

This is equivalent (for small errors) to the Lie-algebra controller $\tau = -K_p \cdot \text{vee}(\log(R_e)) - K_d \cdot \omega_e$ from Day 14, but with the quaternion form being computationally cheaper (no log map needed).

### Questions — Section A.5

**QA.5.1** Starting from $q = (1, 0, 0, 0)$ with constant body angular velocity $\omega = [0, 0, \pi]^\top$ (180°/s about z), integrate $\dot{q} = \frac{1}{2} q \otimes \bar{\omega}$ for $t = 1$ second using exact integration (hint: constant $\omega$ gives an analytical solution). What quaternion do you get? What rotation does it represent?

**QA.5.2** Show that $\|\dot{q}\|$ depends on $\|\omega\|$ but not on $q$ (for unit quaternions). What does this imply about the "speed" of traversal on $S^3$?

**QA.5.3** Compare the quaternion error $e_{\text{quat}} = (q_d q^*)_{\text{vec}}$ with the Lie algebra error $e_R = \text{vee}(\log(R_d R^\top))$ for a 10° error about the z-axis. Compute both numerically. By what factor do they differ?

---

# Module B — Lie Bracket and Adjoint Representation at the Algebra Level

---

## B.1 The Lie Bracket on $\mathfrak{so}(3)$

### B.1.1 Definition

The **Lie bracket** on $\mathfrak{so}(3)$ is the matrix commutator:

$$[A, B] = AB - BA$$

where $A, B \in \mathfrak{so}(3)$ ($3 \times 3$ skew-symmetric matrices).

**Fundamental fact**: If $A$ and $B$ are skew-symmetric, then $[A, B]$ is also skew-symmetric. The Lie bracket is a **closed operation** on $\mathfrak{so}(3)$ — it maps pairs of Lie algebra elements to a new Lie algebra element.

### B.1.2 The Bracket in Terms of Vectors

Recall the "hat" map ($\wedge$): $\mathbb{R}^3 \to \mathfrak{so}(3)$ that maps $\omega \mapsto [\omega]_\times$. The Lie bracket on $\mathfrak{so}(3)$ corresponds to the **cross product** on $\mathbb{R}^3$:

$$[[\alpha]_\times, [\beta]_\times] = [\alpha \times \beta]_\times$$

**Proof**: Expand $[\alpha]_\times [\beta]_\times - [\beta]_\times [\alpha]_\times$ using the identity $[a]_\times [b]_\times = ba^\top - (a \cdot b)I$, and verify the result equals $[\alpha \times \beta]_\times$. The algebra is instructive — do it once.

This is a profound connection: the Lie bracket structure of $\mathfrak{so}(3)$ is *exactly* the cross product structure of $\mathbb{R}^3$. The cross product is not just a computational trick — it is the Lie bracket of the rotation group, making $\mathbb{R}^3$ with the cross product isomorphic to $\mathfrak{so}(3)$ as a Lie algebra.

### B.1.3 Lie Bracket Axioms

The Lie bracket satisfies three axioms:

**Bilinearity**:

$$[\alpha A + \beta B, C] = \alpha[A,C] + \beta[B,C]$$

$$[A, \alpha B + \beta C] = \alpha[A,B] + \beta[A,C]$$

**Antisymmetry**:

$$[A, B] = -[B, A]$$

**Jacobi identity**:

$$[A, [B, C]] + [B, [C, A]] + [C, [A, B]] = 0$$

For $\mathfrak{so}(3)$ via the cross product, the Jacobi identity becomes:

$$\alpha \times (\beta \times \gamma) + \beta \times (\gamma \times \alpha) + \gamma \times (\alpha \times \beta) = 0$$

This is a classical vector identity (verify using the BAC-CAB rule). The Jacobi identity has deep physical consequences — it encodes the consistency of infinitesimal symmetry transformations and appears in angular momentum commutation relations in mechanics.

### B.1.4 Physical Meaning of the Lie Bracket

The Lie bracket $[A, B]$ measures the **failure of two infinitesimal transformations to commute**. More precisely, if you:

1. Flow along $A$ for time $\varepsilon$
2. Flow along $B$ for time $\varepsilon$
3. Flow along $-A$ for time $\varepsilon$
4. Flow along $-B$ for time $\varepsilon$

you do NOT return to the start. The gap is proportional to $\varepsilon^2 [A,B]$:

$$e^{\varepsilon A} e^{\varepsilon B} e^{-\varepsilon A} e^{-\varepsilon B} \approx I + \varepsilon^2 [A,B] + O(\varepsilon^3)$$

For rotations: if you rotate by small $\varepsilon$ about x, then by $\varepsilon$ about y, then by $-\varepsilon$ about x, then by $-\varepsilon$ about y, you end up with a small rotation about z (proportional to $\varepsilon^2$). This is because $[e_x, e_y] = e_z$ under the cross product — exactly matching the commutator of the corresponding skew-symmetric matrices.

**This is the geometric content of non-commutativity**: the Lie bracket tells you what new direction is generated by the commutator of two motions.

### Questions — Section B.1

**QB.1.1** Compute $[[e_1]_\times, [e_2]_\times]$ directly (matrix multiply and subtract). Verify the result equals $[e_3]_\times = [e_1 \times e_2]_\times$.

**QB.1.2** Verify the Jacobi identity for $\alpha = e_1$, $\beta = e_2$, $\gamma = e_3$ (i.e., the standard basis vectors) using the cross product form.

**QB.1.3** Compute $e^{\varepsilon [e_1]_\times} e^{\varepsilon [e_2]_\times} e^{-\varepsilon [e_1]_\times} e^{-\varepsilon [e_2]_\times}$ for $\varepsilon = 0.1$ using Rodrigues (or numerically). Show the result is close to $e^{\varepsilon^2 [e_3]_\times} = R_z(0.01)$. How close?

**QB.1.4** The Lie bracket of $\mathfrak{so}(2)$ ($2 \times 2$ skew-symmetric matrices) is always zero: any two elements commute. What does this say about 2D rotations? Why is 3D fundamentally different from 2D in this regard?

**QB.1.5** Consider two angular velocities $\omega_1$ and $\omega_2$ applied to a rigid body. The Lie bracket $[[\omega_1]_\times, [\omega_2]_\times] = [\omega_1 \times \omega_2]_\times$ represents a new angular velocity direction. Give a physical scenario where this "bracket-generated" direction matters (hint: think about controllability of underactuated systems).

---

## B.2 The Lie Bracket on $\mathfrak{se}(3)$

### B.2.1 Elements of $\mathfrak{se}(3)$

The Lie algebra of $SE(3)$ consists of $4 \times 4$ matrices:

$$[\hat{\xi}] = \begin{bmatrix} [\omega]_\times & v \\ 0^\top & 0 \end{bmatrix} \in \mathfrak{se}(3)$$

where $[\omega]_\times \in \mathfrak{so}(3)$ and $v \in \mathbb{R}^3$. Using the "vee" notation, $\xi = [v; \omega] \in \mathbb{R}^6$.

### B.2.2 The Bracket on $\mathfrak{se}(3)$

For $\xi_1 = [v_1; \omega_1]$ and $\xi_2 = [v_2; \omega_2]$:

$$[\hat{\xi}_1, \hat{\xi}_2] = \hat{\xi}_1 \hat{\xi}_2 - \hat{\xi}_2 \hat{\xi}_1$$

Computing this:

$$[\hat{\xi}_1, \hat{\xi}_2] = \begin{bmatrix} [\omega_1 \times \omega_2]_\times & \omega_1 \times v_2 - \omega_2 \times v_1 \\ 0^\top & 0 \end{bmatrix}$$

In vector form:

$$[\xi_1, \xi_2] = \begin{bmatrix} \omega_1 \times v_2 - \omega_2 \times v_1 \\ \omega_1 \times \omega_2 \end{bmatrix}$$

The angular part is the same as $\mathfrak{so}(3)$ — just the cross product of the angular velocities. The linear part involves the **coupling** between angular and linear components through cross products. This coupling is the hallmark of rigid body kinematics: rotation affects translation.

### B.2.3 Lie Bracket and Screw Geometry

The bracket of two twists has a beautiful screw-theoretic interpretation: it measures the rate of change of one screw as the body moves along the other. If $\xi_1$ is a joint screw and $\xi_2$ is another, $[\xi_1, \xi_2]$ tells you how joint 2's screw axis changes as joint 1 moves — which is exactly the information needed to construct Jacobians.

This connects to the **Lie derivative** in differential geometry: the Lie bracket of two vector fields measures how one field changes along the flow of the other.

### Questions — Section B.2

**QB.2.1** Compute the Lie bracket of $\xi_1 = [0,0,0, 0,0,1]^\top$ (pure rotation about z) and $\xi_2 = [1,0,0, 0,0,0]^\top$ (pure translation in x). Interpret the result geometrically — what new motion is generated?

**QB.2.2** Two revolute joints have twists $\xi_1 = [0,0,0, 0,0,1]^\top$ (z-rotation at origin) and $\xi_2 = [0,L,0, 0,0,1]^\top$ (z-rotation at $(L,0,0)$). Compute $[\xi_1, \xi_2]$. What does this bracket tell you about the kinematics?

**QB.2.3** Verify the Jacobi identity for three twists of your choice in $\mathfrak{se}(3)$.

**QB.2.4** Show that the Lie bracket of two pure translations is always zero. What does this mean physically?

---

## B.3 The Adjoint Representation

### B.3.1 Two Levels of Adjoint

There are **two distinct adjoint maps** in Lie group theory, and conflating them is a common source of confusion:

**1. The (big) Adjoint $\text{Ad}_g$**: maps the Lie algebra to itself via conjugation by a group element $g \in G$:

$$\text{Ad}_g(\xi) = g \xi g^{-1} \quad \text{(in matrix form)}$$

For $SO(3)$: $\text{Ad}_R([\omega]_\times) = R[\omega]_\times R^\top = [R\omega]_\times$. In vector form: $\text{Ad}_R(\omega) = R\omega$. This is just rotation of the angular velocity vector — physically, changing the frame in which the angular velocity is expressed.

For $SE(3)$: the $6 \times 6$ matrix we encountered on Day 13:

$$\text{Ad}_T = \begin{bmatrix} R & [p]_\times R \\ 0 & R \end{bmatrix}$$

**2. The (little) adjoint $\text{ad}_\xi$**: maps the Lie algebra to itself via the Lie bracket with a fixed algebra element:

$$\text{ad}_\xi(\eta) = [\xi, \eta] \quad \text{(the Lie bracket)}$$

For $\mathfrak{so}(3)$: $\text{ad}_\omega(\beta) = [\omega]_\times \beta = \omega \times \beta$. The little adjoint *is* the cross product / skew-symmetric matrix.

For $\mathfrak{se}(3)$: given $\xi = [v; \omega]$, the $6 \times 6$ matrix representation of $\text{ad}_\xi$ is:

$$\text{ad}_\xi = \begin{bmatrix} [\omega]_\times & [v]_\times \\ 0 & [\omega]_\times \end{bmatrix}$$

### B.3.2 The Fundamental Relationship

The big and little adjoints are connected by differentiation:

$$\text{ad}_\xi = \left.\frac{d}{dt}\right|_{t=0} \text{Ad}_{\exp(t\xi)}$$

The little adjoint is the **derivative of the big Adjoint at the identity**. This is a general Lie group fact: the Lie algebra "differentiates" the Lie group.

Equivalently, the matrix exponential connects them:

$$\text{Ad}_{\exp(\xi)} = \exp(\text{ad}_\xi) \quad \text{(as } 6 \times 6 \text{ matrices for } SE(3)\text{)}$$

### B.3.3 The Adjoint in Velocity Kinematics

The Space Jacobian column formula from Day 12:

$$J^s_i = \text{Ad}_{T_1 \cdots T_{i-1}} \xi_i$$

uses the big Adjoint. Each joint's screw is defined at the zero configuration, and the Adjoint "transports" it through the motions of preceding joints.

The **derivative** of the Jacobian — needed for acceleration-level kinematics and Hessian computations — involves the little adjoint (ad) because it captures how screws change infinitesimally as joints move.

### B.3.4 The BCH Formula and the Role of ad

The **Baker-Campbell-Hausdorff (BCH) formula** expresses the log of a product of exponentials:

$$\log(e^A e^B) = A + B + \tfrac{1}{2}[A,B] + \tfrac{1}{12}([A,[A,B]] - [B,[A,B]]) + \cdots$$

All higher-order terms are nested Lie brackets. This formula is fundamental to understanding how joint motions compose in the exponential coordinate framework.

For small motions (first order): $\log(e^A e^B) \approx A + B$. The Lie bracket correction $\frac{1}{2}[A,B]$ is the **leading-order non-commutativity term** — it quantifies how much the result deviates from simple addition.

In robotics, the BCH formula appears when:
- Composing small joint motions in task space
- Analysing the Jacobian of the POE formula
- Understanding geometric integration schemes (Lie group integrators)

### B.3.5 The ad Matrix and Rigid Body Dynamics

In rigid body dynamics on $SE(3)$, the equations of motion in body-frame coordinates are:

$$M \dot{V}_b = \text{ad}^*_{V_b} M V_b + F_b$$

where $M$ is the $6 \times 6$ spatial inertia matrix, $V_b$ is the body twist, $\text{ad}^*$ is the **co-adjoint** (dual of ad), and $F_b$ is the applied wrench. The $\text{ad}^*_{V_b} M V_b$ term contains Coriolis and centrifugal effects.

For $SO(3)$ alone, this reduces to Euler's equation:

$$I \dot{\omega} = (I\omega) \times \omega + \tau = -[\omega]_\times I\omega + \tau$$

The $[\omega]_\times$ matrix appearing here is precisely $\text{ad}_\omega$ — the little adjoint on $\mathfrak{so}(3)$. The Coriolis/centrifugal coupling in rigid body dynamics is a manifestation of the Lie algebra structure.

### Questions — Section B.3

**QB.3.1** For $R = R_z(\theta)$, compute $\text{Ad}_R(\omega)$ for $\omega = [1, 0, 0]^\top$. Verify the result is $R\omega = [\cos \theta, \sin \theta, 0]^\top$. Interpret: a body spinning about x in its own frame — what does the angular velocity look like in the world frame?

**QB.3.2** Compute the $6 \times 6$ $\text{ad}_\xi$ matrix for $\xi = [0, 0, 0, 0, 0, 1]^\top$ (pure z-rotation). Apply it to $\eta = [1, 0, 0, 0, 0, 0]^\top$ (x-translation). Verify the result matches $[\xi, \eta]$ computed via the bracket formula.

**QB.3.3** Verify the identity $\text{Ad}_{\exp(\xi)} = \exp(\text{ad}_\xi)$ for $\xi = [0, 0, 0, 0, 0, \pi/2]^\top$ (90° rotation about z). Compute both sides as $6 \times 6$ matrices.

**QB.3.4** In the BCH formula, compute the first three terms of $\log(e^A e^B)$ for $A = 0.1[e_1]_\times$ and $B = 0.1[e_2]_\times$ (small rotations about x and y). How significant is the bracket correction term relative to $A + B$?

**QB.3.5** Euler's equation $I\dot{\omega} = -[\omega]_\times I\omega + \tau$ can be written as $I\dot{\omega} = -\text{ad}^*_\omega(I\omega) + \tau$ where $\text{ad}^*_\omega(p) = [\omega]_\times p$ for $p \in \mathbb{R}^3$. Explain why this structure (the co-adjoint) arises from the non-commutativity of $SO(3)$. What would the equation look like if $SO(3)$ were commutative (i.e., if the Lie bracket were zero)?

---

# Module C — Contraction Theory and CLIK Convergence

---

## C.1 The Convergence Problem in CLIK

### C.1.1 Recap: CLIK on $SE(3)$

Closed-Loop Inverse Kinematics (CLIK) solves the inverse kinematics problem as a feedback control loop:

$$\dot{\theta} = J^+(θ) \cdot K \cdot \xi_e(\theta)$$

where:
- $\xi_e(\theta) = \text{vee}(\log(T_d \cdot T(\theta)^{-1}))$ is the task-space error in the Lie algebra
- $J^+(\theta)$ is the Jacobian pseudoinverse (or damped least-squares inverse)
- $K$ is a positive-definite gain matrix

This is an autonomous ODE on the joint space: $\dot{\theta} = f(\theta)$. The question is: **does it converge?** And if so, from what initial conditions? And how fast?

### C.1.2 Classical Lyapunov Analysis

The standard approach is to pick a Lyapunov function:

$$V(\theta) = \tfrac{1}{2} \|\xi_e(\theta)\|^2$$

and show $\dot{V} < 0$. This works locally (near the solution) but has limitations:
- The analysis is typically local — it guarantees convergence only in some neighbourhood
- The domain of convergence is hard to characterise precisely
- For redundant robots with null-space terms, the analysis becomes significantly more complex
- The approach gives an existence result ("it converges") but limited quantitative information about convergence rate or robustness

### C.1.3 What Contraction Theory Offers

Contraction theory provides:
- **Global** convergence guarantees (or explicit characterisation of the convergence region)
- **Exponential** convergence rates (not just asymptotic)
- **Compositional** analysis — complex systems can be analysed by composing contracting subsystems
- Natural handling of **time-varying** reference trajectories (not just fixed-point IK)
- **Robustness** guarantees — bounded perturbations produce bounded tracking errors

These are precisely the properties needed for a rigorous analysis of CLIK with null-space policies.

---

## C.2 Contraction Theory Foundations

### C.2.1 The Core Idea

Consider a nonlinear system:

$$\dot{x} = f(x, t)$$

Classical stability asks: does a trajectory converge to an **equilibrium point**? Contraction theory asks a different question: do **all trajectories converge to each other**?

A system is **contracting** if any two trajectories, regardless of initial conditions, converge to each other exponentially:

$$\|x_1(t) - x_2(t)\| \leq e^{-\beta t} \|x_1(0) - x_2(0)\|$$

where $\beta > 0$ is the **contraction rate**. The system "forgets" its initial conditions exponentially fast.

### C.2.2 The Contraction Condition

The system $\dot{x} = f(x, t)$ is contracting with rate $\beta$ if there exists a uniformly positive definite metric $M(x, t)$ such that the **generalised Jacobian** satisfies:

$$F = \frac{\dot{M} + (\partial f/\partial x)^\top M + M(\partial f/\partial x)}{2} \leq -\beta M$$

In the simplest case with $M = I$ (the identity metric):

$$\text{sym}(\partial f/\partial x) \leq -\beta I$$

where $\text{sym}(A) = (A + A^\top)/2$ is the symmetric part. This means: the symmetric part of the Jacobian of $f$ must be **uniformly negative definite**.

**Interpretation**: The Jacobian $\partial f/\partial x$ governs how nearby trajectories evolve relative to each other. If its symmetric part is negative definite everywhere, nearby trajectories converge — the "flow" is contracting in all directions.

### C.2.3 Why the Symmetric Part?

The Jacobian $\partial f/\partial x$ may not be symmetric. Its effect on the distance between nearby trajectories is governed by the virtual displacement $\delta x$:

$$\frac{d}{dt}(\delta x) = \frac{\partial f}{\partial x} \delta x$$

The rate of change of $\|\delta x\|^2$ is:

$$\frac{d}{dt} \|\delta x\|^2 = 2 \delta x^\top \frac{\partial f}{\partial x} \delta x = 2 \delta x^\top \text{sym}\!\left(\frac{\partial f}{\partial x}\right) \delta x$$

Only the symmetric part contributes (the antisymmetric part cancels in the quadratic form). So the symmetric part of the Jacobian is the "contraction-relevant" part.

### C.2.4 Contraction in a General Metric

The identity metric is often too restrictive. A **contraction metric** $M(x, t)$ (symmetric positive definite matrix field) defines a Riemannian distance:

$$\|\delta x\|_M = \sqrt{\delta x^\top M \delta x}$$

The system contracts in this metric if:

$$\dot{M} + (\partial f/\partial x)^\top M + M (\partial f/\partial x) \leq -2\beta M$$

Finding the right metric is the art of contraction analysis. For robotic systems, the **inertia matrix** of the robot or a task-space metric are natural candidates.

### C.2.5 Comparison with Lyapunov Theory

| Aspect | Lyapunov | Contraction |
|---|---|---|
| Analyses | Convergence to a **point** | Convergence of **trajectories to each other** |
| Guarantees | Typically local, asymptotic | Can be global, exponential |
| Time-varying | Requires time-varying $V$ | Natural (contraction of flows) |
| Composition | Hard (no general rules) | Hierarchical / parallel / feedback composition |
| Rate info | Usually qualitative | Explicit contraction rate $\beta$ |
| Metric choice | Lyapunov function $V(x)$ | Contraction metric $M(x)$ |
| Relation | $V(x) = \frac{1}{2}\|x - x^*\|^2$ is Lyapunov for the error | A contracting system implies Lyapunov stability of any particular trajectory |

Contraction theory subsumes Lyapunov for many robotics problems, but finding a contraction metric can be equally challenging.

### Questions — Section C.2

**QC.2.1** Consider the 1D system $\dot{x} = -\alpha x + u(t)$ where $\alpha > 0$ and $u(t)$ is any bounded input. Show this is contracting with rate $\alpha$. What does this mean for any two solutions with different initial conditions?

**QC.2.2** For the 2D system $\dot{x} = Ax$ where $A = \begin{bmatrix} -2 & 1 \\ 0 & -3 \end{bmatrix}$, compute $\text{sym}(A)$. Determine if the system is contracting (in the identity metric) and find the contraction rate if so.

**QC.2.3** Explain intuitively why contraction theory is better suited to trajectory tracking (time-varying references) than fixed-point Lyapunov analysis. What is the key difference in what is being proved?

**QC.2.4** A system has Jacobian $\partial f/\partial x$ with eigenvalues that are all in the left half-plane (negative real parts) but the symmetric part of $\partial f/\partial x$ has a positive eigenvalue at some points. Is the system necessarily non-contracting in the identity metric? Could it still be contracting in some other metric?

**QC.2.5** Construct a 2D example where the system is Lyapunov stable (all trajectories converge to the origin) but NOT contracting in any constant metric. (Hint: think about a system where nearby trajectories can temporarily diverge before converging.)

---

## C.3 Contraction Analysis of CLIK

### C.3.1 The CLIK System as an ODE

The CLIK controller:

$$\dot{\theta} = J^+(\theta) \cdot K \cdot \xi_e(\theta)$$

is a nonlinear ODE. Let's analyse it.

Define the task-space error map:

$$e(\theta) = \xi_e(\theta) = \text{vee}(\log(T_d \cdot T(\theta)^{-1}))$$

The time derivative of $e$ along the CLIK flow is:

$$\dot{e} = \frac{\partial e}{\partial \theta} \dot{\theta} = \frac{\partial e}{\partial \theta} J^+ K e$$

The matrix $\partial e/\partial \theta$ is related to the Jacobian. For the task-space error defined via the log map, near the solution:

$$\frac{\partial e}{\partial \theta} \approx -J_b(\theta) \quad \text{(the body Jacobian, with corrections from the log map derivative)}$$

More precisely, there is a matrix $\mathcal{T}(\xi_e)$ (related to the derivative of the log map, sometimes called the "left Jacobian inverse of $SE(3)$") such that:

$$\dot{e} \approx -\mathcal{T}(\xi_e) J_b J_b^+ K e$$

### C.3.2 Contraction in Task Space

For a **non-redundant** robot ($n = 6$, square invertible Jacobian), $J_b J_b^+ = I$, and:

$$\dot{e} \approx -\mathcal{T}(\xi_e) K e$$

Near the solution ($\xi_e \approx 0$), $\mathcal{T}(\xi_e) \to I$, so:

$$\dot{e} \approx -K e$$

This is a linear contracting system with rate equal to the smallest eigenvalue of $K$. The contraction analysis gives:

**Result**: CLIK is locally exponentially convergent with rate $\lambda_{\min}(K)$ in a neighbourhood of the solution, provided the Jacobian is non-singular.

### C.3.3 The Singularity Problem

At a singularity, $J$ loses rank, and $J^+$ has unbounded norm. The contraction condition breaks:

$$\text{sym}(\partial f/\partial \theta) \text{ is no longer uniformly negative definite}$$

because the pseudoinverse amplifies components in the near-singular directions. The contraction rate degrades as the robot approaches a singularity, and at the singularity itself, convergence is not guaranteed.

**Damped least-squares** (DLS) addresses this by replacing $J^+$ with:

$$J^\dagger_\lambda = J^\top(JJ^\top + \lambda^2 I)^{-1}$$

The damping term $\lambda$ bounds the gain, maintaining contraction at the cost of introducing a steady-state error proportional to $\lambda$. This is a direct trade-off: contraction rate vs tracking accuracy near singularities.

### C.3.4 Contraction and the Null Space

For a **redundant** robot ($n > 6$), the CLIK law with a null-space term is:

$$\dot{\theta} = J^+ K e + (I - J^+ J) z$$

The null-space term $(I - J^+ J)z$ does not affect the task-space error (by construction: $J(I - J^+ J) = 0$). In terms of task-space contraction:

$$\dot{e} \approx -\mathcal{T}(\xi_e) K e \quad \text{(same as before, independent of } z\text{)}$$

**The null-space term preserves the task-space contraction rate.** This is a key result: you can add arbitrary null-space behaviour without degrading task-space convergence, as long as:

1. The Jacobian remains non-singular (the robot stays away from singularities)
2. The null-space policy $z$ does not drive the robot into a singularity
3. The null-space motion does not violate joint limits (causing practical loss of DOF)

Condition 2 is the critical constraint on the RL null-space policy in your research. The policy must learn to avoid configurations that would break the task-space contraction guarantee.

### C.3.5 Contraction Regions and the Log Map

The analysis above used the approximation $\partial e/\partial \theta \approx -\mathcal{T}(\xi_e) J_b$. The matrix $\mathcal{T}(\xi_e)$ is the **left Jacobian** of $SE(3)$ (or its inverse, depending on convention). It has the form:

$$\mathcal{T}(\xi) = I - \tfrac{1}{2} \text{ad}_\xi + \tfrac{1}{6} \text{ad}^2_\xi - \cdots \quad \text{(a power series in } \text{ad}_\xi\text{)}$$

This series converges for $\|\xi\| < 2\pi$ (roughly, when the orientation error is less than a full rotation). The contraction analysis is valid within this region — which is essentially the entire practically relevant workspace.

For $\|\xi_e\| \to 0$, $\mathcal{T} \to I$ and we recover the simple linear analysis. For larger errors, $\mathcal{T}$ introduces coupling between the orientation and translation error channels. The contraction rate depends on the condition number of $\mathcal{T}(\xi_e) \cdot K$, which degrades for large errors. The analysis gives an explicit, computable bound on the convergence rate as a function of the error magnitude.

### C.3.6 Formal Contraction Result for CLIK

Putting it all together, the key theorem (which can be found in various forms in Bullo & Lewis, and in recent works by Slotine and collaborators):

**Theorem** (informal): Consider CLIK on $SE(3)$ with gain $K = kI$ for a non-redundant robot away from singularities. The system is:
- **Locally exponentially contracting** in a ball of radius $\rho$ around any solution, with contraction rate at least $k \cdot \sigma_{\min}(J)^2 / \|\mathcal{T}(\rho)\|$, where $\sigma_{\min}(J)$ is the smallest singular value of the Jacobian and $\mathcal{T}(\rho)$ bounds the log-map derivative.
- For a redundant robot with null-space term, the same task-space contraction rate holds, provided the null-space motion keeps $\sigma_{\min}(J)$ bounded away from zero.

The explicit dependence on $\sigma_{\min}(J)$ makes it clear: **the null-space policy's most important job (from a contraction perspective) is to keep the manipulability high**.

### Questions — Section C.3

**QC.3.1** For a 2R planar arm with CLIK gain $K = \text{diag}(5, 5)$, and the arm at a non-singular configuration where $\sigma_{\min}(J) = 0.3$, estimate the local contraction rate (using the simplified linear analysis $\dot{e} \approx -Ke$ near the solution).

**QC.3.2** Explain why damped least-squares introduces a steady-state error. Write the error dynamics with DLS and identify the bias term. Under what conditions is this bias small?

**QC.3.3** In the redundant CLIK law $\dot{\theta} = J^+ Ke + (I - J^+ J)z$, prove that $\dot{e}$ is independent of $z$ by differentiating $e(\theta)$ and using the property $J(I - J^+ J) = 0$.

**QC.3.4** A null-space policy $z = -\alpha \nabla h(\theta)$ performs gradient descent on some objective $h(\theta)$ (e.g., manipulability, joint-limit avoidance). What conditions must $h$ satisfy so that the null-space motion does not drive the robot toward a singularity?

**QC.3.5** The contraction rate depends on $\sigma_{\min}(J)$. For a 7-DOF arm, this depends on the configuration $\theta$. If the RL null-space policy maximises a reward that includes a manipulability bonus, how does this connect to maximising the contraction rate? Write the relationship explicitly.

---

## C.4 Contraction on Riemannian Manifolds

### C.4.1 Why We Need This

The joint space of a robot is $\mathbb{R}^n$ (or a torus for revolute joints), but the task space is $SE(3)$ — a curved manifold. The CLIK error $\xi_e$ lives in $\mathfrak{se}(3)$, which is a vector space, but the map from $\theta$ to $\xi_e$ goes through the manifold. For a rigorous global analysis, we need contraction theory on manifolds.

### C.4.2 Geodesic Contraction

On a Riemannian manifold $(M, g)$, a system $\dot{x} = f(x)$ is **geodesically contracting** if the distance between any two trajectories, measured along geodesics, decreases exponentially.

The contraction condition becomes:

$$\nabla_f + (\nabla_f)^* \leq -2\beta g$$

where $\nabla_f$ is the **covariant derivative** of $f$ and $(\nabla_f)^*$ is its adjoint with respect to the metric $g$. This generalises the Euclidean condition $\text{sym}(\partial f/\partial x) \leq -\beta I$.

### C.4.3 Contraction on $SO(3)$ and $SE(3)$

For systems evolving on $SO(3)$ (orientation tracking) or $SE(3)$ (pose tracking), the natural metric is the **bi-invariant metric** induced by the Killing form:

$$\langle \xi_1, \xi_2 \rangle = \text{tr}(\hat{\xi}_1^\top \hat{\xi}_2) \quad \text{(for } \mathfrak{so}(3)\text{)}$$

Under this metric, the CLIK controller on $SO(3)$:

$$\omega = K \cdot \text{vee}(\log(R_d R^\top))$$

is geodesically contracting with rate $K$ for $\|\theta_e\| < \pi$. The proof uses the fact that the log map on $SO(3)$ with the bi-invariant metric is a **normal coordinate chart**, and in normal coordinates the geodesic equation simplifies.

This gives a **global** (up to $\theta_e = \pi$, the cut locus) convergence guarantee — much stronger than the local Lyapunov analysis.

### C.4.4 The Cut Locus and Its Implications

The **cut locus** of the identity in $SO(3)$ is the set of rotations by exactly $\pi$ (180°). At the cut locus:
- The logarithm map becomes multi-valued (there are two geodesics of equal length to any $\pi$-rotation)
- The contraction analysis breaks down
- The controller may exhibit ambiguous behaviour (choosing between two equally valid correction directions)

In practice, the cut locus is almost never reached because:
1. The task-space error is typically small (the robot starts near the target)
2. Any noise or asymmetry breaks the ambiguity
3. The contraction guarantee for $\|\theta_e\| < \pi$ covers almost all practical scenarios

### C.4.5 Contraction Composition for Hierarchical Control

This is where contraction theory becomes most powerful for your research architecture:

**Theorem** (hierarchical contraction): If:
1. The task-space controller (CLIK) is contracting with rate $\beta_1$
2. The null-space policy produces joint motions that are contracting (in the null space) with rate $\beta_2$
3. The coupling between task and null-space subsystems satisfies a certain gain condition

Then the combined system is contracting with rate $\min(\beta_1, \beta_2)$ minus a coupling penalty.

This gives a **compositional** convergence guarantee: you can design the task-space controller and the null-space policy **separately**, then verify the coupling condition. For your RL null-space policy:

- Train the RL agent to maximise a reward that includes manipulability (ensuring $\beta_1$ stays high) and null-space objective achievement (ensuring $\beta_2$ is positive)
- The contraction framework guarantees that the combined system converges as long as the coupling is bounded
- The coupling bound can be explicitly computed from the Jacobian and the null-space projector

### Questions — Section C.4

**QC.4.1** Why does the contraction analysis of CLIK on $SO(3)$ break down at exactly $\theta_e = \pi$? What happens to the log map? What happens to the controller output?

**QC.4.2** For the bi-invariant metric on $SO(3)$, the inner product is $\langle \omega_1, \omega_2 \rangle = \omega_1^\top \omega_2$ (just the Euclidean inner product on the Lie algebra). Why is this metric called "bi-invariant"? What does it mean for the metric to be invariant under both left and right multiplication?

**QC.4.3** In the hierarchical contraction theorem, what plays the role of the "coupling" between the task-space and null-space subsystems? For CLIK with null-space projection, why is this coupling typically small?

**QC.4.4** An RL null-space policy has learned to maximise manipulability. During execution, it occasionally drives the robot toward a configuration where $\sigma_{\min}(J)$ drops to 0.01 (nearly singular). How does the contraction framework predict the system's behaviour? What will happen to the task-space tracking error?

**QC.4.5** *Research-level*: Sketch the structure of a convergence proof for CLIK with a learned null-space policy. What are the assumptions you would need on the learned policy? What would you need to verify empirically vs what can be guaranteed analytically? How does the contraction rate appear in the reward function design?

---

## C.5 From Theory to Your Research: CLIK + RL Null-Space Policies

### C.5.1 The Architecture

Your target paper architecture (as outlined in the RA-L direction):

```
       ┌───────────────┐
       │  Task-space    │   ξ_e = vee(log(T_d T⁻¹))
       │  error (SE(3)) │───────────────────────┐
       └───────────────┘                        │
                                                ▼
       ┌───────────────┐     ┌──────────┐    ┌──────────┐
       │  RL Policy     │────►│ z (null  │───►│  CLIK    │───► θ̇
       │  π(s)          │     │  space)  │    │  + null  │
       └───────────────┘     └──────────┘    └──────────┘
              ▲                                    │
              │        ┌───────────┐               │
              └────────│  State s  │◄──────────────┘
                       │  (θ, θ̇,  │
                       │  ξ_e, w) │
                       └───────────┘
```

The contraction-theoretic analysis provides the **safety certificate** for this architecture:

1. **Task-space contraction** (from CLIK) guarantees exponential convergence of the tracking error, provided the Jacobian stays well-conditioned
2. **Null-space independence** (from $J(I - J^+ J) = 0$) guarantees the RL policy cannot directly destabilise task tracking
3. **The RL policy's indirect effect** is through the Jacobian — by moving the robot to configurations with higher or lower manipulability, it modulates the contraction rate

### C.5.2 Reward Design from Contraction Rate

The contraction rate for CLIK is approximately:

$$\beta(\theta) \approx k \cdot \sigma_{\min}(J(\theta))^2$$

where $k$ is the CLIK gain. To maintain fast convergence, the RL policy should keep $\sigma_{\min}(J)$ high. This suggests a reward component:

$$r_{\text{contraction}}(\theta) = \sigma_{\min}(J(\theta)) \quad \text{or} \quad w(\theta) = \sqrt{\det(JJ^\top)}$$

This is the **manipulability** reward — which now has a principled justification from contraction theory, not just a heuristic one.

### C.5.3 What the RL Agent Can and Cannot Break

**Cannot break** (by construction):
- Task-space convergence direction (the null-space projection ensures $J\dot{\theta}_{\text{null}} = 0$)
- The CLIK gain structure

**Can degrade**:
- Convergence rate (by moving to near-singular configurations)
- Joint limit compliance (by commanding large null-space velocities)
- Practical stability (by oscillating between configurations)

**Can improve**:
- Convergence rate (by maintaining high manipulability)
- Joint-limit avoidance
- Obstacle avoidance
- Energy efficiency
- Any other secondary objective that doesn't conflict with task-space tracking

### C.5.4 The Contraction Guarantee as a Safety Constraint

For sim-to-real transfer, you can impose the contraction condition as a **hard constraint** during training:

$$\text{Constraint: } \sigma_{\min}(J(\theta)) \geq \sigma_{\min,\text{threshold}}$$

Any action $z$ that would violate this constraint (driving the robot toward a singularity) is rejected or penalised. This provides a formal safety guarantee: the task-space tracking error is bounded by:

$$\|\xi_e(t)\| \leq e^{-\beta_{\min} t} \|\xi_e(0)\| + \delta/\beta_{\min}$$

where $\delta$ accounts for bounded disturbances and $\beta_{\min} = k \cdot \sigma_{\min,\text{threshold}}^2$ is the guaranteed minimum contraction rate.

### C.5.5 Open Questions for Your Paper

These are the research gaps where your contribution lies:

1. **Empirical contraction rates**: How closely do the theoretical contraction rates (from the linearised analysis) match the empirical convergence in simulation? The gap between theory and practice determines how useful the theory is for reward design.

2. **Metric choice for redundant systems**: The task-space contraction is in the $\mathfrak{se}(3)$ metric. The null-space behaviour needs its own metric. What is the right joint-space metric for the full system? Can the RL policy learn a metric implicitly?

3. **Robustness under model mismatch**: The contraction analysis assumes a known kinematic model. When the model is imperfect (as in any real system), how robust is the convergence guarantee? Contraction theory provides tools for this (input-to-state stability of contracting systems), but the bounds may be conservative.

4. **Extending to dynamic (velocity/torque) control**: The CLIK analysis is kinematic (velocity-level). For real robots with inertia, the dynamics introduce additional coupling. Contraction theory can handle this (via the inertia matrix as a contraction metric), but the analysis is more involved.

### Questions — Section C.5

**QC.5.1** Write the complete CLIK + null-space control law for a 7-DOF arm tracking a desired pose $T_d(t)$, with an RL null-space policy $\pi(s)$ that outputs $z$. Specify every term, the state $s$, and the dimensions.

**QC.5.2** Design a reward function for the RL null-space policy that incorporates: (a) manipulability (for contraction rate), (b) distance from joint limits, (c) a task-specific secondary objective (e.g., elbow height). Write the reward as a weighted sum and justify the relative weights.

**QC.5.3** Prove that if $\sigma_{\min}(J(\theta(t))) \geq \sigma_{\text{th}} > 0$ for all $t$, then the CLIK tracking error satisfies $\|\xi_e(t)\| \leq e^{-kt \sigma_{\text{th}}^2} \|\xi_e(0)\|$ (exponential convergence bound). State the assumptions clearly.

**QC.5.4** An RL policy trained in simulation achieves good manipulability on average but occasionally (1% of timesteps) drops $\sigma_{\min}(J)$ below the threshold. How would you modify the training to eliminate these events? Consider both reward shaping and constrained RL approaches.

**QC.5.5** *Paper-level*: Outline the experiment section of your RA-L paper. What simulated and/or real robot would you use? What baselines would you compare against? What metrics would demonstrate the value of the contraction-theoretic reward design vs naive reward shaping?

---

# Appendix: Additional Notation for These Modules

| Symbol | Meaning |
|---|---|
| $q$ | Unit quaternion $(s, \mathbf{v})$ representing rotation |
| $q^*$ | Quaternion conjugate $(s, -\mathbf{v})$ |
| $\otimes$ | Quaternion multiplication |
| $S^3$ | Unit 3-sphere (space of unit quaternions) |
| SLERP | Spherical Linear Interpolation on $S^3$ |
| $\Omega$ | Angle between two quaternions on $S^3$ |
| $[A, B]$ | Lie bracket $= AB - BA$ |
| $\text{Ad}_g$ | Big Adjoint: conjugation by group element $g$ |
| $\text{ad}_\xi$ | Little adjoint: Lie bracket with $\xi$ |
| BCH | Baker-Campbell-Hausdorff formula |
| $\beta$ | Contraction rate |
| $M(x)$ | Contraction metric (positive definite matrix field) |
| $\sigma_{\min}(J)$ | Smallest singular value of Jacobian |
| $w(\theta)$ | Manipulability index $= \sqrt{\det(JJ^\top)}$ |
| $\mathcal{T}(\xi)$ | Left Jacobian of $SE(3)$ (log map derivative) |
| $J^+$ | Moore-Penrose pseudoinverse |
| $J^\dagger_\lambda$ | Damped least-squares inverse with damping $\lambda$ |
| $\xi_e$ | Task-space error in $\mathfrak{se}(3)$ |
| $\pi(s)$ | RL null-space policy |

---

# Appendix: Key References for These Modules

**Quaternions and SLERP**:
- Shoemake, K. — "Animating Rotation with Quaternion Curves" (SIGGRAPH 1985, the original SLERP paper)
- Kavan, L. et al. — "Dual Quaternions for Rigid Transformation Blending" (extends to $SE(3)$)
- Sola, J. — "Quaternion Kinematics for the Error-State Kalman Filter" (excellent practical reference, freely available)

**Lie Brackets and Adjoint**:
- Murray, Li, Sastry — *A Mathematical Introduction to Robotic Manipulation*, Ch. 2–3 (freely available)
- Selig, J.M. — *Geometric Fundamentals of Robotics*, Ch. 4–7 (deep screw/Lie theory)
- Hall, B. — *Lie Groups, Lie Algebras, and Representations*, Ch. 3 (the ad and Ad maps with full proofs)
- Park, F.C. — "Computational Aspects of the Product-of-Exponentials Formula" (Jacobian derivation via ad)

**Contraction Theory**:
- Lohmiller, W. & Slotine, J.J.E. — "On Contraction Analysis for Non-linear Systems" (Automatica 1998, the foundational paper)
- Slotine, J.J.E. — "Modular Stability Tools for Distributed Computation and Control" (Int. J. Adaptive Control, 2003)
- Bullo, F. — *Contraction Theory for Nonlinear Stability Analysis and Learning-based Control* (2023 monograph, available online, directly relevant to your work)
- Tsukamoto, H. et al. — "Neural Contraction Metrics for Robust Estimation and Control" (IEEE L-CSS 2021, learned contraction metrics)

**CLIK Convergence**:
- Siciliano, B. — "A Closed-Loop Inverse Kinematic Scheme" (1990, foundational CLIK paper)
- Bullo, F. & Murray, R.M. — "Proportional Derivative (PD) Control on the Euclidean Group" (geometric PD on $SE(3)$)
- Pham, Q.C. — "Kinematic Controller Convergence on $SE(3)$" (contraction-based CLIK analysis)
