# Robotics Mathematics: Advanced Topics

> **Prerequisite**: The 2-week core roadmap (Days 1–14).
> **Audience**: Researchers targeting geometric control and Lie-group methods for redundant manipulators.
> **Scope**: Three deep-dive modules that bridge the core roadmap to research-level geometric robotics.

---

# Module A — Quaternion Algebra and SLERP

---

## A.1 Why Quaternions?

The 2-week roadmap established rotation matrices (SO(3)) and exponential coordinates as the primary representations. Quaternions offer a **complementary** representation with specific computational advantages:

- **4 numbers, 1 constraint** vs 9 numbers, 6 constraints (matrices) or 3 numbers with singularities (Euler angles)
- **Composition** is quaternion multiplication — 16 multiplies vs 27 for matrix multiplication
- **Interpolation** has a clean, closed-form geodesic (SLERP) — no need for log/exp
- **Double cover** of SO(3) — topologically S³, which is simply connected, eliminating certain path-planning ambiguities
- **Numerically stable** — renormalising a quaternion to unit length is trivial (divide by its norm); re-orthogonalising a rotation matrix is expensive (SVD or Gram-Schmidt)

The trade-off: quaternions obscure the Lie-algebraic structure that makes Jacobians, twists, and null-space projections clean. In practice, modern robotics systems often use quaternions for **state representation and integration** but switch to Lie algebra (exponential coordinates) for **control and optimisation**.

---

## A.2 Quaternion Fundamentals

### A.2.1 Definition

A **quaternion** is a 4-tuple:

```
q = q₀ + q₁i + q₂j + q₃k = (s, v)
```

where s = q₀ is the **scalar part** and v = [q₁, q₂, q₃]ᵀ is the **vector part**. The basis elements satisfy the Hamilton relations:

```
i² = j² = k² = ijk = −1

ij = k,   jk = i,   ki = j
ji = −k,  kj = −i,  ik = −j
```

These relations encode the non-commutativity of 3D rotations at the algebraic level. The fact that ij ≠ ji is not a quirk — it is the quaternion manifestation of the same phenomenon as Rx(90°)Rz(90°) ≠ Rz(90°)Rx(90°).

### A.2.2 Quaternion Multiplication

Given p = (p₀, p_v) and q = (q₀, q_v):

```
pq = (p₀q₀ − p_v · q_v,  p₀q_v + q₀p_v + p_v × q_v)
```

The scalar part involves a dot product (measuring alignment). The vector part involves both scaling (the p₀q_v and q₀p_v terms) and a cross product (encoding rotational composition). The cross product term is what makes quaternion multiplication non-commutative.

### A.2.3 Conjugate, Norm, and Inverse

**Conjugate**: q* = (s, −v). Geometrically: same rotation axis, opposite angle.

**Norm**: ‖q‖ = √(q₀² + q₁² + q₂² + q₃²)

**Inverse**: q⁻¹ = q* / ‖q‖²

For **unit quaternions** (‖q‖ = 1): q⁻¹ = q*. This parallels R⁻¹ = Rᵀ for rotation matrices — inversion is trivially cheap.

### A.2.4 Unit Quaternions and Rotations

A **unit quaternion** represents a rotation via:

```
q = (cos(θ/2),  sin(θ/2) ω̂)
```

where ω̂ is the unit rotation axis and θ is the rotation angle. The set of all unit quaternions forms S³ (the 3-sphere in ℝ⁴).

**The rotation action**: to rotate a vector v ∈ ℝ³ by quaternion q, embed v as a pure quaternion v̄ = (0, v), then:

```
v_rotated = q v̄ q*
```

The result is a pure quaternion whose vector part is the rotated vector.

### A.2.5 The Double Cover

The quaternions q and −q represent the **same rotation**:

```
q v̄ q* = (−q) v̄ (−q)*
```

This is because negating both q and q* cancels out in the sandwich product. Geometrically, (ω̂, θ) and (−ω̂, θ + 2π) or equivalently (ω̂, θ) and (−ω̂, −(2π − θ)) produce the same rotation. The quaternion captures both with q and −q.

This double cover has a topological consequence: S³ is the **universal cover** of SO(3). Every loop in SO(3) that cannot be contracted to a point *can* be contracted in S³. This is the mathematical content of the Dirac belt trick — a 2π rotation in SO(3) is a non-contractible loop, but its lift to S³ is only half a great circle, which *can* be extended to a contractible loop (4π rotation = full great circle = contractible).

**Practical consequence**: When interpolating or tracking quaternion trajectories, you must ensure **quaternion consistency** — always pick the sign of q that is "closer" to the previous quaternion (i.e., ensure q_prev · q_current ≥ 0). Otherwise the interpolation may take the long way around S³.

### Questions — Section A.2

**QA.2.1** Compute the quaternion for a 90° rotation about the z-axis. Then compute the quaternion for a 90° rotation about the x-axis. Multiply them (in both orders) and convert back to axis-angle. Verify the results match those from multiplying Rz(90°)Rx(90°) and Rx(90°)Rz(90°).

**QA.2.2** Show that q and −q produce the same rotation by explicitly expanding qv̄q* and (−q)v̄(−q)* for an arbitrary pure quaternion v̄.

**QA.2.3** Given q = (cos(π/6), sin(π/6)[0, 0, 1]ᵀ), compute q* and verify that qq* = (1, 0, 0, 0) (the identity quaternion).

**QA.2.4** Derive the quaternion multiplication formula from the Hamilton relations. Start with (p₀ + p₁i + p₂j + p₃k)(q₀ + q₁i + q₂j + q₃k), expand all 16 terms, and group into scalar and vector parts.

**QA.2.5** Why does the half-angle θ/2 appear in the quaternion representation rather than the full angle θ? (Hint: consider what happens under composition — if you compose two rotations by θ, the quaternion angle adds as θ/2 + θ/2 = θ, but what does this correspond to on SO(3)?)

---

## A.3 Quaternion ↔ Rotation Matrix Conversions

### A.3.1 Quaternion → Matrix

Given unit quaternion q = (q₀, q₁, q₂, q₃):

```
R = [1−2(q₂²+q₃²)    2(q₁q₂−q₀q₃)    2(q₁q₃+q₀q₂)]
    [2(q₁q₂+q₀q₃)    1−2(q₁²+q₃²)    2(q₂q₃−q₀q₁)]
    [2(q₁q₃−q₀q₂)    2(q₂q₃+q₀q₁)    1−2(q₁²+q₂²)]
```

This can be derived by expanding the sandwich product qv̄q* in components.

### A.3.2 Matrix → Quaternion (Shepperd's Method)

The naive approach (compute θ from trace, then axis from R − Rᵀ) has numerical issues near θ = 0 and θ = π. **Shepperd's method** avoids these by choosing the largest diagonal element:

```
1. Compute: t = tr(R) = R₁₁ + R₂₂ + R₃₃
2. Find the largest of {t, R₁₁, R₂₂, R₃₃}
3. Use the corresponding formula:
```

If t is largest:
```
s = 2√(1 + t)
q₀ = s/4
q₁ = (R₃₂ − R₂₃)/s
q₂ = (R₁₃ − R₃₁)/s
q₃ = (R₂₁ − R₁₂)/s
```

If R₁₁ is largest:
```
s = 2√(1 + R₁₁ − R₂₂ − R₃₃)
q₁ = s/4
q₀ = (R₃₂ − R₂₃)/s
q₂ = (R₂₁ + R₁₂)/s
q₃ = (R₁₃ + R₃₁)/s
```

(Analogous formulas for R₂₂ and R₃₃ largest.)

The key insight: each formula divides by a different quantity, so you always pick the one with the largest denominator, avoiding division-by-small-number instabilities.

### A.3.3 Quaternion ↔ Axis-Angle

These are the most direct conversions:

**Axis-angle → Quaternion**:
```
q = (cos(θ/2),  sin(θ/2) ω̂)
```

**Quaternion → Axis-angle**:
```
θ = 2 arccos(q₀)      [or equivalently: θ = 2 arctan2(‖q_v‖, q₀)]
ω̂ = q_v / ‖q_v‖       [undefined when θ = 0, i.e., q_v = 0]
```

The arctan2 form is numerically preferable because arccos has poor sensitivity near q₀ = ±1.

### Questions — Section A.3

**QA.3.1** Convert the quaternion q = (√2/2, 0, 0, √2/2) to a rotation matrix using the formula. Verify the result is Rz(90°).

**QA.3.2** Take R = Rx(180°). Convert to a quaternion using Shepperd's method. Why would the naive trace-based approach (θ = arccos((tr(R)−1)/2)) be problematic here?

**QA.3.3** Implement (mentally or on paper) the full Shepperd algorithm for an arbitrary R. Count the number of arithmetic operations. Compare to extracting axis-angle via the log map (Day 7). Which is cheaper? Which is more numerically robust?

---

## A.4 SLERP — Spherical Linear Interpolation

### A.4.1 The Problem

Given two orientations q₀ and q₁ (as unit quaternions), find the "shortest path" interpolation q(t) for t ∈ [0, 1] such that:
- q(0) = q₀
- q(1) = q₁
- The path is a **geodesic** on S³ (constant angular velocity)

### A.4.2 Why Linear Interpolation Fails

The naive approach — LERP (linear interpolation):

```
q_lerp(t) = (1−t)q₀ + tq₁       ← NOT unit length!
```

The result does not lie on S³. You could renormalise:

```
q_nlerp(t) = normalize((1−t)q₀ + tq₁)
```

This is called **NLERP** (normalised linear interpolation). It does produce valid rotations, but the angular velocity is **not constant** — the interpolation speeds up in the middle and slows down near the endpoints. For many robotics applications (trajectory generation, motion planning), constant angular velocity is essential.

### A.4.3 The SLERP Formula

**SLERP** (Spherical Linear Interpolation) follows the great circle arc on S³:

```
SLERP(q₀, q₁, t) = q₀ · sin((1−t)Ω) / sin(Ω) + q₁ · sin(tΩ) / sin(Ω)
```

where:
```
Ω = arccos(q₀ · q₁)
```

Here q₀ · q₁ is the 4D dot product. The angle Ω is the angle between the two quaternions on S³ (which is **half** the rotation angle between the two orientations due to the double cover).

### A.4.4 Derivation from Geodesics on S³

S³ is a Riemannian manifold with constant positive curvature (it's a sphere). Geodesics on spheres are great circles. The parametric equation of a great circle through two points on a unit sphere is:

```
q(t) = q₀ · sin((1−t)Ω) / sin(Ω) + q₁ · sin(tΩ) / sin(Ω)
```

This is the spherical analogue of the linear combination (1−t)a + tb on a flat space. The sin functions ensure q(t) stays on S³ with constant angular speed.

**Proof that ‖q(t)‖ = 1**:

```
‖q(t)‖² = sin²((1−t)Ω)/sin²(Ω) + sin²(tΩ)/sin²(Ω) + 2 cos(Ω) sin((1−t)Ω)sin(tΩ)/sin²(Ω)
```

Using the identity sin(A)sin(B) = ½[cos(A−B) − cos(A+B)] and the fact that q₀ · q₁ = cos Ω, this simplifies to 1. The algebra is tedious but the geometric reason is simple: we're parameterising a great circle, which lies on the unit sphere by definition.

### A.4.5 Important Implementation Details

**1. Hemisphere check**: Before interpolating, ensure q₀ · q₁ ≥ 0. If not, negate one quaternion:

```
if q₀ · q₁ < 0:
    q₁ ← −q₁    (same rotation, opposite hemisphere)
    Ω ← arccos(−(q₀ · q₁))   [now takes the short path]
```

Without this, SLERP may interpolate the "long way around" (rotating 270° instead of 90°, for example).

**2. Small angle fallback**: When Ω ≈ 0 (q₀ ≈ q₁), sin Ω ≈ 0 and the formula degenerates. Fall back to NLERP:

```
if Ω < ε:
    return normalize((1−t)q₀ + tq₁)
```

For small angles, NLERP and SLERP are virtually identical, so this is safe.

**3. Constant angular velocity**: SLERP traverses the geodesic at constant speed. The instantaneous angular velocity at any t is:

```
|dθ/dt| = 2Ω / 1 = 2Ω   (constant)
```

This makes SLERP directly suitable for trajectory generation where you want smooth, predictable angular motion.

### A.4.6 SLERP via the Exponential Map

There is an equivalent formulation using the group structure:

```
SLERP(q₀, q₁, t) = q₀ · (q₀⁻¹ q₁)^t
```

where q^t means "raise the quaternion to the power t". For a unit quaternion q = (cos α, sin α · ω̂):

```
q^t = (cos(tα), sin(tα) · ω̂)
```

**Interpretation**: q₀⁻¹q₁ is the "difference rotation" from q₀ to q₁. Taking it to the power t scales that rotation to fraction t. Then left-multiplying by q₀ applies this partial rotation starting from q₀.

This is exactly analogous to the Lie group interpolation:

```
R(t) = R₀ · exp(t · log(R₀ᵀ R₁))
```

In fact, SLERP on quaternions and geodesic interpolation on SO(3) via exp/log produce **identical rotation trajectories**. They are two computational paths to the same geometric object.

### A.4.7 SLERP vs Lie Group Interpolation

| Aspect | Quaternion SLERP | SO(3) exp/log |
|---|---|---|
| Formula | q₀(q₀⁻¹q₁)^t | R₀ exp(t log(R₀ᵀR₁)) |
| Computational cost | ~20 multiplies + 1 arccos + 2 sin | Rodrigues log + exp (~50 multiplies) |
| Singularity | Ω → 0 (trivial fallback) | θ → 0 or π (needs careful handling) |
| Double cover | Must handle q vs −q | No ambiguity |
| Extends to SE(3) | Not natural | Directly via 𝔰𝔢(3) |
| Control/Jacobian use | Must convert to axis-angle | Native |

**Bottom line**: Use SLERP for interpolation and integration. Use exp/log for control laws and Jacobians.

### A.4.8 Beyond SLERP: SQUAD for Multi-Point Interpolation

SLERP interpolates between two orientations. For a sequence of orientations q₀, q₁, ..., qₙ, you need smooth transitions at the knot points. **SQUAD** (Spherical Quadrangle interpolation) provides C¹-continuous (continuous angular velocity) interpolation:

```
SQUAD(qᵢ, qᵢ₊₁, sᵢ, sᵢ₊₁, t) = SLERP(SLERP(qᵢ, qᵢ₊₁, t), SLERP(sᵢ, sᵢ₊₁, t), 2t(1−t))
```

where sᵢ are intermediate control quaternions constructed from the neighbouring orientations:

```
sᵢ = qᵢ · exp(−(log(qᵢ⁻¹qᵢ₊₁) + log(qᵢ⁻¹qᵢ₋₁)) / 4)
```

This is the spherical analogue of cubic spline interpolation with Hermite-like tangent matching. The construction ensures that the angular velocity is continuous at each knot point, which is essential for smooth robot trajectories.

### Questions — Section A.4

**QA.4.1** Compute SLERP(q₀, q₁, 0.5) where q₀ = (1, 0, 0, 0) (identity) and q₁ = (√2/2, 0, 0, √2/2) (90° about z). Verify the result corresponds to a 45° rotation about z.

**QA.4.2** Show that SLERP(q₀, q₁, t) = q₀(q₀⁻¹q₁)^t is equivalent to the sin-based formula. (Hint: write q₀⁻¹q₁ in axis-angle form, take it to the power t, left-multiply by q₀, and simplify using trig identities.)

**QA.4.3** Explain why NLERP does not produce constant angular velocity. Consider three equally spaced parameter values t = 0, 0.5, 1 and compute the rotation angle at each for q₀ = identity, q₁ = 90° about z. Is the angle at t = 0.5 exactly 45°?

**QA.4.4** You are generating a trajectory for a robot end-effector that must pass through orientations q₀, q₁, q₂. Using piecewise SLERP (SLERP from q₀ to q₁, then SLERP from q₁ to q₂), is the angular velocity continuous at q₁? Why or why not? How does SQUAD fix this?

**QA.4.5** Prove that for unit quaternions, ‖SLERP(q₀, q₁, t)‖ = 1 for all t ∈ [0,1]. (Use the sin-based formula and trig identities.)

---

## A.5 Quaternion Angular Velocity and Integration

### A.5.1 Quaternion Kinematics Equation

The analogue of Ṙ = [ω]× R for quaternions is:

```
q̇ = ½ q ⊗ ω̄
```

where ω̄ = (0, ω) is the body-frame angular velocity embedded as a pure quaternion, and ⊗ denotes quaternion multiplication. In matrix form:

```
q̇ = ½ Ω(ω) q
```

where Ω(ω) is the 4×4 matrix:

```
Ω(ω) = [ 0   −ω₁  −ω₂  −ω₃]
        [ ω₁    0    ω₃  −ω₂]
        [ ω₂  −ω₃    0    ω₁]
        [ ω₃   ω₂  −ω₁    0 ]
```

This is a **linear** ODE in q (given ω). This linearity is a significant advantage for integration — you can use standard integrators (RK4, etc.) directly, then renormalise.

### A.5.2 Integration and Unit Constraint

After each integration step, the quaternion may drift from unit length due to numerical error. The fix is trivial:

```
q ← q / ‖q‖
```

Compare this to rotation matrices, where maintaining orthogonality requires SVD decomposition or iterative Gram-Schmidt — vastly more expensive. This is the primary reason many simulators (MuJoCo, Bullet, Drake) use quaternions for state representation.

### A.5.3 Quaternion Error for Control

The quaternion orientation error between desired q_d and current q is:

```
q_e = q_d ⊗ q⁻¹ = q_d ⊗ q*
```

The vector part of q_e gives a 3D error signal:

```
e_quat = q_e_vec = sin(θ_e/2) ω̂_e
```

For small errors, sin(θ_e/2) ≈ θ_e/2, so e_quat ≈ ½ θ_e ω̂_e — proportional to the axis-angle error. Many PD controllers on SO(3) use this directly:

```
τ = −K_p · e_quat − K_d · ω_e
```

This is equivalent (for small errors) to the Lie-algebra controller τ = −K_p · vee(log(R_e)) − K_d · ω_e from Day 14, but with the quaternion form being computationally cheaper (no log map needed).

### Questions — Section A.5

**QA.5.1** Starting from q = (1, 0, 0, 0) with constant body angular velocity ω = [0, 0, π]ᵀ (180°/s about z), integrate q̇ = ½ q ⊗ ω̄ for t = 1 second using exact integration (hint: constant ω gives an analytical solution). What quaternion do you get? What rotation does it represent?

**QA.5.2** Show that ‖q̇‖ depends on ‖ω‖ but not on q (for unit quaternions). What does this imply about the "speed" of traversal on S³?

**QA.5.3** Compare the quaternion error e_quat = (q_d q*)_vec with the Lie algebra error e_R = vee(log(R_d Rᵀ)) for a 10° error about the z-axis. Compute both numerically. By what factor do they differ?

---

# Module B — Lie Bracket and Adjoint Representation at the Algebra Level

---

## B.1 The Lie Bracket on 𝔰𝔬(3)

### B.1.1 Definition

The **Lie bracket** on 𝔰𝔬(3) is the matrix commutator:

```
[A, B] = AB − BA
```

where A, B ∈ 𝔰𝔬(3) (3×3 skew-symmetric matrices).

**Fundamental fact**: If A and B are skew-symmetric, then [A, B] is also skew-symmetric. The Lie bracket is a **closed operation** on 𝔰𝔬(3) — it maps pairs of Lie algebra elements to a new Lie algebra element.

### B.1.2 The Bracket in Terms of Vectors

Recall the "hat" map (∧): ℝ³ → 𝔰𝔬(3) that maps ω ↦ [ω]×. The Lie bracket on 𝔰𝔬(3) corresponds to the **cross product** on ℝ³:

```
[[α]×, [β]×] = [α × β]×
```

**Proof**: Expand [α]×[β]× − [β]×[α]× using the identity [a]×[b]× = baᵀ − (a·b)I, and verify the result equals [α × β]×. The algebra is instructive — do it once.

This is a profound connection: the Lie bracket structure of 𝔰𝔬(3) is *exactly* the cross product structure of ℝ³. The cross product is not just a computational trick — it is the Lie bracket of the rotation group, making ℝ³ with the cross product isomorphic to 𝔰𝔬(3) as a Lie algebra.

### B.1.3 Lie Bracket Axioms

The Lie bracket satisfies three axioms:

**Bilinearity**:
```
[αA + βB, C] = α[A,C] + β[B,C]
[A, αB + βC] = α[A,B] + β[A,C]
```

**Antisymmetry**:
```
[A, B] = −[B, A]
```

**Jacobi identity**:
```
[A, [B, C]] + [B, [C, A]] + [C, [A, B]] = 0
```

For 𝔰𝔬(3) via the cross product, the Jacobi identity becomes:
```
α × (β × γ) + β × (γ × α) + γ × (α × β) = 0
```

This is a classical vector identity (verify using the BAC-CAB rule). The Jacobi identity has deep physical consequences — it encodes the consistency of infinitesimal symmetry transformations and appears in angular momentum commutation relations in mechanics.

### B.1.4 Physical Meaning of the Lie Bracket

The Lie bracket [A, B] measures the **failure of two infinitesimal transformations to commute**. More precisely, if you:

1. Flow along A for time ε
2. Flow along B for time ε
3. Flow along −A for time ε
4. Flow along −B for time ε

you do NOT return to the start. The gap is proportional to ε²[A, B]:

```
e^{εA} e^{εB} e^{−εA} e^{−εB} ≈ I + ε²[A,B] + O(ε³)
```

For rotations: if you rotate by small ε about x, then by ε about y, then by −ε about x, then by −ε about y, you end up with a small rotation about z (proportional to ε²). This is because [eₓ, eᵧ] = eᵤ under the cross product — exactly matching the commutator of the corresponding skew-symmetric matrices.

**This is the geometric content of non-commutativity**: the Lie bracket tells you what new direction is generated by the commutator of two motions.

### Questions — Section B.1

**QB.1.1** Compute [[e₁]×, [e₂]×] directly (matrix multiply and subtract). Verify the result equals [e₃]× = [e₁ × e₂]×.

**QB.1.2** Verify the Jacobi identity for α = e₁, β = e₂, γ = e₃ (i.e., the standard basis vectors) using the cross product form.

**QB.1.3** Compute e^{ε[e₁]×} e^{ε[e₂]×} e^{−ε[e₁]×} e^{−ε[e₂]×} for ε = 0.1 using Rodrigues (or numerically). Show the result is close to e^{ε²[e₃]×} = Rz(0.01). How close?

**QB.1.4** The Lie bracket of 𝔰𝔬(2) (2×2 skew-symmetric matrices) is always zero: any two elements commute. What does this say about 2D rotations? Why is 3D fundamentally different from 2D in this regard?

**QB.1.5** Consider two angular velocities ω₁ and ω₂ applied to a rigid body. The Lie bracket [ω₁]×, [ω₂]× = [ω₁ × ω₂]× represents a new angular velocity direction. Give a physical scenario where this "bracket-generated" direction matters (hint: think about controllability of underactuated systems).

---

## B.2 The Lie Bracket on 𝔰𝔢(3)

### B.2.1 Elements of 𝔰𝔢(3)

The Lie algebra of SE(3) consists of 4×4 matrices:

```
[ξ̂] = [[ω]×  v]  ∈ 𝔰𝔢(3)
       [  0ᵀ  0]
```

where [ω]× ∈ 𝔰𝔬(3) and v ∈ ℝ³. Using the "vee" notation, ξ = [v; ω] ∈ ℝ⁶.

### B.2.2 The Bracket on 𝔰𝔢(3)

For ξ₁ = [v₁; ω₁] and ξ₂ = [v₂; ω₂]:

```
[ξ̂₁, ξ̂₂] = ξ̂₁ξ̂₂ − ξ̂₂ξ̂₁
```

Computing this:

```
[ξ̂₁, ξ̂₂] = [   [ω₁ × ω₂]×     ω₁ × v₂ − ω₂ × v₁  ]
              [      0ᵀ                    0              ]
```

In vector form:
```
[ξ₁, ξ₂] = [ω₁ × v₂ − ω₂ × v₁]
             [     ω₁ × ω₂       ]
```

The angular part is the same as 𝔰𝔬(3) — just the cross product of the angular velocities. The linear part involves the **coupling** between angular and linear components through cross products. This coupling is the hallmark of rigid body kinematics: rotation affects translation.

### B.2.3 Lie Bracket and Screw Geometry

The bracket of two twists has a beautiful screw-theoretic interpretation: it measures the rate of change of one screw as the body moves along the other. If ξ₁ is a joint screw and ξ₂ is another, [ξ₁, ξ₂] tells you how joint 2's screw axis changes as joint 1 moves — which is exactly the information needed to construct Jacobians.

This connects to the **Lie derivative** in differential geometry: the Lie bracket of two vector fields measures how one field changes along the flow of the other.

### Questions — Section B.2

**QB.2.1** Compute the Lie bracket of ξ₁ = [0,0,0, 0,0,1]ᵀ (pure rotation about z) and ξ₂ = [1,0,0, 0,0,0]ᵀ (pure translation in x). Interpret the result geometrically — what new motion is generated?

**QB.2.2** Two revolute joints have twists ξ₁ = [0,0,0, 0,0,1]ᵀ (z-rotation at origin) and ξ₂ = [0,L,0, 0,0,1]ᵀ (z-rotation at (L,0,0)). Compute [ξ₁, ξ₂]. What does this bracket tell you about the kinematics?

**QB.2.3** Verify the Jacobi identity for three twists of your choice in 𝔰𝔢(3).

**QB.2.4** Show that the Lie bracket of two pure translations is always zero. What does this mean physically?

---

## B.3 The Adjoint Representation

### B.3.1 Two Levels of Adjoint

There are **two distinct adjoint maps** in Lie group theory, and conflating them is a common source of confusion:

**1. The (big) Adjoint Ad_g**: maps the Lie algebra to itself via conjugation by a group element g ∈ G:

```
Ad_g(ξ) = g ξ g⁻¹       (in matrix form)
```

For SO(3): Ad_R(ω̂×) = R[ω]× Rᵀ = [Rω]×. In vector form: Ad_R(ω) = Rω. This is just rotation of the angular velocity vector — physically, changing the frame in which the angular velocity is expressed.

For SE(3): the 6×6 matrix we encountered on Day 13:
```
Ad_T = [R   [p]×R]
       [0     R  ]
```

**2. The (little) adjoint ad_ξ**: maps the Lie algebra to itself via the Lie bracket with a fixed algebra element:

```
ad_ξ(η) = [ξ, η]       (the Lie bracket)
```

For 𝔰𝔬(3): ad_ω(β) = [ω]× β = ω × β. The little adjoint *is* the cross product / skew-symmetric matrix.

For 𝔰𝔢(3): given ξ = [v; ω], the 6×6 matrix representation of ad_ξ is:

```
ad_ξ = [[ω]×   [v]×]
       [  0    [ω]×]
```

### B.3.2 The Fundamental Relationship

The big and little adjoints are connected by differentiation:

```
ad_ξ = d/dt|_{t=0} Ad_{exp(tξ)}
```

The little adjoint is the **derivative of the big Adjoint at the identity**. This is a general Lie group fact: the Lie algebra "differentiates" the Lie group.

Equivalently, the matrix exponential connects them:

```
Ad_{exp(ξ)} = exp(ad_ξ)     (as 6×6 matrices for SE(3))
```

### B.3.3 The Adjoint in Velocity Kinematics

The Space Jacobian column formula from Day 12:

```
Jˢ_i = Ad_{T₁···T_{i-1}} ξ_i
```

uses the big Adjoint. Each joint's screw is defined at the zero configuration, and the Adjoint "transports" it through the motions of preceding joints.

The **derivative** of the Jacobian — needed for acceleration-level kinematics and Hessian computations — involves the little adjoint (ad) because it captures how screws change infinitesimally as joints move.

### B.3.4 The BCH Formula and the Role of ad

The **Baker-Campbell-Hausdorff (BCH) formula** expresses the log of a product of exponentials:

```
log(e^A e^B) = A + B + ½[A,B] + 1/12([A,[A,B]] − [B,[A,B]]) + ...
```

All higher-order terms are nested Lie brackets. This formula is fundamental to understanding how joint motions compose in the exponential coordinate framework.

For small motions (first order): log(e^A e^B) ≈ A + B. The Lie bracket correction ½[A,B] is the **leading-order non-commutativity term** — it quantifies how much the result deviates from simple addition.

In robotics, the BCH formula appears when:
- Composing small joint motions in task space
- Analysing the Jacobian of the POE formula
- Understanding geometric integration schemes (Lie group integrators)

### B.3.5 The ad Matrix and Rigid Body Dynamics

In rigid body dynamics on SE(3), the equations of motion in body-frame coordinates are:

```
M V̇_b = ad*_{V_b} M V_b + F_b
```

where M is the 6×6 spatial inertia matrix, V_b is the body twist, ad* is the **co-adjoint** (dual of ad), and F_b is the applied wrench. The ad*_{V_b} M V_b term contains Coriolis and centrifugal effects.

For SO(3) alone, this reduces to Euler's equation:

```
I ω̇ = (Iω) × ω + τ = −[ω]× Iω + τ
```

The [ω]× matrix appearing here is precisely ad_ω — the little adjoint on 𝔰𝔬(3). The Coriolis/centrifugal coupling in rigid body dynamics is a manifestation of the Lie algebra structure.

### Questions — Section B.3

**QB.3.1** For R = Rz(θ), compute Ad_R(ω) for ω = [1, 0, 0]ᵀ. Verify the result is Rω = [cos θ, sin θ, 0]ᵀ. Interpret: a body spinning about x in its own frame — what does the angular velocity look like in the world frame?

**QB.3.2** Compute the 6×6 ad_ξ matrix for ξ = [0, 0, 0, 0, 0, 1]ᵀ (pure z-rotation). Apply it to η = [1, 0, 0, 0, 0, 0]ᵀ (x-translation). Verify the result matches [ξ, η] computed via the bracket formula.

**QB.3.3** Verify the identity Ad_{exp(ξ)} = exp(ad_ξ) for ξ = [0, 0, 0, 0, 0, π/2]ᵀ (90° rotation about z). Compute both sides as 6×6 matrices.

**QB.3.4** In the BCH formula, compute the first three terms of log(e^A e^B) for A = 0.1[e₁]× and B = 0.1[e₂]× (small rotations about x and y). How significant is the bracket correction term relative to A + B?

**QB.3.5** Euler's equation Iω̇ = −[ω]× Iω + τ can be written as Iω̇ = −ad*_ω(Iω) + τ where ad*_ω(p) = [ω]× p for p ∈ ℝ³. Explain why this structure (the co-adjoint) arises from the non-commutativity of SO(3). What would the equation look like if SO(3) were commutative (i.e., if the Lie bracket were zero)?

---

# Module C — Contraction Theory and CLIK Convergence

---

## C.1 The Convergence Problem in CLIK

### C.1.1 Recap: CLIK on SE(3)

Closed-Loop Inverse Kinematics (CLIK) solves the inverse kinematics problem as a feedback control loop:

```
θ̇ = J⁺(θ) · K · ξ_e(θ)
```

where:
- ξ_e(θ) = vee(log(T_d · T(θ)⁻¹)) is the task-space error in the Lie algebra
- J⁺(θ) is the Jacobian pseudoinverse (or damped least-squares inverse)
- K is a positive-definite gain matrix

This is an autonomous ODE on the joint space: θ̇ = f(θ). The question is: **does it converge?** And if so, from what initial conditions? And how fast?

### C.1.2 Classical Lyapunov Analysis

The standard approach is to pick a Lyapunov function:

```
V(θ) = ½ ‖ξ_e(θ)‖²
```

and show V̇ < 0. This works locally (near the solution) but has limitations:
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

```
ẋ = f(x, t)
```

Classical stability asks: does a trajectory converge to an **equilibrium point**? Contraction theory asks a different question: do **all trajectories converge to each other**?

A system is **contracting** if any two trajectories, regardless of initial conditions, converge to each other exponentially:

```
‖x₁(t) − x₂(t)‖ ≤ e^{−βt} ‖x₁(0) − x₂(0)‖
```

where β > 0 is the **contraction rate**. The system "forgets" its initial conditions exponentially fast.

### C.2.2 The Contraction Condition

The system ẋ = f(x, t) is contracting with rate β if there exists a uniformly positive definite metric M(x, t) such that the **generalised Jacobian** satisfies:

```
F = (Ṁ + (∂f/∂x)ᵀM + M(∂f/∂x)) / 2 ≤ −β M
```

In the simplest case with M = I (the identity metric):

```
sym(∂f/∂x) ≤ −βI
```

where sym(A) = (A + Aᵀ)/2 is the symmetric part. This means: the symmetric part of the Jacobian of f must be **uniformly negative definite**.

**Interpretation**: The Jacobian ∂f/∂x governs how nearby trajectories evolve relative to each other. If its symmetric part is negative definite everywhere, nearby trajectories converge — the "flow" is contracting in all directions.

### C.2.3 Why the Symmetric Part?

The Jacobian ∂f/∂x may not be symmetric. Its effect on the distance between nearby trajectories is governed by the virtual displacement δx:

```
d/dt(δx) = (∂f/∂x) δx
```

The rate of change of ‖δx‖² is:

```
d/dt ‖δx‖² = 2 δxᵀ (∂f/∂x) δx = 2 δxᵀ sym(∂f/∂x) δx
```

Only the symmetric part contributes (the antisymmetric part cancels in the quadratic form). So the symmetric part of the Jacobian is the "contraction-relevant" part.

### C.2.4 Contraction in a General Metric

The identity metric is often too restrictive. A **contraction metric** M(x, t) (symmetric positive definite matrix field) defines a Riemannian distance:

```
‖δx‖_M = √(δxᵀ M δx)
```

The system contracts in this metric if:

```
Ṁ + (∂f/∂x)ᵀ M + M (∂f/∂x) ≤ −2β M
```

Finding the right metric is the art of contraction analysis. For robotic systems, the **inertia matrix** of the robot or a task-space metric are natural candidates.

### C.2.5 Comparison with Lyapunov Theory

| Aspect | Lyapunov | Contraction |
|---|---|---|
| Analyses | Convergence to a **point** | Convergence of **trajectories to each other** |
| Guarantees | Typically local, asymptotic | Can be global, exponential |
| Time-varying | Requires time-varying V | Natural (contraction of flows) |
| Composition | Hard (no general rules) | Hierarchical / parallel / feedback composition |
| Rate info | Usually qualitative | Explicit contraction rate β |
| Metric choice | Lyapunov function V(x) | Contraction metric M(x) |
| Relation | V(x) = ½‖x − x*‖² is Lyapunov for the error | A contracting system implies Lyapunov stability of any particular trajectory |

Contraction theory subsumes Lyapunov for many robotics problems, but finding a contraction metric can be equally challenging.

### Questions — Section C.2

**QC.2.1** Consider the 1D system ẋ = −αx + u(t) where α > 0 and u(t) is any bounded input. Show this is contracting with rate α. What does this mean for any two solutions with different initial conditions?

**QC.2.2** For the 2D system ẋ = Ax where A = [−2, 1; 0, −3], compute sym(A). Determine if the system is contracting (in the identity metric) and find the contraction rate if so.

**QC.2.3** Explain intuitively why contraction theory is better suited to trajectory tracking (time-varying references) than fixed-point Lyapunov analysis. What is the key difference in what is being proved?

**QC.2.4** A system has Jacobian ∂f/∂x with eigenvalues that are all in the left half-plane (negative real parts) but the symmetric part of ∂f/∂x has a positive eigenvalue at some points. Is the system necessarily non-contracting in the identity metric? Could it still be contracting in some other metric?

**QC.2.5** Construct a 2D example where the system is Lyapunov stable (all trajectories converge to the origin) but NOT contracting in any constant metric. (Hint: think about a system where nearby trajectories can temporarily diverge before converging.)

---

## C.3 Contraction Analysis of CLIK

### C.3.1 The CLIK System as an ODE

The CLIK controller:

```
θ̇ = J⁺(θ) · K · ξ_e(θ)
```

is a nonlinear ODE. Let's analyse it.

Define the task-space error map:

```
e(θ) = ξ_e(θ) = vee(log(T_d · T(θ)⁻¹))
```

The time derivative of e along the CLIK flow is:

```
ė = (∂e/∂θ) θ̇ = (∂e/∂θ) J⁺ K e
```

The matrix ∂e/∂θ is related to the Jacobian. For the task-space error defined via the log map, near the solution:

```
∂e/∂θ ≈ −J_b(θ)     (the body Jacobian, with corrections from the log map derivative)
```

More precisely, there is a matrix T(ξ_e) (related to the derivative of the log map, sometimes called the "left Jacobian inverse of SE(3)") such that:

```
ė ≈ −T(ξ_e) J_b J_b⁺ K e
```

### C.3.2 Contraction in Task Space

For a **non-redundant** robot (n = 6, square invertible Jacobian), J_b J_b⁺ = I, and:

```
ė ≈ −T(ξ_e) K e
```

Near the solution (ξ_e ≈ 0), T(ξ_e) → I, so:

```
ė ≈ −K e
```

This is a linear contracting system with rate equal to the smallest eigenvalue of K. The contraction analysis gives:

**Result**: CLIK is locally exponentially convergent with rate λ_min(K) in a neighbourhood of the solution, provided the Jacobian is non-singular.

### C.3.3 The Singularity Problem

At a singularity, J loses rank, and J⁺ has unbounded norm. The contraction condition breaks:

```
sym(∂f/∂θ) is no longer uniformly negative definite
```

because the pseudoinverse amplifies components in the near-singular directions. The contraction rate degrades as the robot approaches a singularity, and at the singularity itself, convergence is not guaranteed.

**Damped least-squares** (DLS) addresses this by replacing J⁺ with:

```
J†_λ = Jᵀ(JJᵀ + λ²I)⁻¹
```

The damping term λ bounds the gain, maintaining contraction at the cost of introducing a steady-state error proportional to λ. This is a direct trade-off: contraction rate vs tracking accuracy near singularities.

### C.3.4 Contraction and the Null Space

For a **redundant** robot (n > 6), the CLIK law with a null-space term is:

```
θ̇ = J⁺ K e + (I − J⁺J) z
```

The null-space term (I − J⁺J)z does not affect the task-space error (by construction: J(I − J⁺J) = 0). In terms of task-space contraction:

```
ė ≈ −T(ξ_e) K e    (same as before, independent of z)
```

**The null-space term preserves the task-space contraction rate.** This is a key result: you can add arbitrary null-space behaviour without degrading task-space convergence, as long as:

1. The Jacobian remains non-singular (the robot stays away from singularities)
2. The null-space policy z does not drive the robot into a singularity
3. The null-space motion does not violate joint limits (causing practical loss of DOF)

Condition 2 is the critical constraint on the RL null-space policy in your research. The policy must learn to avoid configurations that would break the task-space contraction guarantee.

### C.3.5 Contraction Regions and the Log Map

The analysis above used the approximation ∂e/∂θ ≈ −T(ξ_e) J_b. The matrix T(ξ_e) is the **left Jacobian** of SE(3) (or its inverse, depending on convention). It has the form:

```
T(ξ) = I − ½ ad_ξ + (1/6) ad²_ξ − ...    (a power series in ad_ξ)
```

This series converges for ‖ξ‖ < 2π (roughly, when the orientation error is less than a full rotation). The contraction analysis is valid within this region — which is essentially the entire practically relevant workspace.

For ‖ξ_e‖ → 0, T → I and we recover the simple linear analysis. For larger errors, T introduces coupling between the orientation and translation error channels. The contraction rate depends on the condition number of T(ξ_e) · K, which degrades for large errors. The analysis gives an explicit, computable bound on the convergence rate as a function of the error magnitude.

### C.3.6 Formal Contraction Result for CLIK

Putting it all together, the key theorem (which can be found in various forms in Bullo & Lewis, and in recent works by Slotine and collaborators):

**Theorem** (informal): Consider CLIK on SE(3) with gain K = kI for a non-redundant robot away from singularities. The system is:
- **Locally exponentially contracting** in a ball of radius ρ around any solution, with contraction rate at least k · σ_min(J)² / ‖T(ρ)‖, where σ_min(J) is the smallest singular value of the Jacobian and T(ρ) bounds the log-map derivative.
- For a redundant robot with null-space term, the same task-space contraction rate holds, provided the null-space motion keeps σ_min(J) bounded away from zero.

The explicit dependence on σ_min(J) makes it clear: **the null-space policy's most important job (from a contraction perspective) is to keep the manipulability high**.

### Questions — Section C.3

**QC.3.1** For a 2R planar arm with CLIK gain K = diag(5, 5), and the arm at a non-singular configuration where σ_min(J) = 0.3, estimate the local contraction rate (using the simplified linear analysis ė ≈ −Ke near the solution).

**QC.3.2** Explain why damped least-squares introduces a steady-state error. Write the error dynamics with DLS and identify the bias term. Under what conditions is this bias small?

**QC.3.3** In the redundant CLIK law θ̇ = J⁺Ke + (I − J⁺J)z, prove that ė is independent of z by differentiating e(θ) and using the property J(I − J⁺J) = 0.

**QC.3.4** A null-space policy z = −α∇h(θ) performs gradient descent on some objective h(θ) (e.g., manipulability, joint-limit avoidance). What conditions must h satisfy so that the null-space motion does not drive the robot toward a singularity?

**QC.3.5** The contraction rate depends on σ_min(J). For a 7-DOF arm, this depends on the configuration θ. If the RL null-space policy maximises a reward that includes a manipulability bonus, how does this connect to maximising the contraction rate? Write the relationship explicitly.

---

## C.4 Contraction on Riemannian Manifolds

### C.4.1 Why We Need This

The joint space of a robot is ℝⁿ (or a torus for revolute joints), but the task space is SE(3) — a curved manifold. The CLIK error ξ_e lives in 𝔰𝔢(3), which is a vector space, but the map from θ to ξ_e goes through the manifold. For a rigorous global analysis, we need contraction theory on manifolds.

### C.4.2 Geodesic Contraction

On a Riemannian manifold (M, g), a system ẋ = f(x) is **geodesically contracting** if the distance between any two trajectories, measured along geodesics, decreases exponentially.

The contraction condition becomes:

```
∇_f + (∇_f)* ≤ −2β g
```

where ∇_f is the **covariant derivative** of f and (∇_f)* is its adjoint with respect to the metric g. This generalises the Euclidean condition sym(∂f/∂x) ≤ −βI.

### C.4.3 Contraction on SO(3) and SE(3)

For systems evolving on SO(3) (orientation tracking) or SE(3) (pose tracking), the natural metric is the **bi-invariant metric** induced by the Killing form:

```
⟨ξ₁, ξ₂⟩ = tr(ξ̂₁ᵀ ξ̂₂)    (for 𝔰𝔬(3))
```

Under this metric, the CLIK controller on SO(3):

```
ω = K · vee(log(R_d Rᵀ))
```

is geodesically contracting with rate K for ‖θ_e‖ < π. The proof uses the fact that the log map on SO(3) with the bi-invariant metric is a **normal coordinate chart**, and in normal coordinates the geodesic equation simplifies.

This gives a **global** (up to θ_e = π, the cut locus) convergence guarantee — much stronger than the local Lyapunov analysis.

### C.4.4 The Cut Locus and Its Implications

The **cut locus** of the identity in SO(3) is the set of rotations by exactly π (180°). At the cut locus:
- The logarithm map becomes multi-valued (there are two geodesics of equal length to any π-rotation)
- The contraction analysis breaks down
- The controller may exhibit ambiguous behaviour (choosing between two equally valid correction directions)

In practice, the cut locus is almost never reached because:
1. The task-space error is typically small (the robot starts near the target)
2. Any noise or asymmetry breaks the ambiguity
3. The contraction guarantee for ‖θ_e‖ < π covers almost all practical scenarios

### C.4.5 Contraction Composition for Hierarchical Control

This is where contraction theory becomes most powerful for your research architecture:

**Theorem** (hierarchical contraction): If:
1. The task-space controller (CLIK) is contracting with rate β₁
2. The null-space policy produces joint motions that are contracting (in the null space) with rate β₂
3. The coupling between task and null-space subsystems satisfies a certain gain condition

Then the combined system is contracting with rate min(β₁, β₂) minus a coupling penalty.

This gives a **compositional** convergence guarantee: you can design the task-space controller and the null-space policy **separately**, then verify the coupling condition. For your RL null-space policy:

- Train the RL agent to maximise a reward that includes manipulability (ensuring β₁ stays high) and null-space objective achievement (ensuring β₂ is positive)
- The contraction framework guarantees that the combined system converges as long as the coupling is bounded
- The coupling bound can be explicitly computed from the Jacobian and the null-space projector

### Questions — Section C.4

**QC.4.1** Why does the contraction analysis of CLIK on SO(3) break down at exactly θ_e = π? What happens to the log map? What happens to the controller output?

**QC.4.2** For the bi-invariant metric on SO(3), the inner product is ⟨ω₁, ω₂⟩ = ω₁ᵀω₂ (just the Euclidean inner product on the Lie algebra). Why is this metric called "bi-invariant"? What does it mean for the metric to be invariant under both left and right multiplication?

**QC.4.3** In the hierarchical contraction theorem, what plays the role of the "coupling" between the task-space and null-space subsystems? For CLIK with null-space projection, why is this coupling typically small?

**QC.4.4** An RL null-space policy has learned to maximise manipulability. During execution, it occasionally drives the robot toward a configuration where σ_min(J) drops to 0.01 (nearly singular). How does the contraction framework predict the system's behaviour? What will happen to the task-space tracking error?

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
2. **Null-space independence** (from J(I − J⁺J) = 0) guarantees the RL policy cannot directly destabilise task tracking
3. **The RL policy's indirect effect** is through the Jacobian — by moving the robot to configurations with higher or lower manipulability, it modulates the contraction rate

### C.5.2 Reward Design from Contraction Rate

The contraction rate for CLIK is approximately:

```
β(θ) ≈ k · σ_min(J(θ))²
```

where k is the CLIK gain. To maintain fast convergence, the RL policy should keep σ_min(J) high. This suggests a reward component:

```
r_contraction(θ) = σ_min(J(θ))    or    w(θ) = √det(JJᵀ)
```

This is the **manipulability** reward — which now has a principled justification from contraction theory, not just a heuristic one.

### C.5.3 What the RL Agent Can and Cannot Break

**Cannot break** (by construction):
- Task-space convergence direction (the null-space projection ensures Jθ̇_null = 0)
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

```
Constraint: σ_min(J(θ)) ≥ σ_min_threshold
```

Any action z that would violate this constraint (driving the robot toward a singularity) is rejected or penalised. This provides a formal safety guarantee: the task-space tracking error is bounded by:

```
‖ξ_e(t)‖ ≤ e^{−β_min t} ‖ξ_e(0)‖ + δ/β_min
```

where δ accounts for bounded disturbances and β_min = k · σ_min_threshold² is the guaranteed minimum contraction rate.

### C.5.5 Open Questions for Your Paper

These are the research gaps where your contribution lies:

1. **Empirical contraction rates**: How closely do the theoretical contraction rates (from the linearised analysis) match the empirical convergence in simulation? The gap between theory and practice determines how useful the theory is for reward design.

2. **Metric choice for redundant systems**: The task-space contraction is in the 𝔰𝔢(3) metric. The null-space behaviour needs its own metric. What is the right joint-space metric for the full system? Can the RL policy learn a metric implicitly?

3. **Robustness under model mismatch**: The contraction analysis assumes a known kinematic model. When the model is imperfect (as in any real system), how robust is the convergence guarantee? Contraction theory provides tools for this (input-to-state stability of contracting systems), but the bounds may be conservative.

4. **Extending to dynamic (velocity/torque) control**: The CLIK analysis is kinematic (velocity-level). For real robots with inertia, the dynamics introduce additional coupling. Contraction theory can handle this (via the inertia matrix as a contraction metric), but the analysis is more involved.

### Questions — Section C.5

**QC.5.1** Write the complete CLIK + null-space control law for a 7-DOF arm tracking a desired pose T_d(t), with an RL null-space policy π(s) that outputs z. Specify every term, the state s, and the dimensions.

**QC.5.2** Design a reward function for the RL null-space policy that incorporates: (a) manipulability (for contraction rate), (b) distance from joint limits, (c) a task-specific secondary objective (e.g., elbow height). Write the reward as a weighted sum and justify the relative weights.

**QC.5.3** Prove that if σ_min(J(θ(t))) ≥ σ_th > 0 for all t, then the CLIK tracking error satisfies ‖ξ_e(t)‖ ≤ e^{−kt σ²_th} ‖ξ_e(0)‖ (exponential convergence bound). State the assumptions clearly.

**QC.5.4** An RL policy trained in simulation achieves good manipulability on average but occasionally (1% of timesteps) drops σ_min(J) below the threshold. How would you modify the training to eliminate these events? Consider both reward shaping and constrained RL approaches.

**QC.5.5** *Paper-level*: Outline the experiment section of your RA-L paper. What simulated and/or real robot would you use? What baselines would you compare against? What metrics would demonstrate the value of the contraction-theoretic reward design vs naive reward shaping?

---

# Appendix: Additional Notation for These Modules

| Symbol | Meaning |
|---|---|
| q | Unit quaternion (s, v) representing rotation |
| q* | Quaternion conjugate (s, −v) |
| ⊗ | Quaternion multiplication |
| S³ | Unit 3-sphere (space of unit quaternions) |
| SLERP | Spherical Linear Interpolation on S³ |
| Ω | Angle between two quaternions on S³ |
| [A, B] | Lie bracket = AB − BA |
| Ad_g | Big Adjoint: conjugation by group element g |
| ad_ξ | Little adjoint: Lie bracket with ξ |
| BCH | Baker-Campbell-Hausdorff formula |
| β | Contraction rate |
| M(x) | Contraction metric (positive definite matrix field) |
| σ_min(J) | Smallest singular value of Jacobian |
| w(θ) | Manipulability index = √det(JJᵀ) |
| T(ξ) | Left Jacobian of SE(3) (log map derivative) |
| J⁺ | Moore-Penrose pseudoinverse |
| J†_λ | Damped least-squares inverse with damping λ |
| ξ_e | Task-space error in 𝔰𝔢(3) |
| π(s) | RL null-space policy |

---

# Appendix: Key References for These Modules

**Quaternions and SLERP**:
- Shoemake, K. — "Animating Rotation with Quaternion Curves" (SIGGRAPH 1985, the original SLERP paper)
- Kavan, L. et al. — "Dual Quaternions for Rigid Transformation Blending" (extends to SE(3))
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
- Bullo, F. & Murray, R.M. — "Proportional Derivative (PD) Control on the Euclidean Group" (geometric PD on SE(3))
- Pham, Q.C. — "Kinematic Controller Convergence on SE(3)" (contraction-based CLIK analysis)