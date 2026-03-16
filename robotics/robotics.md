# Robotics Mathematics

# Part I — Rotations, $\mathrm{SO}(3)$, and Angular Motion

---

## Chapter 1: Vectors, Dot Products, Cross Products, and Frames

### 1.1 Vectors in $\mathbb{R}^3$

A **vector** is a geometric object — an arrow with magnitude and direction. It exists independently of any coordinate system. What changes when you pick a coordinate frame is the **representation** (the tuple of numbers), not the vector itself.

Given an orthonormal basis $\{\mathbf{e}_1, \mathbf{e}_2, \mathbf{e}_3\}$, any vector $\mathbf{v}$ can be written as:

$$\mathbf{v} = v_1 \mathbf{e}_1 + v_2 \mathbf{e}_2 + v_3 \mathbf{e}_3$$

The numbers $(v_1, v_2, v_3)$ are the **coordinates** of $\mathbf{v}$ in that basis. Switch to a different orthonormal basis $\{\mathbf{f}_1, \mathbf{f}_2, \mathbf{f}_3\}$ and the same physical vector gets different coordinates.

**Key insight**: In robotics, every sensor, every link, every camera lives in its own frame. The same gravity vector has coordinates $[0, 0, -9.81]^\top$ in the world frame but something entirely different in a tilted IMU frame. Understanding this distinction between a vector and its coordinates is the single most important prerequisite for everything that follows.

### 1.2 Dot Product

The dot product of two vectors $\mathbf{a}$ and $\mathbf{b}$ is:

$$\mathbf{a} \cdot \mathbf{b} = a_1 b_1 + a_2 b_2 + a_3 b_3 = \|\mathbf{a}\| \|\mathbf{b}\| \cos\theta$$

where $\theta$ is the angle between them.

**Geometric meaning**: The dot product measures *alignment*. It tells you the signed length of the projection of one vector onto another.

| Value of $\mathbf{a} \cdot \mathbf{b}$ | Meaning |
|---|---|
| Positive | Vectors point in "similar" directions ($\theta < 90°$) |
| Zero | Vectors are **orthogonal** ($\theta = 90°$) |
| Negative | Vectors point in "opposing" directions ($\theta > 90°$) |

**Norm** (length) of a vector:

$$\|\mathbf{v}\| = \sqrt{\mathbf{v} \cdot \mathbf{v}} = \sqrt{v_1^2 + v_2^2 + v_3^2}$$

### 1.3 Cross Product

The cross product $\mathbf{a} \times \mathbf{b}$ produces a new vector that is:
- **Perpendicular** to both $\mathbf{a}$ and $\mathbf{b}$
- Has magnitude $\|\mathbf{a}\| \|\mathbf{b}\| \sin\theta$ (the area of the parallelogram spanned by $\mathbf{a}$ and $\mathbf{b}$)
- Direction given by the **right-hand rule**

$$\mathbf{a} \times \mathbf{b} = \begin{vmatrix} \mathbf{e}_1 & \mathbf{e}_2 & \mathbf{e}_3 \\ a_1 & a_2 & a_3 \\ b_1 & b_2 & b_3 \end{vmatrix} = (a_2 b_3 - a_3 b_2)\mathbf{e}_1 - (a_1 b_3 - a_3 b_1)\mathbf{e}_2 + (a_1 b_2 - a_2 b_1)\mathbf{e}_3$$

**Critical properties**:
- **Anti-commutativity**: $\mathbf{a} \times \mathbf{b} = -(\mathbf{b} \times \mathbf{a})$. Order matters.
- **Not associative**: $\mathbf{a} \times (\mathbf{b} \times \mathbf{c}) \neq (\mathbf{a} \times \mathbf{b}) \times \mathbf{c}$ in general.
- $\mathbf{a} \times \mathbf{a} = \mathbf{0}$ always (a vector is parallel to itself, so $\sin 0 = 0$).

**Robotics connection**: The cross product appears everywhere — torque ($\tau = \mathbf{r} \times \mathbf{F}$), angular velocity effects ($\omega \times \mathbf{v}$), and critically in the construction of skew-symmetric matrices (Chapter 4).

### 1.4 Coordinate Frames and Basis Vectors

An **orthonormal frame** is a set of three mutually perpendicular unit vectors plus an origin. In robotics, we typically label them $\{\hat{\mathbf{x}}, \hat{\mathbf{y}}, \hat{\mathbf{z}}\}$ for each frame.

A frame satisfies:
- **Orthogonality**: $\hat{\mathbf{x}} \cdot \hat{\mathbf{y}} = 0$, $\hat{\mathbf{y}} \cdot \hat{\mathbf{z}} = 0$, $\hat{\mathbf{x}} \cdot \hat{\mathbf{z}} = 0$
- **Unit length**: $\|\hat{\mathbf{x}}\| = \|\hat{\mathbf{y}}\| = \|\hat{\mathbf{z}}\| = 1$
- **Right-handedness**: $\hat{\mathbf{x}} \times \hat{\mathbf{y}} = \hat{\mathbf{z}}$

When you stack these basis vectors as columns, you get a rotation matrix (Chapter 3). This is not a coincidence — it is the entire point.

**Why orthonormal bases matter in robotics**: Every joint, link, sensor, and tool has its own frame. Composing these frames correctly is the core challenge of kinematics.

### 1.5 Changing Coordinates Between Frames

Suppose frame $\{A\}$ and frame $\{B\}$ share the same origin, but their axes are rotated relative to each other. If a vector has coordinates $\mathbf{v}_A$ in frame $A$, then its coordinates in frame $B$ are:

$$\mathbf{v}_B = R_{BA} \cdot \mathbf{v}_A$$

where $R_{BA}$ is the rotation matrix from $A$ to $B$ (we will derive this in Chapter 3). The vector itself hasn't moved — only its description has changed.

### Exercises

**Q1.1** Given $\mathbf{a} = [3, -1, 2]^\top$ and $\mathbf{b} = [1, 4, -2]^\top$, compute: (a) $\mathbf{a} \cdot \mathbf{b}$, (b) $\mathbf{a} \times \mathbf{b}$, (c) the angle between them, (d) verify that $(\mathbf{a} \times \mathbf{b})$ is orthogonal to both $\mathbf{a}$ and $\mathbf{b}$.

**Q1.2** You have a gravity vector $\mathbf{g} = [0, 0, -9.81]^\top$ in the world frame. An IMU is mounted with its z-axis pointing 30° away from vertical (tilted toward the world x-axis). What are the approximate coordinates of $\mathbf{g}$ in the IMU frame? (Hint: this is a rotation about the y-axis.)

**Q1.3** Prove algebraically that $\mathbf{a} \times \mathbf{b} = -(\mathbf{b} \times \mathbf{a})$ using the component formula.

**Q1.4** Why can't you simply subtract coordinate tuples from two different frames to get a meaningful vector? Give a concrete physical example from robotics.

**Q1.5** If $\mathbf{a} \cdot \mathbf{b} = 0$ and $\mathbf{a} \times \mathbf{b} = \mathbf{0}$, what can you conclude about $\mathbf{a}$ and/or $\mathbf{b}$?

---

## Chapter 2: Matrices as Linear Transformations

### 2.1 The Core Idea: Matrices Are Actions

A matrix is not a table of numbers. A matrix is a **function** that maps vectors to vectors. An $m \times n$ matrix $A$ defines a linear map from $\mathbb{R}^n$ to $\mathbb{R}^m$:

$$A : \mathbb{R}^n \to \mathbb{R}^m, \quad \mathbf{v} \mapsto A\mathbf{v}$$

"Linear" means:
- $A(\mathbf{u} + \mathbf{v}) = A\mathbf{u} + A\mathbf{v}$
- $A(\alpha\mathbf{v}) = \alpha(A\mathbf{v})$

**Think of it as**: the matrix tells you where the basis vectors go. If you know what $A$ does to $\mathbf{e}_1$, $\mathbf{e}_2$, $\mathbf{e}_3$, you know what $A$ does to *any* vector, because every vector is a linear combination of basis vectors.

In fact, the columns of $A$ *are* the images of the standard basis vectors:

$$A = \begin{bmatrix} A\mathbf{e}_1 & A\mathbf{e}_2 & A\mathbf{e}_3 \end{bmatrix}$$

### 2.2 Types of 2D Transformations

Consider what different $2 \times 2$ matrices do to the unit square:

**Scaling** (stretch/shrink along axes):

$$S = \begin{bmatrix} s_x & 0 \\ 0 & s_y \end{bmatrix} \quad \text{stretches by } s_x \text{ along } x,\; s_y \text{ along } y$$

**Rotation** (preserves shape, just turns):

$$R(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}$$

**Shear** (tilts one axis):

$$H = \begin{bmatrix} 1 & k \\ 0 & 1 \end{bmatrix} \quad \text{shears the } y\text{-axis by factor } k$$

**Reflection** (mirror across a line):

$$M_x = \begin{bmatrix} 1 & 0 \\ 0 & -1 \end{bmatrix} \quad \text{reflects across } x\text{-axis}$$

The crucial observation: **rotations preserve lengths and angles, and have determinant $+1$**. Reflections preserve lengths but have determinant $-1$. Scaling and shear change lengths.

### 2.3 Eigenvectors and Eigenvalues — Geometrically

An **eigenvector** of $A$ is a nonzero vector $\mathbf{v}$ such that:

$$A\mathbf{v} = \lambda \mathbf{v}$$

That is, $A$ acts on $\mathbf{v}$ by simply scaling it by the factor $\lambda$ (the **eigenvalue**). The direction of $\mathbf{v}$ is preserved (or reversed if $\lambda < 0$).

**Geometric meaning**: Eigenvectors are the **invariant directions** of the transformation. Under $A$, most vectors get both rotated and stretched. But eigenvectors only get stretched (or flipped).

**What eigenvalues tell you**:

| Eigenvalue $\lambda$ | Effect on eigenvector direction |
|---|---|
| $\lambda > 1$ | Stretched away from origin |
| $0 < \lambda < 1$ | Compressed toward origin |
| $\lambda = 1$ | Unchanged (identity direction) |
| $\lambda < 0$ | Direction reversed + scaling |
| $\lambda$ complex | **Rotation/oscillation** — no real invariant direction |

### 2.4 Complex Eigenvalues Mean Rotation

A 2D rotation matrix $R(\theta)$ has eigenvalues:

$$\lambda = \cos\theta \pm i\sin\theta = e^{\pm i\theta}$$

These are complex. The magnitude is $|\lambda| = 1$ (no stretching), and the angle is $\theta$ (the rotation amount). There is **no real eigenvector** because a rotation has no invariant direction (unless $\theta = 0$ or $\pi$).

This is a recurring theme: **complex eigenvalues signal rotational or oscillatory behaviour**. This will reappear in stability analysis (Chapter 14).

### 2.5 Orthogonal Matrices

A square matrix $Q$ is **orthogonal** if:

$$Q^\top Q = Q Q^\top = I \iff Q^{-1} = Q^\top$$

Properties:
- **Preserves lengths**: $\|Q\mathbf{v}\| = \|\mathbf{v}\|$ for all $\mathbf{v}$
- **Preserves angles**: the angle between $Q\mathbf{u}$ and $Q\mathbf{v}$ equals the angle between $\mathbf{u}$ and $\mathbf{v}$
- $\det(Q) = \pm 1$

If $\det(Q) = +1$, $Q$ is a **proper rotation** (element of $\mathrm{SO}(n)$).
If $\det(Q) = -1$, $Q$ is an **improper rotation** (rotation + reflection).

**Why this matters**: Rotation matrices are orthogonal with $\det = +1$. The inverse of a rotation is just its transpose. This makes many robotics computations cheap and numerically stable.

### Exercises

**Q2.1** Write down a $2 \times 2$ matrix that scales by 2 in the x-direction and by 0.5 in the y-direction. What are its eigenvalues and eigenvectors? What does it do to a unit circle?

**Q2.2** Compute the eigenvalues of $R(45°)$. Verify they are complex with magnitude 1. What does this tell you geometrically?

**Q2.3** Show that if $Q$ is orthogonal, then $\|Q\mathbf{v}\| = \|\mathbf{v}\|$ for any $\mathbf{v}$. (Hint: $\|Q\mathbf{v}\|^2 = (Q\mathbf{v})^\top(Q\mathbf{v}) = \ldots$)

**Q2.4** A matrix $A$ has eigenvalues $\lambda_1 = 2$, $\lambda_2 = -0.5$. Without computing anything else, describe qualitatively what this matrix does: does it rotate? stretch? compress? in which directions?

**Q2.5** Why does a shear matrix NOT have orthogonal eigenvectors in general? What property of a matrix guarantees orthogonal eigenvectors?

---

## Chapter 3: Rotation Matrices in 2D and 3D

### 3.1 The 2D Rotation Matrix

A counterclockwise rotation by angle $\theta$ in 2D:

$$R(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}$$

**Derivation**: The standard basis vectors $\mathbf{e}_1 = [1,0]^\top$ and $\mathbf{e}_2 = [0,1]^\top$ are rotated to:
- $\mathbf{e}_1 \to [\cos\theta, \sin\theta]^\top$
- $\mathbf{e}_2 \to [-\sin\theta, \cos\theta]^\top$

These become the columns of $R(\theta)$. That's it — the matrix tells you where the basis vectors go.

### 3.2 3D Rotation Matrices: Elementary Rotations

Rotation about the **z-axis** by angle $\theta$:

$$R_z(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta & 0 \\ \sin\theta & \cos\theta & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

The z-axis is unchanged (it's the rotation axis).

Rotation about the **x-axis** by angle $\theta$:

$$R_x(\theta) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\theta & -\sin\theta \\ 0 & \sin\theta & \cos\theta \end{bmatrix}$$

Rotation about the **y-axis** by angle $\theta$:

$$R_y(\theta) = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix}$$

**Pattern**: In each case, the rotation axis column is $[0,0,\ldots,1,\ldots,0]^\top$ (a standard basis vector), and the other $2 \times 2$ block is a 2D rotation. Note the sign flip in $R_y$ — this is to maintain right-handedness.

### 3.3 Properties of Rotation Matrices

A $3 \times 3$ matrix $R$ is a rotation matrix if and only if:

1. **Orthogonality**: $R^\top R = I$ (equivalently, $R^{-1} = R^\top$)
2. **Proper**: $\det(R) = +1$

Together these define the set $\mathrm{SO}(3)$ (Chapter 5).

**Column interpretation**: The columns of $R$ are the unit basis vectors of the rotated frame, expressed in the original frame. If $R = [\mathbf{r}_1 \mid \mathbf{r}_2 \mid \mathbf{r}_3]$, then:
- $\mathbf{r}_1$ = where the x-axis of the rotated frame points
- $\mathbf{r}_2$ = where the y-axis of the rotated frame points
- $\mathbf{r}_3$ = where the z-axis of the rotated frame points

This is one of the most useful ways to "read" a rotation matrix.

### 3.4 Composition of Rotations

Rotations compose by matrix multiplication:

$$R_{\text{total}} = R_2 \cdot R_1$$

This applies $R_1$ first, then $R_2$. **Order matters** — rotation is non-commutative:

$$R_x(90°) \cdot R_z(90°) \neq R_z(90°) \cdot R_x(90°)$$

You can verify this numerically. This non-commutativity is not just a mathematical nuisance — it reflects deep physical reality. Try rotating a book 90° about x then 90° about z, versus the reverse.

### 3.5 Euler Angles and Their Problems

Any rotation can be decomposed as three successive rotations about specified axes (e.g., ZYX, ZYZ, etc.). These are Euler angles.

**The gimbal lock problem**: When two axes align (middle angle $= \pm 90°$), one degree of freedom is lost. The system becomes singular — infinitely many Euler angle combinations produce the same rotation. This is not a bug in the implementation; it is a topological inevitability of trying to parametrise $\mathrm{SO}(3)$ with three numbers. We will see better representations (axis-angle, exponential coordinates) in Chapters 6–7.

### Exercises

**Q3.1** Build $R_z(30°)$ and $R_z(90°)$ numerically. Verify that $R^\top R = I$ and $\det(R) = 1$ for each.

**Q3.2** Rotate the vector $\mathbf{v} = [1, 0, 0]^\top$ by 90° about the z-axis. Verify the result matches your geometric intuition.

**Q3.3** Compute $R_x(90°) \cdot R_z(90°)$ and $R_z(90°) \cdot R_x(90°)$. Verify they are different. Describe what each composite rotation does to the standard basis vectors.

**Q3.4** Read the columns of the matrix you computed in Q3.3 (either one). Describe the rotated frame's orientation.

**Q3.5** Explain, without computation, why the inverse of a rotation must equal its transpose. (Hint: think about what "undoing" a rotation means geometrically, and what the columns of $R$ represent.)

---

## Chapter 4: Skew-Symmetric Matrices and Angular Velocity

### 4.1 The Cross Product as a Matrix Operation

Given a vector $\omega = [\omega_1, \omega_2, \omega_3]^\top$, we can build the **skew-symmetric matrix**:

$$[\omega]_\times = \begin{bmatrix} 0 & -\omega_3 & \omega_2 \\ \omega_3 & 0 & -\omega_1 \\ -\omega_2 & \omega_1 & 0 \end{bmatrix}$$

This matrix satisfies:

$$[\omega]_\times \mathbf{v} = \omega \times \mathbf{v} \quad \text{for all } \mathbf{v} \in \mathbb{R}^3$$

The cross product — inherently a nonlinear-looking operation — is actually a **linear map** on the second argument. The skew-symmetric matrix is that linear map.

### 4.2 Properties of Skew-Symmetric Matrices

A matrix $S$ is skew-symmetric if $S^\top = -S$. Equivalently, $S_{ij} = -S_{ji}$ for all $i, j$.

Key properties:
- **Trace is zero**: $\operatorname{tr}(S) = 0$
- **Eigenvalues are $\{0, \pm i\|\omega\|\}$** — purely imaginary (plus zero)
- **The zero eigenvalue's eigenvector is $\omega$ itself** — the rotation axis is invariant
- **They generate infinitesimal rotations**: for small angle $\delta\theta$, $R \approx I + [\hat{\omega}]_\times \,\delta\theta$

The space of all $3 \times 3$ skew-symmetric matrices is denoted $\mathfrak{so}(3)$ — the **Lie algebra** of $\mathrm{SO}(3)$. It is a 3-dimensional vector space (parametrised by $\omega_1, \omega_2, \omega_3$).

### 4.3 Angular Velocity and the Kinematic Equation

If a frame is rotating with angular velocity vector $\omega$, the time derivative of its rotation matrix is:

$$\dot{R} = [\omega]_\times R \qquad \text{(body angular velocity in spatial frame)}$$

or equivalently:

$$\dot{R} = R [\omega_b]_\times \qquad \text{(body angular velocity in body frame)}$$

This is the fundamental **rotation kinematics equation**. It says the rate of change of orientation is determined by the angular velocity through the skew-symmetric map.

**Why this is profound**: It directly connects the Lie algebra element (the skew-symmetric matrix $[\omega]_\times$, which lives in the tangent space) to changes in the Lie group element (the rotation matrix $R$, which lives in $\mathrm{SO}(3)$).

### 4.4 From Angular Velocity to the Exponential Map (Preview)

If $\omega$ is **constant**, we can integrate:

$$\dot{R} = [\omega]_\times R, \quad R(0) = I$$

The solution is:

$$R(t) = e^{[\omega]_\times t}$$

This is the **matrix exponential**, which we study in Chapter 6. The skew-symmetric matrix is the "generator" and the matrix exponential "exponentiates" it into a finite rotation.

### Exercises

**Q4.1** For $\omega = [1, 2, 3]^\top$, construct $[\omega]_\times$. Verify that $[\omega]_\times \mathbf{v} = \omega \times \mathbf{v}$ for $\mathbf{v} = [4, 5, 6]^\top$.

**Q4.2** Verify that $[\omega]_\times$ is skew-symmetric: show that $[\omega]_\times^\top = -[\omega]_\times$.

**Q4.3** Compute the eigenvalues of $[\omega]_\times$ for $\omega = [0, 0, 1]^\top$. Verify they are $\{0, +i, -i\}$. What is the eigenvector corresponding to $\lambda = 0$?

**Q4.4** Show that for any skew-symmetric matrix $S$, the diagonal entries must be zero. (Hint: use $S_{ii} = -S_{ii}$.)

**Q4.5** The equation $\dot{R} = [\omega]_\times R$ is an ODE on $\mathrm{SO}(3)$. Why can't we simply write $R(t) = R(0) + [\omega]_\times R(0)\, t$ as a solution (the way we would for a scalar ODE)? What goes wrong?

---

## Chapter 5: Introduction to $\mathrm{SO}(3)$

### 5.1 $\mathrm{SO}(3)$ as a Group

$\mathrm{SO}(3) = \{R \in \mathbb{R}^{3 \times 3} : R^\top R = I,\; \det(R) = 1\}$

This is the **Special Orthogonal group** in 3 dimensions. "Special" means $\det = +1$ (proper rotations only; no reflections). "Orthogonal" means $R^\top R = I$.

It is a **group** under matrix multiplication:
- **Closure**: If $R_1, R_2 \in \mathrm{SO}(3)$, then $R_1 R_2 \in \mathrm{SO}(3)$
- **Identity**: $I \in \mathrm{SO}(3)$
- **Inverse**: $R^{-1} = R^\top \in \mathrm{SO}(3)$
- **Associativity**: $(R_1 R_2) R_3 = R_1 (R_2 R_3)$

The natural operation is **multiplication**, not addition. If you add two rotation matrices, the result is not a rotation matrix.

### 5.2 $\mathrm{SO}(3)$ is Not a Vector Space

This is one of the most important things to internalise:

$$R_1 + R_2 \notin \mathrm{SO}(3) \quad \text{in general}$$

$$\tfrac{1}{2}(R_1 + R_2) \notin \mathrm{SO}(3) \quad \text{in general}$$

You cannot "average" rotations by averaging their matrices. You cannot "interpolate" rotations by linearly interpolating matrices. The set of rotations does not form a flat space — it is curved.

$\mathrm{SO}(3)$ is a **manifold** — a 3-dimensional surface embedded in $\mathbb{R}^9$ (the space of all $3 \times 3$ matrices). It is curved, compact, and has a non-trivial topology.

### 5.3 $\mathrm{SO}(3)$ as a Lie Group

A **Lie group** is a group that is also a smooth manifold, where the group operations (multiplication and inversion) are smooth maps. $\mathrm{SO}(3)$ is a 3-dimensional Lie group.

The associated **Lie algebra** $\mathfrak{so}(3)$ is the tangent space at the identity, which is the space of $3 \times 3$ skew-symmetric matrices (Chapter 4). It is a 3-dimensional vector space — and in this space, addition and scaling *do* work.

The **exponential map** connects the Lie algebra to the Lie group:

$$\exp : \mathfrak{so}(3) \to \mathrm{SO}(3), \quad [\omega]_\times \theta \mapsto R$$

This will be the subject of Chapter 6.

### 5.4 Dimensionality

A $3 \times 3$ matrix has 9 entries. The constraint $R^\top R = I$ gives 6 independent equations (the upper triangle of the symmetric matrix $R^\top R - I$). So $\mathrm{SO}(3)$ has $9 - 6 =$ **3 degrees of freedom**, which matches our physical intuition: orientation in 3D requires 3 parameters.

### 5.5 Topology of $\mathrm{SO}(3)$

$\mathrm{SO}(3)$ is topologically equivalent to $\mathbb{RP}^3$ (real projective 3-space) — the 3-sphere with antipodal points identified. This means:
- It is **compact** (closed and bounded) — there is no "infinity" in rotation space
- It is **not simply connected** — there exist loops in $\mathrm{SO}(3)$ that cannot be smoothly contracted to a point (the "plate trick" / Dirac belt trick)
- No **singularity-free** 3-parameter representation exists (this is why Euler angles always have gimbal lock)

These topological facts have practical consequences: quaternions (which live on $S^3$, the double cover of $\mathrm{SO}(3)$) avoid gimbal lock precisely because they use 4 parameters with one constraint, rather than 3 free parameters.

### Exercises

**Q5.1** Verify that the product of two rotation matrices is a rotation matrix. Take $R = R_z(30°)$ and $S = R_x(45°)$. Compute $RS$. Check that $(RS)^\top(RS) = I$ and $\det(RS) = 1$.

**Q5.2** Take $R = R_z(30°)$ and $S = R_x(45°)$. Compute $R + S$. Verify that the result is NOT a rotation matrix (check the orthogonality condition).

**Q5.3** $\mathrm{SO}(3)$ has 3 degrees of freedom, yet we use 9 numbers (a $3 \times 3$ matrix) to represent it. Name at least two other representations of orientation and state how many numbers each uses and how many constraints each has.

**Q5.4** Explain in your own words why the group operation on rotations must be multiplication rather than addition. What physical absurdity would arise if you tried to "add" two orientations?

**Q5.5** Why does the fact that $\mathrm{SO}(3)$ is not simply connected matter for robotics? (Hint: think about continuous trajectory planning for orientation.)

---

## Chapter 6: Matrix Exponential and Rodrigues Formula

### 6.1 The Matrix Exponential

The matrix exponential is defined by the power series:

$$e^A = I + A + \frac{A^2}{2!} + \frac{A^3}{3!} + \cdots$$

This converges for all square matrices $A$. When $A = [\hat{\omega}]_\times \theta$ (a skew-symmetric matrix scaled by an angle), the result is a rotation matrix in $\mathrm{SO}(3)$.

### 6.2 The Exponential Map: $\mathfrak{so}(3) \to \mathrm{SO}(3)$

Given a unit rotation axis $\hat{\omega}$ ($\|\hat{\omega}\| = 1$) and angle $\theta$, the rotation matrix is:

$$R = \exp([\hat{\omega}]_\times \theta) = I + \sin\theta \, [\hat{\omega}]_\times + (1 - \cos\theta) \, [\hat{\omega}]_\times^2$$

This is the **Rodrigues formula**. It is the closed-form expression for the matrix exponential of a skew-symmetric matrix.

### 6.3 Deriving Rodrigues from the Power Series

The key identity that makes the series collapse is:

$$[\hat{\omega}]_\times^3 = -[\hat{\omega}]_\times \qquad (\text{when } \|\hat{\omega}\| = 1)$$

This means higher powers cycle:

$$\begin{aligned}
[\hat{\omega}]_\times^4 &= -[\hat{\omega}]_\times^2 \\
[\hat{\omega}]_\times^5 &= [\hat{\omega}]_\times \\
[\hat{\omega}]_\times^6 &= [\hat{\omega}]_\times^2 \\
&\;\;\vdots
\end{aligned}$$

Substituting into the exponential series and grouping:

$$e^{[\hat{\omega}]_\times \theta} = I + \left(\theta - \frac{\theta^3}{3!} + \frac{\theta^5}{5!} - \cdots\right) [\hat{\omega}]_\times + \left(\frac{\theta^2}{2!} - \frac{\theta^4}{4!} + \frac{\theta^6}{6!} - \cdots\right) [\hat{\omega}]_\times^2 = I + \sin\theta \, [\hat{\omega}]_\times + (1 - \cos\theta) \, [\hat{\omega}]_\times^2$$

The odd powers produce $\sin\theta$ and the even powers produce $(1 - \cos\theta)$.

### 6.4 Rodrigues in Vector Form

If you want to rotate a vector $\mathbf{v}$ about axis $\hat{\omega}$ by angle $\theta$ without building the full matrix:

$$\mathbf{v}_{\text{rot}} = \mathbf{v} \cos\theta + (\hat{\omega} \times \mathbf{v}) \sin\theta + \hat{\omega}(\hat{\omega} \cdot \mathbf{v})(1 - \cos\theta)$$

This decomposes the rotation into three components:
1. The part of $\mathbf{v}$ in the rotation plane, scaled by $\cos\theta$
2. The perpendicular component in the rotation plane, scaled by $\sin\theta$
3. The part of $\mathbf{v}$ along the rotation axis, which doesn't change

### 6.5 Why Exponential Coordinates Are Better Than Euler Angles

| Property | Euler Angles | Exponential Coordinates |
|---|---|---|
| Parameters | 3 angles ($\varphi, \theta, \psi$) | Axis $\hat{\omega}$ and angle $\theta$ (or 3D vector $\hat{\omega}\theta$) |
| Singularity | Gimbal lock at $\theta = \pm 90°$ | None (the map is smooth everywhere) |
| Geometric meaning | Sequence of rotations about fixed/body axes | Single rotation about a geometric axis |
| Interpolation | Non-trivial, path-dependent | Natural (scale the angle) |
| Composition | Messy trigonometric expressions | Matrix multiplication on $\mathrm{SO}(3)$ |

Exponential coordinates have a **single, clean geometric interpretation**: a rotation by angle $\theta$ about axis $\hat{\omega}$. There is no ambiguity about axis conventions (intrinsic vs extrinsic, which axes, which order).

### 6.6 Practical Notes

- When $\theta = 0$, $R = I$ (no rotation). The Rodrigues formula handles this correctly ($\sin 0 = 0$, $1 - \cos 0 = 0$).
- When $\theta = \pi$, $\sin\theta = 0$, and the formula becomes $R = I + 2[\hat{\omega}]_\times^2$. This is still well-defined.
- Numerically, for very small $\theta$, use the small-angle approximation $R \approx I + \theta[\hat{\omega}]_\times$ to avoid division-by-zero issues in some implementations.

### Exercises

**Q6.1** Take $\hat{\omega} = [0, 0, 1]^\top$ (z-axis) and $\theta = 90°$. Compute $R$ using the Rodrigues formula. Verify it matches $R_z(90°)$.

**Q6.2** Prove the identity $[\hat{\omega}]_\times^3 = -[\hat{\omega}]_\times$ when $\|\hat{\omega}\| = 1$. (Hint: use $[\hat{\omega}]_\times^2 = \hat{\omega}\hat{\omega}^\top - I$, which you can derive from the vector identity $\mathbf{a} \times (\mathbf{b} \times \mathbf{c}) = \mathbf{b}(\mathbf{a} \cdot \mathbf{c}) - \mathbf{c}(\mathbf{a} \cdot \mathbf{b})$.)

**Q6.3** Using the vector form of Rodrigues, rotate $\mathbf{v} = [1, 0, 0]^\top$ about $\hat{\omega} = [0, 0, 1]^\top$ by $\theta = 60°$. Verify the result lies in the xy-plane with the expected angle.

**Q6.4** What rotation matrix does $\theta = 2\pi$ about any axis produce? What about $\theta = \pi$ about $\hat{\omega} = [1, 0, 0]^\top$? Compute both using Rodrigues.

**Q6.5** Explain why the exponential map from $\mathfrak{so}(3)$ to $\mathrm{SO}(3)$ is **not injective** (one-to-one). Give a concrete example of two different Lie algebra elements that map to the same rotation. What does this mean topologically?

---

## Chapter 7: Logarithm Map and Axis-Angle

### 7.1 The Logarithm Map: $\mathrm{SO}(3) \to \mathfrak{so}(3)$

The logarithm map is the **inverse** of the exponential map. Given a rotation matrix $R$, it recovers the axis-angle representation:

$$\log : \mathrm{SO}(3) \to \mathfrak{so}(3), \quad R \mapsto [\hat{\omega}]_\times \theta$$

**Algorithm to extract axis and angle from $R$**:

**Step 1 — Find the angle $\theta$**:

$$\theta = \arccos\!\left(\frac{\operatorname{tr}(R) - 1}{2}\right)$$

This comes from the fact that $\operatorname{tr}(R) = 1 + 2\cos\theta$ (the trace of a rotation matrix).

**Step 2 — Find the axis $\hat{\omega}$**:

*Case $\theta \neq 0$ and $\theta \neq \pi$*:

$$[\hat{\omega}]_\times = \frac{R - R^\top}{2\sin\theta}$$

Then extract $\hat{\omega}$ from the skew-symmetric matrix: $\omega_1 = [\hat{\omega}]_{\times_{32}}$, $\omega_2 = [\hat{\omega}]_{\times_{13}}$, $\omega_3 = [\hat{\omega}]_{\times_{21}}$.

*Case $\theta = 0$*: $R = I$, any axis works (no rotation).

*Case $\theta = \pi$*: $\sin\theta = 0$, so the formula above breaks down. Instead, use $R + I = 2\hat{\omega}\hat{\omega}^\top$, and extract $\hat{\omega}$ as the normalised column of $R + I$ with the largest norm.

### 7.2 The Logarithm as Geometric Error

This is where the logarithm map becomes essential for robotics. If $R_d$ is the desired orientation and $R$ is the current orientation, the **orientation error** is:

$$\begin{aligned}
R_e &= R_d R^\top \qquad \text{(the rotation FROM current TO desired)} \\
\theta_e [\hat{\omega}_e]_\times &= \log(R_e) \qquad \text{(axis-angle of the error)}
\end{aligned}$$

The vector $\theta_e \hat{\omega}_e$ is the **geometric orientation error** — it lives in the Lie algebra and tells you both *which axis* to rotate about and *how much* to rotate. Controllers can directly use this as an error signal.

This is vastly superior to "subtracting" Euler angles, which has no clean geometric meaning and breaks at singularities.

### 7.3 Moving Between Representations

You now have a full circuit:

```
                    exp (Rodrigues)
  Axis-angle (ω̂, θ)  ────────────────►  Rotation matrix R
         ▲                                      │
         │                                      │
         │              log (extract)            │
         └──────────────────────────────────────┘
```

And the skew-symmetric matrix $[\hat{\omega}]_\times \theta$ is the Lie algebra intermediate in both directions.

### 7.4 Connections to Quaternions

Quaternions provide another representation. A unit quaternion $\mathbf{q} = [\cos(\theta/2),\; \sin(\theta/2)\hat{\omega}]$ represents the same rotation as axis-angle $(\hat{\omega}, \theta)$. The conversions are:

- Quaternion $\to$ Axis-angle: $\theta = 2\arccos(q_0)$, $\hat{\omega} = \mathbf{q}_{\text{vec}} / \sin(\theta/2)$
- Quaternion $\to$ Matrix: well-known formula involving $q_0, q_1, q_2, q_3$
- Axis-angle $\to$ Quaternion: $q_0 = \cos(\theta/2)$, $\mathbf{q}_{\text{vec}} = \sin(\theta/2)\hat{\omega}$

Quaternions avoid gimbal lock and are computationally efficient for composition (quaternion multiplication) and interpolation (SLERP). However, for kinematics and control, the Lie algebra / exponential coordinate framework is more natural.

### 7.5 Part I Summary

| Ch. | Core Concept | Key Takeaway |
|---|---|---|
| 1 | Vectors, frames | A vector $\neq$ its coordinates; frames are everywhere |
| 2 | Linear transformations | Matrices are actions; eigenvalues reveal their character |
| 3 | Rotation matrices | $R \in \mathrm{SO}(3)$: orthogonal, $\det = +1$, columns = rotated basis |
| 4 | Skew-symmetric matrices | $[\omega]_\times$ encodes angular velocity; $\dot{R} = [\omega]_\times R$ |
| 5 | $\mathrm{SO}(3)$ as a group | Rotations compose by multiplication; $\mathrm{SO}(3)$ is curved |
| 6 | Exponential map + Rodrigues | $R = I + \sin\theta \, [\hat{\omega}]_\times + (1-\cos\theta)[\hat{\omega}]_\times^2$ |
| 7 | Logarithm map | Recovers axis-angle; gives geometric orientation error |

### Exercises

**Q7.1** Given $R = R_z(60°)$, extract the rotation axis and angle using the logarithm algorithm. Verify you recover $\hat{\omega} = [0, 0, 1]^\top$ and $\theta = 60°$.

**Q7.2** Compute $R_e = R_z(90°) \cdot R_x(45°)^\top$. What does this "error rotation" represent? Extract its axis and angle.

**Q7.3** Explain why the formula $\theta = \arccos((\operatorname{tr}(R)-1)/2)$ breaks down (or becomes ambiguous) when $\theta = \pi$. What happens to the trace, and how do you handle this case?

**Q7.4** You have a rotation matrix $R$ with eigenvalues $\{1, e^{i\alpha}, e^{-i\alpha}\}$. What is the rotation angle in terms of $\alpha$? What is the rotation axis in terms of $R$'s eigenvectors?

**Q7.5** *Comprehensive*: Given $R_1 = R_x(30°)$ and $R_2 = R_y(45°)$, compute $R = R_2 R_1$. Then extract the equivalent axis-angle representation. Finally, reconstruct $R$ from that axis-angle via Rodrigues and verify.

---

# Part II — $\mathrm{SE}(3)$, Twists, Jacobians, and Control

---

## Chapter 8: Homogeneous Transforms and $\mathrm{SE}(3)$

### 8.1 Rigid Body Pose = Rotation + Translation

A rigid body's **pose** in 3D space has 6 degrees of freedom: 3 for orientation (rotation) and 3 for position (translation). We combine them into a single mathematical object.

If frame $\{B\}$ has orientation $R \in \mathrm{SO}(3)$ and origin position $\mathbf{p} \in \mathbb{R}^3$ relative to frame $\{A\}$, then a point $\mathbf{q}$ with coordinates $\mathbf{q}_B$ in frame $\{B\}$ has coordinates in frame $\{A\}$:

$$\mathbf{q}_A = R \cdot \mathbf{q}_B + \mathbf{p}$$

This is an **affine transformation** — a linear transformation (rotation) followed by a translation.

### 8.2 Homogeneous Transformation Matrix

To make this composition a pure matrix multiplication, we embed everything in 4D:

$$T = \begin{bmatrix} R & \mathbf{p} \\ \mathbf{0}^\top & 1 \end{bmatrix} \in \mathbb{R}^{4 \times 4}$$

Then:

$$\begin{bmatrix} \mathbf{q}_A \\ 1 \end{bmatrix} = \begin{bmatrix} R & \mathbf{p} \\ \mathbf{0}^\top & 1 \end{bmatrix} \begin{bmatrix} \mathbf{q}_B \\ 1 \end{bmatrix}$$

Points are represented with a trailing 1 (homogeneous coordinates).

### 8.3 $\mathrm{SE}(3)$: The Special Euclidean Group

$\mathrm{SE}(3)$ is the set of all rigid body transformations in 3D:

$$\mathrm{SE}(3) = \left\{ T \in \mathbb{R}^{4 \times 4} : T = \begin{bmatrix} R & \mathbf{p} \\ \mathbf{0}^\top & 1 \end{bmatrix},\; R \in \mathrm{SO}(3),\; \mathbf{p} \in \mathbb{R}^3 \right\}$$

It is a **6-dimensional Lie group** under matrix multiplication.

**Group operations**:
- **Composition**: $T_2 T_1 = \begin{bmatrix} R_2 R_1 & R_2 \mathbf{p}_1 + \mathbf{p}_2 \\ \mathbf{0}^\top & 1 \end{bmatrix}$
- **Identity**: $I_4$ ($4 \times 4$ identity)
- **Inverse**: $T^{-1} = \begin{bmatrix} R^\top & -R^\top \mathbf{p} \\ \mathbf{0}^\top & 1 \end{bmatrix}$

Note the inverse: the translation part is not simply $-\mathbf{p}$, it is $-R^\top \mathbf{p}$. This is because you must "un-rotate" the translation.

### 8.4 Composition of Poses

If $T_{AB}$ describes frame $\{B\}$ relative to $\{A\}$, and $T_{BC}$ describes frame $\{C\}$ relative to $\{B\}$, then:

$$T_{AC} = T_{AB} \cdot T_{BC}$$

This is the fundamental **kinematic chain** operation. In a serial robot with $n$ links, the end-effector pose is:

$$T_{0n} = T_{01} \cdot T_{12} \cdot T_{23} \cdots T_{(n-1)n}$$

Each $T_{(i-1)i}$ depends on the joint variable $q_i$.

### 8.5 $\mathrm{SE}(3)$ vs $\mathrm{SO}(3)$: What's New

| Property | $\mathrm{SO}(3)$ | $\mathrm{SE}(3)$ |
|---|---|---|
| Dimension | 3 | 6 |
| Matrix size | $3 \times 3$ | $4 \times 4$ |
| Represents | Orientation | Full pose (orientation + position) |
| Lie algebra | $\mathfrak{so}(3)$: $3 \times 3$ skew-symmetric | $\mathfrak{se}(3)$: $4 \times 4$ twist matrices |
| Algebra dimension | 3 | 6 |

### Exercises

**Q8.1** Construct the homogeneous transform $T$ for a 90° rotation about z followed by a translation of $[1, 2, 3]^\top$. Apply it to the point $\mathbf{q} = [1, 0, 0]^\top$.

**Q8.2** Compute $T^{-1}$ for the transform in Q8.1 using the formula $T^{-1} = [R^\top \;\; {-R^\top \mathbf{p}};\; \mathbf{0}^\top \;\; 1]$. Verify that $T \cdot T^{-1} = I$.

**Q8.3** Explain the difference between:
- Rotating a vector: $\mathbf{v}' = R \cdot \mathbf{v}$
- Transforming a point's frame: $\mathbf{q}_A = T_{AB} \cdot \mathbf{q}_B$ (in homogeneous coordinates)

When would you use one vs the other?

**Q8.4** You have $T_{AB}$ and $T_{BC}$. You want to express a point known in frame $\{C\}$ in frame $\{A\}$. Write the formula. Now, you want to express a point known in frame $\{A\}$ in frame $\{C\}$. Write that formula.

**Q8.5** Why is the inverse translation $-R^\top \mathbf{p}$ rather than just $-\mathbf{p}$? Give a geometric argument.

---

## Chapter 9: Twists and Screw Motion

### 9.1 Velocity of a Rigid Body

A rigid body moving in space has both **linear velocity** $\mathbf{v} \in \mathbb{R}^3$ and **angular velocity** $\omega \in \mathbb{R}^3$. Together, these form a 6D velocity vector called a **twist**:

$$V = \begin{bmatrix} \mathbf{v} \\ \omega \end{bmatrix} \in \mathbb{R}^6$$

(Convention varies — some texts put $\omega$ first. We use the $[\mathbf{v};\; \omega]$ convention here, matching Murray-Li-Sastry and the modern robotics convention.)

### 9.2 Spatial vs Body Twist

There are two natural frames in which to express the twist:

**Spatial (world-frame) twist** $V^s$: velocities expressed in the fixed world frame.

**Body twist** $V^b$: velocities expressed in the body's own frame.

They are related by the **Adjoint** (Chapter 13):

$$V^s = \operatorname{Ad}_T \cdot V^b$$

### 9.3 Screw Interpretation (Chasles' Theorem)

**Chasles' theorem**: Any rigid body motion can be decomposed into a rotation about some axis in space combined with a translation along that same axis. This combined motion is called a **screw motion**.

A **screw** is defined by:
- An **axis** (a line in space): point $\mathbf{q}$ on the axis, direction $\hat{\omega}$
- A **pitch** $h$: the ratio of linear to angular displacement along/about the axis

The twist for a screw motion is:

$$V = \dot{\theta} \begin{bmatrix} -\hat{\omega} \times \mathbf{q} + h\hat{\omega} \\ \hat{\omega} \end{bmatrix} = \dot{\theta} \begin{bmatrix} \mathbf{v}_0 \\ \hat{\omega} \end{bmatrix}$$

**Special cases**:
- **Pure rotation** ($h = 0$): The body rotates about the screw axis with no translation along it.
- **Pure translation** ($h = \infty$, $\omega = 0$): The body translates along a direction with no rotation.

### 9.4 Why Twists Matter for Robotics

Each **joint** in a robot defines a screw motion:

- A **revolute joint** is a pure rotation screw ($h = 0$) about the joint axis
- A **prismatic joint** is a pure translation screw ($h = \infty$) along the joint axis

This means joint velocities map directly to twists, and the entire velocity kinematics of a serial manipulator can be expressed in terms of twists.

### 9.5 Twist as a Lie Algebra Element

A twist can be written as a $4 \times 4$ matrix in $\mathfrak{se}(3)$:

$$[\hat{V}] = \begin{bmatrix} [\omega]_\times & \mathbf{v} \\ \mathbf{0}^\top & 0 \end{bmatrix} \in \mathfrak{se}(3)$$

This is the Lie algebra of $\mathrm{SE}(3)$. The exponential of this matrix gives a finite rigid body transformation (Chapter 10).

### Exercises

**Q9.1** Write the twist for: (a) pure translation along the x-axis at speed 1, (b) pure rotation about the z-axis at rate 1 rad/s, (c) a screw motion about the z-axis through the origin with pitch $h = 0.5$.

**Q9.2** A revolute joint is located at point $\mathbf{q} = [1, 0, 0]^\top$ with axis $\hat{\omega} = [0, 0, 1]^\top$. Compute the joint twist (the 6D twist associated with unit joint velocity).

**Q9.3** Explain physically why every rigid body motion can be decomposed into a screw motion (Chasles' theorem). Why isn't "rotation about one axis plus translation along a different axis" a counterexample?

**Q9.4** Write the $4 \times 4$ matrix $[\hat{V}]$ for the twist $V = [1, 0, 0, 0, 0, 1]^\top$ (translation in x + rotation about z). Verify it is in $\mathfrak{se}(3)$ (upper-left block is skew-symmetric, last row is zeros).

**Q9.5** Why is a prismatic joint described as a screw with infinite pitch? What happens to the angular velocity component? What does the "axis" of the screw represent in this case?

---

## Chapter 10: Exponential Coordinates on $\mathrm{SE}(3)$

### 10.1 The Exponential Map for Rigid Body Motion

Just as the Rodrigues formula exponentiates a skew-symmetric matrix to get a rotation, we can exponentiate a twist to get a rigid body transformation:

$$T = \exp([\hat{\xi}]\theta)$$

where $[\hat{\xi}] \in \mathfrak{se}(3)$ is the $4 \times 4$ twist matrix and $\theta$ is the joint displacement.

### 10.2 Closed-Form Formula

For a twist $\xi = [\mathbf{v};\; \omega]$ with $\|\omega\| = 1$ (revolute case):

$$\exp([\hat{\xi}]\theta) = \begin{bmatrix} e^{[\omega]_\times \theta} & \left(I\theta + (1-\cos\theta)[\omega]_\times + (\theta-\sin\theta)[\omega]_\times^2\right)\mathbf{v} \\ \mathbf{0}^\top & 1 \end{bmatrix}$$

The rotation block is just the Rodrigues formula. The translation block is a more complex expression that accounts for the coupling between rotation and translation in a screw motion.

For the **pure translation** case ($\omega = 0$, $\|\mathbf{v}\| = 1$):

$$\exp([\hat{\xi}]\theta) = \begin{bmatrix} I & \mathbf{v}\theta \\ \mathbf{0}^\top & 1 \end{bmatrix}$$

This is just a translation by $\mathbf{v}\theta$.

### 10.3 Joint Motion as an Exponential

Each joint in a serial robot can be modelled as:

$$T_{\text{joint}}(\theta) = \exp([\hat{\xi}_i] \theta_i)$$

where $\xi_i$ is the joint's screw axis (defined at the zero configuration) and $\theta_i$ is the joint variable (angle for revolute, displacement for prismatic).

This is the foundation of the **Product of Exponentials** formula (Chapter 11).

### 10.4 Geometric Interpretation

For a revolute joint about axis $\hat{\omega}$ through point $\mathbf{q}$:
- The twist is $\xi = [-\hat{\omega} \times \mathbf{q};\; \hat{\omega}]$
- $\exp(\hat{\xi}\theta)$ rotates everything about the line through $\mathbf{q}$ in direction $\hat{\omega}$ by angle $\theta$
- Points on the axis don't move
- Points off the axis trace circular arcs

For a prismatic joint along direction $\hat{\mathbf{v}}$:
- The twist is $\xi = [\hat{\mathbf{v}};\; \mathbf{0}]$
- $\exp(\hat{\xi}\theta)$ translates everything by $\theta\hat{\mathbf{v}}$

### Exercises

**Q10.1** For a revolute joint about the z-axis through the origin ($\mathbf{q} = \mathbf{0}$, $\hat{\omega} = [0,0,1]^\top$), compute the twist $\xi$ and the $4 \times 4$ matrix $\exp(\hat{\xi}\theta)$. Verify the rotation block matches $R_z(\theta)$ and the translation is zero.

**Q10.2** For a revolute joint about the z-axis through the point $\mathbf{q} = [L, 0, 0]^\top$ (like a link of length $L$), compute the twist $\xi$. What does the resulting $\exp(\hat{\xi}\theta)$ look like?

**Q10.3** Explain conceptually why the translation part of $\exp(\hat{\xi}\theta)$ for a revolute joint is not simply $\mathbf{p} = \theta \mathbf{v}$, but involves the more complex expression with $[\omega]_\times$ and $[\omega]_\times^2$ terms. What coupling is being captured?

**Q10.4** Show that for $\theta \to 0$, $\exp([\hat{\xi}]\theta) \approx I + [\hat{\xi}]\theta$. Why is this the "infinitesimal motion" approximation?

**Q10.5** Why is the exponential map from $\mathfrak{se}(3)$ to $\mathrm{SE}(3)$ **surjective** (onto) but not injective? Give examples of different algebra elements mapping to the same group element.

---

## Chapter 11: Product of Exponentials (POE) for Forward Kinematics

### 11.1 The POE Formula

For a serial manipulator with $n$ joints, the forward kinematics is:

$$T(\theta) = e^{[\hat{\xi}_1]\theta_1} \cdot e^{[\hat{\xi}_2]\theta_2} \cdots e^{[\hat{\xi}_n]\theta_n} \cdot M$$

where:
- $\xi_i$ is the twist of joint $i$, defined in the **zero configuration**
- $\theta_i$ is the joint variable
- $M = T(0)$ is the end-effector pose when all joint angles are zero (the **reference configuration**)

### 11.2 How to Set Up POE

**Step 1**: Place the robot in its zero configuration (all joint angles = 0).

**Step 2**: For each joint $i$, identify:
- The joint type (revolute or prismatic)
- The joint axis direction $\hat{\omega}_i$ (in the world/spatial frame)
- A point $\mathbf{q}_i$ on the joint axis (for revolute joints)

**Step 3**: Compute the twist:
- Revolute: $\xi_i = [-\hat{\omega}_i \times \mathbf{q}_i;\; \hat{\omega}_i]$
- Prismatic: $\xi_i = [\hat{\mathbf{v}}_i;\; \mathbf{0}]$

**Step 4**: Record $M$ as the end-effector pose at $\theta = 0$.

### 11.3 POE vs DH Parameters

| Aspect | DH | POE |
|---|---|---|
| Frame assignment | Every link needs a frame; complex rules for where to place them | Only need joint axes and the zero-config pose |
| Convention | Multiple conventions (standard, modified, Craig, etc.) | Unique |
| Singularity-free | Depends on Euler angle choices | Intrinsically uses exponentials |
| Geometric clarity | Frame-to-frame transforms; can be hard to visualise | Each factor is a physical joint motion |
| Implementation | Chain of $4 \times 4$ transforms from 4 params each | Chain of exponentials from twists |

**POE advantages**: No need to assign intermediate frames. The physical meaning is transparent — each exponential "sweeps" the rest of the chain through the joint's screw motion. The zero-configuration pose $M$ anchors everything.

### 11.4 Example: 2R Planar Arm via POE

Consider a 2-link planar arm in the xy-plane:
- Link 1: length $L_1$, revolute joint at the origin about z
- Link 2: length $L_2$, revolute joint at $(L_1, 0, 0)$ about z
- End-effector at $(L_1 + L_2, 0, 0)$ in zero configuration

**Zero configuration**: $\theta_1 = \theta_2 = 0$, arm stretched along x-axis.

$M$ (end-effector at zero config):

$$M = \begin{bmatrix} I & [L_1 + L_2,\; 0,\; 0]^\top \\ \mathbf{0}^\top & 1 \end{bmatrix}$$

**Joint 1**: revolute about z through origin $\to$ $\xi_1 = [0, 0, 0, 0, 0, 1]^\top$

**Joint 2**: revolute about z through $(L_1, 0, 0)$ $\to$ $\xi_2 = [0, L_1, 0, 0, 0, 1]^\top$
(because $-\hat{\omega} \times \mathbf{q} = -[0,0,1] \times [L_1,0,0] = [0, L_1, 0]$)

**Forward kinematics**:

$$T(\theta_1, \theta_2) = e^{\hat{\xi}_1 \theta_1} \cdot e^{\hat{\xi}_2 \theta_2} \cdot M$$

You can multiply this out and recover the standard 2R forward kinematics: $x = L_1\cos\theta_1 + L_2\cos(\theta_1+\theta_2)$, $y = L_1\sin\theta_1 + L_2\sin(\theta_1+\theta_2)$.

### Exercises

**Q11.1** For the 2R planar arm above, compute $\exp(\hat{\xi}_1 \theta_1)$ in closed form. Verify it's a rotation about z with associated translation.

**Q11.2** Set up the POE for a 3R planar arm (three revolute joints in the plane, link lengths $L_1, L_2, L_3$). Write all three twists and $M$.

**Q11.3** Explain why, in the POE formula, the twists are defined in the **zero configuration** rather than in the current configuration. What would go wrong if you used current-configuration twists?

**Q11.4** Compare: for the 2R arm, derive the forward kinematics using (a) the POE formula and (b) direct geometry (trig). Verify they give the same result. Which was easier to set up?

**Q11.5** How does POE handle a prismatic joint in a serial chain? Give an example of a 2-joint robot with one revolute and one prismatic joint, and set up the POE.

---

## Chapter 12: Jacobians, Singularities, and Velocity Kinematics

### 12.1 The Velocity Kinematics Problem

Given the forward kinematics map $T(\theta)$, the **velocity kinematics** relates joint velocities $\dot{\theta}$ to the end-effector twist $V$:

$$V = J(\theta) \dot{\theta}$$

where $J(\theta)$ is the **Jacobian** — a $6 \times n$ matrix (6 rows for the twist, $n$ columns for $n$ joints). Each column $J_i$ of the Jacobian is the twist generated by joint $i$ at unit velocity, expressed in the appropriate frame.

### 12.2 Space Jacobian vs Body Jacobian

**Space Jacobian** $J^s$: end-effector twist expressed in the spatial (world) frame.

$$V^s = J^s(\theta) \dot{\theta}$$

The $i$-th column of $J^s$ is the twist of joint $i$, transformed to account for the motion of joints 1 through $i-1$:

$$J^s_i = \operatorname{Ad}_{T_1 \cdots T_{i-1}} \xi_i$$

where $T_k = \exp(\hat{\xi}_k \theta_k)$.

**Body Jacobian** $J^b$: end-effector twist expressed in the end-effector (body) frame.

The two are related by: $J^s = \operatorname{Ad}_T \cdot J^b$ where $T$ is the full forward kinematics.

### 12.3 Singularities

A **singularity** occurs when the Jacobian loses rank:

$$\operatorname{rank}(J(\theta)) < \min(6, n)$$

At a singularity:
- Some end-effector twist directions become **unachievable** regardless of joint velocities
- The null space of $J^\top$ becomes non-trivial — there are wrenches that produce zero joint torques
- Inverse kinematics algorithms break down (the pseudo-inverse blows up)

**Types of singularities**:
- **Workspace boundary**: arm fully extended or fully folded
- **Internal**: joint axes align, reducing the effective DOF

### 12.4 The 2R Planar Arm Jacobian

For a 2R arm with end-effector at $(x, y)$:

$$\begin{aligned}
x &= L_1 \cos\theta_1 + L_2 \cos(\theta_1 + \theta_2) \\
y &= L_1 \sin\theta_1 + L_2 \sin(\theta_1 + \theta_2)
\end{aligned}$$

The (analytical) Jacobian:

$$J = \begin{bmatrix} -L_1 \sin\theta_1 - L_2 \sin(\theta_1+\theta_2) & -L_2 \sin(\theta_1+\theta_2) \\ L_1 \cos\theta_1 + L_2 \cos(\theta_1+\theta_2) & L_2 \cos(\theta_1+\theta_2) \end{bmatrix}$$

**Singularity**: $\det(J) = L_1 L_2 \sin\theta_2 = 0$, so $\theta_2 = 0$ or $\pi$. At $\theta_2 = 0$, the arm is fully extended — it cannot move radially. At $\theta_2 = \pi$, it is fully folded — same problem.

### 12.5 Null Space of the Jacobian

For a **redundant** manipulator ($n > 6$), the Jacobian has a non-trivial **null space**: joint velocities in the null space produce zero end-effector motion.

$$\dot{\theta} = J^+ V + (I - J^+ J) \mathbf{z}$$

where $J^+$ is the pseudoinverse, $V$ is the desired end-effector twist, and $\mathbf{z}$ is an arbitrary vector. The term $(I - J^+ J)\mathbf{z}$ is the **null-space component** — it moves the joints without affecting the end-effector.

This is the foundation of **redundancy resolution**: you can use the extra freedom to optimise secondary objectives (avoid obstacles, maximise manipulability, stay away from joint limits) while still tracking the desired end-effector motion.

**This is directly connected to your research on null-space policies** — the RL agent learns to choose $\mathbf{z}$ optimally.

### 12.6 Manipulability

The **manipulability index** quantifies how "well-conditioned" the robot is at a given configuration:

$$w(\theta) = \sqrt{\det(J J^\top)}$$

At a singularity, $w = 0$. Away from singularities, higher $w$ means the robot can move more easily in all directions. Maximising manipulability is a classic null-space objective.

### Exercises

**Q12.1** Derive the $2 \times 2$ Jacobian of the 2R planar arm by differentiating the FK equations. Find the singular configurations.

**Q12.2** At the singularity $\theta_2 = 0$ (arm extended), what direction can the end-effector NOT move? Verify this by examining the rank-1 Jacobian.

**Q12.3** For a 3R planar arm (3 joints, 2D end-effector), the Jacobian is $2 \times 3$. What is the dimension of the null space? What does this physically mean?

**Q12.4** Explain why maximising manipulability $w(\theta)$ is a good null-space objective. What happens if you don't use the null space at all?

**Q12.5** The formula $\dot{\theta} = J^+ V + (I - J^+ J)\mathbf{z}$ decomposes the joint velocity into two orthogonal components. Prove that $J(I - J^+ J) = 0$ (the null-space component produces zero end-effector velocity).

---

## Chapter 13: Adjoint Map, Body vs Space Jacobians, and Wrenches

### 13.1 The Adjoint Map

Given a rigid body transformation $T = [R \;\; \mathbf{p};\; \mathbf{0}^\top \;\; 1] \in \mathrm{SE}(3)$, the **Adjoint map** $\operatorname{Ad}_T$ transforms twists between frames:

$$\operatorname{Ad}_T = \begin{bmatrix} R & [\mathbf{p}]_\times R \\ \mathbf{0} & R \end{bmatrix} \in \mathbb{R}^{6 \times 6}$$

If $V_b$ is the twist in frame $\{B\}$ and $T_{AB}$ is the transform from $\{B\}$ to $\{A\}$, then:

$$V_a = \operatorname{Ad}_{T_{AB}} \cdot V_b$$

The Adjoint is the proper way to change the reference frame of a twist. You cannot simply multiply by $R$ or $T$ — you need the full $6 \times 6$ Adjoint because linear and angular velocities are coupled through the $[\mathbf{p}]_\times R$ term.

### 13.2 Why We Need the Adjoint

Consider a body rotating about a remote axis. An observer at the origin sees both angular velocity (the rotation) and linear velocity (because the body's origin is sweeping through space). An observer riding on the body sees only the angular velocity. The Adjoint captures this coupling.

**Properties of the Adjoint**:
- $\operatorname{Ad}_{T_1 T_2} = \operatorname{Ad}_{T_1} \cdot \operatorname{Ad}_{T_2}$ (it's a group homomorphism)
- $\operatorname{Ad}_{T^{-1}} = (\operatorname{Ad}_T)^{-1}$
- $\operatorname{Ad}_I = I_6$

### 13.3 Space Jacobian vs Body Jacobian (Detailed)

**Space Jacobian**: Each column represents a joint's screw axis as seen from the world frame:

$$J^s_i(\theta) = \operatorname{Ad}_{\exp(\hat{\xi}_1\theta_1)\cdots\exp(\hat{\xi}_{i-1}\theta_{i-1})} \cdot \xi_i$$

For $i = 1$, the first column is just $\xi_1$ (the first joint's screw doesn't depend on any other joints). For $i > 1$, the screw of joint $i$ has been "moved" by the preceding joints.

**Body Jacobian**: Each column represents a joint's screw axis as seen from the end-effector frame. More complex to write directly but related by:

$$J^b = \operatorname{Ad}_{T^{-1}} J^s \qquad \text{or equivalently} \qquad J^s = \operatorname{Ad}_T J^b$$

### 13.4 Wrenches and Force Duality

A **wrench** is a 6D generalised force:

$$F = \begin{bmatrix} \mathbf{f} \\ \tau \end{bmatrix} \in \mathbb{R}^6$$

where $\mathbf{f}$ is the linear force and $\tau$ is the torque/moment.

The **power** generated by a twist $V$ acting against a wrench $F$ is:

$$P = V^\top F = \mathbf{v}^\top \mathbf{f} + \omega^\top \tau$$

This is a scalar invariant (independent of frame).

### 13.5 Static Force Relationship

The wrench at the end-effector $F$ maps to joint torques $\tau_{\text{joint}}$ via:

$$\tau_{\text{joint}} = J^\top F$$

This is the **transpose Jacobian relationship**. It is the dual of $V = J\dot{\theta}$.

**Why $J^\top$?**: This follows from power conservation. The power at the joints must equal the power at the end-effector:

$$P = \dot{\theta}^\top \tau_{\text{joint}} = V^\top F = (J\dot{\theta})^\top F = \dot{\theta}^\top J^\top F$$

Since this must hold for all $\dot{\theta}$, we get $\tau_{\text{joint}} = J^\top F$.

### Exercises

**Q13.1** Compute the Adjoint $\operatorname{Ad}_T$ for $T = [R_z(90°),\; [1,0,0]^\top;\; \mathbf{0}^\top,\; 1]$. Write out the full $6 \times 6$ matrix.

**Q13.2** A body twist $V_b = [0, 0, 0, 0, 0, 1]^\top$ (pure rotation about body z-axis). Using the $\operatorname{Ad}_T$ from Q13.1, compute the spatial twist $V_s$. Interpret the result — why does a linear velocity component appear?

**Q13.3** For a 2R planar arm at configuration $\theta_1 = 90°$, $\theta_2 = 0$, write the Space Jacobian. Apply a unit force in the x-direction at the end-effector ($F = [1, 0, 0, 0, 0, 0]^\top$). Compute the joint torques via $\tau = J^\top F$.

**Q13.4** Prove that the Adjoint satisfies $\operatorname{Ad}_{T_1 T_2} = \operatorname{Ad}_{T_1} \operatorname{Ad}_{T_2}$. (Hint: show both sides produce the same result when applied to an arbitrary twist.)

**Q13.5** In operational space control, the controller applies an end-effector wrench $F$ and maps it to joint torques via $\tau = J^\top F$. What happens at a singularity? Can you still apply any desired wrench?

---

## Chapter 14: Stability, Eigenvalues, and Geometric Control

### 14.1 Linear Systems and Stability

The linearised dynamics of many robotic systems take the form:

$$\dot{x} = Ax$$

The **eigenvalues** of $A$ determine the system's behavior:

| Eigenvalue type | System behavior |
|---|---|
| All $\operatorname{Re}(\lambda) < 0$ | **Asymptotically stable** — converges to origin |
| Any $\operatorname{Re}(\lambda) > 0$ | **Unstable** — diverges |
| $\operatorname{Re}(\lambda) = 0$ (and others negative) | **Marginally stable** — oscillates without growing or shrinking |

### 14.2 Complex Eigenvalues and Oscillation

If $A$ has complex eigenvalue $\lambda = \sigma \pm i\omega$, the corresponding mode behaves as:

$$e^{\sigma t} [\cos(\omega t) \text{ and } \sin(\omega t) \text{ terms}]$$

- $\sigma < 0$: oscillation with decaying envelope (stable spiral)
- $\sigma > 0$: oscillation with growing envelope (unstable spiral)
- $\sigma = 0$: sustained oscillation (center)
- $\omega = 0$ (real eigenvalue): pure exponential growth/decay, no oscillation

**Example**: Eigenvalues $\lambda = -1 \pm 2i$ mean $\sigma = -1$ (stable, converging), $\omega = 2$ (oscillation frequency 2 rad/s). The system spirals inward.

### 14.3 Stability for Robotic Systems

For a manipulator tracking a desired trajectory, the error dynamics (after linearisation around the trajectory) take the form:

$$\dot{e} = A_{\text{cl}}\, e$$

where $e$ is the tracking error and $A_{\text{cl}}$ is the closed-loop system matrix. The controller must be designed so that all eigenvalues of $A_{\text{cl}}$ have negative real parts.

**PD control example**: For a second-order system $\ddot{e} + K_d \dot{e} + K_p e = 0$, the characteristic polynomial is $s^2 + K_d s + K_p = 0$ with eigenvalues:

$$\lambda = \frac{-K_d \pm \sqrt{K_d^2 - 4K_p}}{2}$$

Both eigenvalues have negative real parts when $K_p > 0$ and $K_d > 0$. If $K_d^2 < 4K_p$, the eigenvalues are complex, giving oscillatory convergence (underdamped). If $K_d^2 > 4K_p$, both eigenvalues are real and negative (overdamped).

### 14.4 Why Geometric Error Matters

**The naive approach** (and why it fails):

$$e_{\text{orientation}} = R_{\text{desired}} - R_{\text{current}} \quad \leftarrow \text{WRONG}$$

The difference of two rotation matrices is NOT a rotation matrix. It does not live in any meaningful space. Controllers built on this error have poor convergence properties and can fail entirely near large errors.

**The geometric approach**:

$$R_e = R_d \cdot R_{\text{current}}^\top \qquad \text{(the rotation FROM current TO desired)}$$

Then extract the error vector:

$$e_R = \operatorname{vee}(\log(R_e)) \qquad \text{(the axis-angle vector)}$$

This error:
- Lives in the Lie algebra (a vector space) — you CAN do linear algebra on it
- Has a clear geometric meaning: rotate about axis $\hat{e}$ by angle $\|e_R\|$ to get from current to desired
- Is well-defined for all orientations (no gimbal lock)
- Goes to zero if and only if $R_{\text{current}} = R_{\text{desired}}$

### 14.5 Full Pose Error on $\mathrm{SE}(3)$

For full 6-DOF tracking, the pose error is:

$$T_e = T_d \cdot T_{\text{current}}^{-1}$$

And the error twist:

$$\xi_e = \operatorname{vee}(\log(T_e))$$

This gives a 6D error vector in $\mathfrak{se}(3)$ that can drive a controller:

$$V_{\text{command}} = K \cdot \xi_e$$

This is the basis of **CLIK (Closed-Loop Inverse Kinematics)** on $\mathrm{SE}(3)$ — which is central to your research direction.

### 14.6 Lyapunov Stability (Brief Preview)

For nonlinear systems (which robots are), eigenvalue analysis only works locally (near equilibria). **Lyapunov's direct method** provides a more general tool:

Find a function $V(x)$ (a "generalised energy") such that:
- $V(x) > 0$ for all $x \neq 0$
- $\dot{V}(x) \leq 0$ along trajectories

If $\dot{V} < 0$ strictly, the system is asymptotically stable. This approach works for large-scale nonlinear analysis and is the foundation of provably stable robotic controllers.

### Exercises

**Q14.1** A system $\dot{x} = Ax$ has eigenvalues $\lambda = -1 \pm 2i$. Describe the trajectory: does it converge or diverge? Does it oscillate? What is the approximate settling behaviour?

**Q14.2** A PD controller gives closed-loop dynamics $\ddot{e} + 4\dot{e} + 20e = 0$. Compute the eigenvalues. Is the system stable? Underdamped or overdamped? What is the oscillation frequency?

**Q14.3** Explain, in your own words, why the "error" $R_d - R$ does not make geometric sense. What mathematical structure is violated?

**Q14.4** Given $R_{\text{current}} = R_z(30°)$ and $R_{\text{desired}} = R_z(90°)$, compute $R_e = R_d R_{\text{current}}^\top$. Extract the axis and angle. Verify the axis and angle make intuitive sense.

**Q14.5** In CLIK, the control law is $\dot{\theta} = J^+(\theta) \cdot K \cdot \xi_e$ where $\xi_e = \operatorname{vee}(\log(T_d T(\theta)^{-1}))$. Explain each component: what does $J^+$ do? What does $K$ do? What does the log map provide? When does this controller fail?

---

# Appendix: Notation Reference

| Symbol | Meaning |
|---|---|
| $\mathrm{SO}(3)$ | Special Orthogonal group — all 3D rotations |
| $\mathrm{SE}(3)$ | Special Euclidean group — all rigid body poses |
| $\mathfrak{so}(3)$ | Lie algebra of $\mathrm{SO}(3)$ — $3 \times 3$ skew-symmetric matrices |
| $\mathfrak{se}(3)$ | Lie algebra of $\mathrm{SE}(3)$ — $4 \times 4$ twist matrices |
| $R$ | Rotation matrix $\in \mathrm{SO}(3)$ |
| $T$ | Homogeneous transformation $\in \mathrm{SE}(3)$ |
| $[\omega]_\times$ | Skew-symmetric matrix of vector $\omega$ |
| $\exp$ | Matrix exponential (maps Lie algebra $\to$ Lie group) |
| $\log$ | Matrix logarithm (maps Lie group $\to$ Lie algebra) |
| $\operatorname{vee}(\cdot)$ | Extracts the vector from a skew or twist matrix |
| $\hat{\omega}$ | Unit rotation axis ($\|\hat{\omega}\| = 1$) |
| $\xi$ | Twist (6D: $[\mathbf{v};\; \omega]$) |
| $J$ | Jacobian ($6 \times n$ matrix) |
| $J^+$ | Moore-Penrose pseudoinverse of $J$ |
| $\operatorname{Ad}_T$ | Adjoint map ($6 \times 6$, transforms twists between frames) |
| $F$ | Wrench (6D: $[\mathbf{f};\; \tau]$) |
| $V$ | Twist/spatial velocity (6D: $[\mathbf{v};\; \omega]$) |
| $M$ | Reference configuration (end-effector pose at $\theta = 0$) |
| $\theta$ | Joint angle or rotation angle (context-dependent) |

---

# Appendix: Suggested References

- **Murray, Li, Sastry** — *A Mathematical Introduction to Robotic Manipulation* (the canonical Lie-group kinematics text; freely available online)
- **Lynch & Park** — *Modern Robotics: Mechanics, Planning, and Control* (excellent POE-first exposition; companion videos on YouTube)
- **Siciliano et al.** — *Robotics: Modelling, Planning and Control* (comprehensive, more classical DH-heavy but thorough)
- **Selig** — *Geometric Fundamentals of Robotics* (deep Lie-group/screw theory perspective)
- **Hall** — *Lie Groups, Lie Algebras, and Representations* (if you want the pure math foundations)
