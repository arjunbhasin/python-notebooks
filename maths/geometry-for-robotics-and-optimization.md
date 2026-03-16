# Geometry for Robotics and Optimization

## Lie Groups, Differential Geometry, and Matrix Manifolds

---

> **Overview.** This document is a self-contained introduction to three pillars of modern geometric methods: *differential geometry of curves and surfaces*, *Lie groups and Lie algebras*, and *optimization on matrix manifolds*. The unifying theme is the geometry of constrained spaces — rotation matrices, rigid-body transformations, orthonormal frames — that arise naturally in robotics, computer vision, and machine learning. We develop the theory from first principles, building from the concrete (curves in $\mathbb{R}^3$) to the abstract (Riemannian gradient descent on matrix manifolds).
>
> **Prerequisites.** Linear algebra (eigendecomposition, SVD), multivariable calculus, and basic familiarity with ordinary differential equations. Some exposure to group theory and topology is helpful but not required.

---

# Part I — Differential Geometry of Curves and Surfaces

---

## 1. Parametric Curves and Arc Length

**Definition 1.1 (Parametric Curve).** A *parametric curve* in $\mathbb{R}^3$ is a smooth map $\alpha : I \to \mathbb{R}^3$, where $I \subset \mathbb{R}$ is an open interval. The curve is *regular* if $\alpha'(t) \neq 0$ for all $t \in I$.

Regularity ensures the curve has a well-defined tangent direction at every point — it never "stops" or develops a cusp.

**Definition 1.2 (Arc Length).** The *arc length* of $\alpha$ from $a$ to $t$ is

$$s(t) = \int_a^t \|\alpha'(\tau)\| \, d\tau.$$

Since $\alpha$ is regular, $ds/dt = \|\alpha'(t)\| > 0$, so $s(t)$ is strictly increasing and hence invertible.

**Definition 1.3 (Arc-Length Parametrization).** A curve $\beta$ is said to be *parametrized by arc length* (or *unit-speed*) if $\|\beta'(s)\| = 1$ for all $s$.

**Proposition 1.4.** Every regular curve admits an arc-length reparametrization. Given $\alpha(t)$ with arc-length function $s(t)$, the reparametrized curve $\beta(s) = \alpha(t(s))$ satisfies $\|\beta'(s)\| = 1$.

*Proof.* By the chain rule, $\beta'(s) = \alpha'(t(s)) \cdot t'(s)$. Since $s'(t) = \|\alpha'(t)\|$, we have $t'(s) = 1/\|\alpha'(t(s))\|$, giving $\|\beta'(s)\| = \|\alpha'(t)\| / \|\alpha'(t)\| = 1$. $\square$

**Example 1.5 (Circle in the Plane).** Consider the circle $\alpha(t) = (\cos t, \sin t, 0)$ for $t \in [0, 2\pi]$.

- **Tangent vector:** $\alpha'(t) = (-\sin t, \cos t, 0)$, so $\|\alpha'(t)\| = 1$ for all $t$. This curve is *already* unit-speed.
- **Arc length:** $s(t) = \int_0^t 1 \, d\tau = t$. The parameter $t$ *is* the arc length.

Now consider the same circle traversed at non-uniform speed: $\gamma(t) = (\cos t^2, \sin t^2, 0)$. Then $\gamma'(t) = 2t(-\sin t^2, \cos t^2, 0)$, so $\|\gamma'(t)\| = 2|t|$. The speed varies with $t$, but the geometric path is the same circle. The arc-length function is $s(t) = \int_0^t 2\tau \, d\tau = t^2$, and reparametrizing by $s$ recovers the unit-speed circle.

**Example 1.6 (Straight Line).** The line $\alpha(t) = (1 + 3t, \, 2 + 4t, \, 0)$ has $\alpha'(t) = (3, 4, 0)$ and $\|\alpha'(t)\| = 5$. The arc length from $t = 0$ to $t = 1$ is $s = 5$. To reparametrize by arc length, set $t = s/5$, giving $\beta(s) = (1 + 3s/5, \, 2 + 4s/5, \, 0)$ with $\|\beta'(s)\| = 1$.

**Remark 1.7.** Arc-length parametrization decouples *geometry* (the shape of the path) from *timing* (the speed of traversal). In robotics, this separation is fundamental: one first designs the geometric path, then assigns a velocity profile along it.

---

## 2. Curvature and Torsion

For a unit-speed curve $\beta(s)$, the acceleration $\beta''(s)$ is purely geometric — it captures how fast the curve turns.

**Definition 2.1 (Curvature, unit-speed).** The *curvature* of a unit-speed curve is

$$\kappa(s) = \|\beta''(s)\|.$$

For a general (non-unit-speed) parametrization, we need a formula that is invariant under reparametrization.

**Theorem 2.2 (General Curvature and Torsion Formulas).** For a regular curve $\alpha(t)$:

$$\kappa(t) = \frac{\|\alpha'(t) \times \alpha''(t)\|}{\|\alpha'(t)\|^3}$$

$$\tau(t) = \frac{(\alpha'(t) \times \alpha''(t)) \cdot \alpha'''(t)}{\|\alpha'(t) \times \alpha''(t)\|^2}$$

*Proof sketch.* Write $\alpha' = \dot{s} \, T$ where $\dot{s} = \|\alpha'\|$. Differentiating:

$$\alpha'' = \ddot{s} \, T + \dot{s}^2 \kappa \, N$$

where $N$ is the unit normal (defined below). Taking the cross product:

$$\alpha' \times \alpha'' = \dot{s}^3 \kappa \, B$$

where $B = T \times N$ is the binormal. Taking norms yields the curvature formula. For torsion, one differentiates once more and projects onto $B$. $\square$

**Example 2.3 (Circular Helix).** The helix $\alpha(t) = (a\cos t, \, a\sin t, \, bt)$ with $a > 0$ has

$$\kappa = \frac{a}{a^2 + b^2}, \qquad \tau = \frac{b}{a^2 + b^2}.$$

Both are constant. The helix is the unique curve (up to rigid motion) with constant nonzero curvature and constant torsion.

**Example 2.5 (Circle — Curvature by Hand).** For the circle $\alpha(t) = (r\cos t, \, r\sin t, \, 0)$:

- $\alpha'(t) = (-r\sin t, \, r\cos t, \, 0)$, so $\|\alpha'\| = r$
- $\alpha''(t) = (-r\cos t, \, -r\sin t, \, 0)$
- $\alpha' \times \alpha'' = (0, \, 0, \, r^2\sin^2 t + r^2\cos^2 t) = (0, 0, r^2)$, so $\|\alpha' \times \alpha''\| = r^2$

Therefore $\kappa = r^2 / r^3 = 1/r$. A circle of radius $r$ has constant curvature $1/r$. Smaller circles curve more sharply — this matches intuition. A straight line has $\kappa = 0$ (infinite radius).

Since $\alpha'''(t) = (r\sin t, \, -r\cos t, \, 0)$, we get $(\alpha' \times \alpha'') \cdot \alpha''' = 0$, hence $\tau = 0$. The circle is planar, as expected.

**Example 2.6 (Parabola $y = x^2$).** Parametrize as $\alpha(t) = (t, t^2, 0)$. Then:

- $\alpha' = (1, 2t, 0)$, $\alpha'' = (0, 2, 0)$
- $\alpha' \times \alpha'' = (0, 0, 2)$
- $\kappa(t) = 2 / (1 + 4t^2)^{3/2}$

At $t = 0$ (the vertex): $\kappa = 2$, corresponding to the osculating circle of radius $1/2$. As $|t| \to \infty$, the parabola straightens out and $\kappa \to 0$.

**Remark 2.7.** A curve is planar if and only if $\tau \equiv 0$. Curvature measures the rate of turning within the osculating plane; torsion measures how the osculating plane itself rotates.

---

## 3. The Frenet-Serret Frame

**Definition 3.1 (Frenet-Serret Frame).** For a unit-speed curve $\beta(s)$ with $\kappa(s) > 0$, the *Frenet-Serret frame* is the ordered orthonormal basis $\{T, N, B\}$:

| Vector | Name | Definition |
|:---:|:---:|:---:|
| $T$ | Unit tangent | $T = \beta'$ |
| $N$ | Principal normal | $N = T' / \|T'\|$ |
| $B$ | Binormal | $B = T \times N$ |

The frame $\{T, N, B\}$ is a right-handed orthonormal basis at each point of the curve.

**Theorem 3.2 (Frenet-Serret Equations).** The derivatives of the frame vectors satisfy

$$\frac{d}{ds}\begin{bmatrix} T \\ N \\ B \end{bmatrix} = \begin{bmatrix} 0 & \kappa & 0 \\ -\kappa & 0 & \tau \\ 0 & -\tau & 0 \end{bmatrix} \begin{bmatrix} T \\ N \\ B \end{bmatrix}$$

*Proof.* Since $\{T, N, B\}$ is orthonormal, we may write $T' = a_{12} N + a_{13} B$, and similarly for $N'$ and $B'$. Differentiating the orthonormality conditions (e.g., $T \cdot T = 1$ implies $T' \cdot T = 0$) forces the coefficient matrix to be *skew-symmetric*. By definition, $a_{12} = \kappa$ and $a_{23} = -\tau$, while $a_{13} = 0$ follows from $T' = \kappa N$. $\square$

**Remark 3.3 (Connection to Lie Theory).** The skew-symmetry of the Frenet-Serret matrix is no coincidence. Assembling the frame vectors as columns of a rotation matrix $R(s) = [T \; N \; B] \in SO(3)$, the Frenet-Serret equations become

$$\frac{dR}{ds} = R \, \Omega(s), \qquad \Omega(s) = \begin{bmatrix} 0 & -\kappa & 0 \\ \kappa & 0 & -\tau \\ 0 & \tau & 0 \end{bmatrix} \in \mathfrak{so}(3)$$

This is an ODE on the Lie group $SO(3)$, with the angular velocity $\Omega$ living in the Lie algebra $\mathfrak{so}(3)$ — a connection we will make precise in Part II.

**Theorem 3.4 (Fundamental Theorem of Space Curves).** Given smooth functions $\kappa(s) > 0$ and $\tau(s)$, there exists a unit-speed curve $\beta : I \to \mathbb{R}^3$ with curvature $\kappa$ and torsion $\tau$. Moreover, this curve is unique up to a rigid motion (rotation and translation) of $\mathbb{R}^3$.

*Proof idea.* The Frenet-Serret equations constitute a linear ODE system on $SO(3)$ with prescribed coefficients $\kappa(s)$ and $\tau(s)$. By the Picard-Lindelöf theorem, a unique solution $R(s)$ exists for any initial condition $R(0) \in SO(3)$. The curve is recovered by integrating $\beta(s) = \int_0^s T(\sigma) \, d\sigma$. Different initial conditions correspond to rigid motions. $\square$

**Example 3.5 (Frenet-Serret Frame of the Helix).** For the helix $\alpha(t) = (a\cos t, \, a\sin t, \, bt)$ with $c = \sqrt{a^2 + b^2}$:

- $T = \frac{1}{c}(-a\sin t, \, a\cos t, \, b)$ — tilted tangent spiraling upward
- $N = (-\cos t, \, -\sin t, \, 0)$ — always points toward the helix axis (horizontal)
- $B = \frac{1}{c}(b\sin t, \, -b\cos t, \, a)$ — tilted binormal

The Frenet-Serret matrix has constant entries $\kappa = a/c^2$ and $\tau = b/c^2$, so the frame rotates at a constant rate as we move along the helix.

**Example 3.6 (Frenet Frame of a Circle).** For a circle of radius $r$ in the $xy$-plane, $\alpha(s) = (r\cos(s/r), \, r\sin(s/r), \, 0)$:

- $T = (-\sin(s/r), \, \cos(s/r), \, 0)$ — tangent to the circle
- $N = (-\cos(s/r), \, -\sin(s/r), \, 0)$ — points inward toward the center
- $B = (0, 0, 1)$ — constant, pointing up

Here $\kappa = 1/r$ and $\tau = 0$. The Frenet-Serret equations reduce to $T' = (1/r) N$ and $N' = -(1/r) T$, which is simply rotation in the $TN$-plane at rate $1/r$.

---

## 4. Surfaces and the First Fundamental Form

**Definition 4.1 (Parametric Surface).** A *parametric surface* is a smooth map $\sigma : U \subset \mathbb{R}^2 \to \mathbb{R}^3$. It is *regular* if the partial derivatives $\sigma_u$ and $\sigma_v$ are linearly independent at every point.

**Definition 4.2 (First Fundamental Form).** The *first fundamental form* (or *metric tensor*) of a surface $\sigma$ is defined by the coefficients

$$E = \sigma_u \cdot \sigma_u, \qquad F = \sigma_u \cdot \sigma_v, \qquad G = \sigma_v \cdot \sigma_v$$

In matrix form, the metric tensor is $g = \begin{bmatrix} E & F \\ F & G \end{bmatrix}$.

The first fundamental form encodes all *intrinsic* metric information: lengths, angles, and areas on the surface.

**Proposition 4.3 (Arc Length on a Surface).** For a curve $\gamma(t) = \sigma(u(t), v(t))$ lying on the surface,

$$L = \int_a^b \sqrt{E\dot{u}^2 + 2F\dot{u}\dot{v} + G\dot{v}^2} \; dt$$

*Proof.* $\gamma' = \sigma_u \dot{u} + \sigma_v \dot{v}$, so $\|\gamma'\|^2 = E\dot{u}^2 + 2F\dot{u}\dot{v} + G\dot{v}^2$. $\square$

**Proposition 4.4 (Area Element).**

$$dA = \sqrt{EG - F^2} \; du \, dv = \|\sigma_u \times \sigma_v\| \; du \, dv$$

**Example 4.5 (Sphere).** For $\sigma(\theta, \phi) = (r\sin\theta\cos\phi, \, r\sin\theta\sin\phi, \, r\cos\theta)$:

$$E = r^2, \quad F = 0, \quad G = r^2 \sin^2\theta$$

The area element is $dA = r^2 \sin\theta \, d\theta \, d\phi$, yielding total area $4\pi r^2$.

**Example 4.6 (Cylinder).** For the cylinder $\sigma(u, v) = (\cos u, \, \sin u, \, v)$:

- $\sigma_u = (-\sin u, \cos u, 0)$, $\sigma_v = (0, 0, 1)$
- $E = 1$, $F = 0$, $G = 1$

The metric tensor is $g = I_2$ — identical to the flat plane! This means a cylinder is *intrinsically flat*: you can unroll it into a plane without stretching. An ant walking on a cylinder would measure the same distances as on a flat sheet.

**Arc length** on the cylinder: $L = \int \sqrt{\dot{u}^2 + \dot{v}^2} \, dt$, exactly as in $\mathbb{R}^2$.

**Area** of a cylindrical strip with $u \in [0, 2\pi]$, $v \in [0, h]$: $A = \int_0^h \int_0^{2\pi} 1 \, du \, dv = 2\pi h$.

**Example 4.7 (Plane — the Simplest Surface).** For $\sigma(u, v) = (u, v, 0)$, we have $E = 1$, $F = 0$, $G = 1$. The arc-length formula gives $L = \int \sqrt{\dot{u}^2 + \dot{v}^2} \, dt$, which is just the Euclidean distance formula. The area element is $dA = du \, dv$ — no correction factor needed.

**Remark 4.8.** The first fundamental form defines a *Riemannian metric* on the surface. Two surfaces with the same $E, F, G$ (as functions of coordinates) are *isometric* — they have identical intrinsic geometry, even if they look different in $\mathbb{R}^3$.

---

## 5. Second Fundamental Form and Curvatures

The first fundamental form captures intrinsic geometry; the *second* fundamental form captures how the surface curves within the ambient space $\mathbb{R}^3$.

**Definition 5.1 (Unit Normal and Second Fundamental Form).** The *unit normal* to the surface is

$$\hat{n} = \frac{\sigma_u \times \sigma_v}{\|\sigma_u \times \sigma_v\|}$$

The *second fundamental form* has coefficients

$$L = \sigma_{uu} \cdot \hat{n}, \qquad M = \sigma_{uv} \cdot \hat{n}, \qquad N = \sigma_{vv} \cdot \hat{n}$$

**Definition 5.2 (Shape Operator).** The *shape operator* (or *Weingarten map*) is the $2 \times 2$ matrix

$$S = \begin{bmatrix} E & F \\ F & G \end{bmatrix}^{-1} \begin{bmatrix} L & M \\ M & N \end{bmatrix}$$

It encodes the directional change of the normal vector as one moves along the surface. The shape operator is self-adjoint with respect to the first fundamental form.

**Definition 5.3 (Principal, Gaussian, and Mean Curvatures).** The eigenvalues $\kappa_1, \kappa_2$ of $S$ are the *principal curvatures*. The corresponding eigenvectors give the *principal directions* — the directions of maximum and minimum normal curvature. From these:

$$K = \kappa_1 \kappa_2 = \frac{LN - M^2}{EG - F^2} \qquad \text{(Gaussian curvature)}$$

$$H = \frac{\kappa_1 + \kappa_2}{2} = \frac{EN - 2FM + GL}{2(EG - F^2)} \qquad \text{(Mean curvature)}$$

**Theorem 5.4 (Gauss's Theorema Egregium).** *The Gaussian curvature $K$ depends only on the first fundamental form coefficients $E, F, G$ and their first and second partial derivatives. It is an intrinsic invariant of the surface.*

This is one of the most remarkable results in differential geometry: $K$ is defined extrinsically (via the normal vector and the embedding in $\mathbb{R}^3$), yet it turns out to be intrinsic. A consequence is that $K$ is preserved under isometric deformations — bending a surface without stretching it cannot change $K$.

*Proof sketch.* One expresses $K$ entirely in terms of the Christoffel symbols $\Gamma^i_{jk}$ (which depend only on $g_{ij}$) via the Gauss equations:

$$K = \frac{1}{E}\left(\frac{\partial \Gamma^2_{11}}{\partial v} - \frac{\partial \Gamma^2_{12}}{\partial u} + \Gamma^1_{11}\Gamma^2_{12} - \Gamma^1_{12}\Gamma^2_{11} + \Gamma^2_{11}\Gamma^2_{22} - (\Gamma^2_{12})^2\right)$$

(for the case $F = 0$). Since the Christoffel symbols depend only on $E, F, G$, so does $K$. $\square$

**Example 5.5.**

| Surface | $K$ | $H$ | Geometry |
|:---:|:---:|:---:|:---:|
| Sphere of radius $r$ | $1/r^2$ | $1/r$ | Positive curvature everywhere |
| Plane | $0$ | $0$ | Flat |
| Saddle $z = xy$ | $< 0$ | $0$ | Negative curvature |
| Cylinder | $0$ | $1/(2r)$ | Intrinsically flat |
| Torus | Variable sign | Variable | Positive outside, negative inside |

**Example 5.7 (Sphere — Full Computation).** For $\sigma(\theta, \phi) = (r\sin\theta\cos\phi, \, r\sin\theta\sin\phi, \, r\cos\theta)$:

*First fundamental form:* $E = r^2$, $F = 0$, $G = r^2\sin^2\theta$ (computed in Example 4.5).

*Unit normal:* $\hat{n} = (\sin\theta\cos\phi, \, \sin\theta\sin\phi, \, \cos\theta)$ — points radially outward.

*Second fundamental form:* $\sigma_{\theta\theta} = (-r\sin\theta\cos\phi, \, -r\sin\theta\sin\phi, \, -r\cos\theta) = -r\hat{n}$, so $L = \sigma_{\theta\theta} \cdot \hat{n} = -r$. Similarly, $M = 0$ and $N = -r\sin^2\theta$.

*Shape operator:*

$$S = \frac{1}{r^2\sin^2\theta}\begin{bmatrix} \sin^2\theta & 0 \\ 0 & r^2 \end{bmatrix} \begin{bmatrix} -r & 0 \\ 0 & -r\sin^2\theta \end{bmatrix} = \begin{bmatrix} -1/r & 0 \\ 0 & -1/r \end{bmatrix}$$

*Principal curvatures:* $\kappa_1 = \kappa_2 = -1/r$ (or $1/r$ with inward normal convention). Every direction is a principal direction — the sphere curves equally in all directions.

*Gaussian curvature:* $K = 1/r^2$. *Mean curvature:* $H = 1/r$.

**Example 5.8 (Saddle Surface $z = xy$).** Parametrize as $\sigma(u,v) = (u, v, uv)$.

- $\sigma_u = (1, 0, v)$, $\sigma_v = (0, 1, u)$
- $E = 1 + v^2$, $F = uv$, $G = 1 + u^2$
- At the origin $(0,0)$: $E = 1$, $F = 0$, $G = 1$, $L = 0$, $M = 1/1 = 1$, $N = 0$

$$K = \frac{LN - M^2}{EG - F^2} = \frac{0 - 1}{1 - 0} = -1$$

The negative Gaussian curvature reflects the saddle shape: the surface curves upward in one direction and downward in the perpendicular direction.

**Remark 5.9 (Gauss-Bonnet Theorem).** For a compact surface $M$ without boundary,

$$\int_M K \, dA = 2\pi \chi(M)$$

where $\chi(M)$ is the Euler characteristic — a topological invariant. For a sphere, $\chi = 2$, recovering $\int K \, dA = 4\pi$. For a torus, $\chi = 0$, so the total Gaussian curvature vanishes: positive and negative regions cancel exactly. This profound result links local geometry to global topology.

---

## 6. Geodesics

**Definition 6.1 (Geodesic).** A *geodesic* on a surface is a curve that is locally length-minimizing. Equivalently, its acceleration has no tangential component — it "goes as straight as possible" while remaining on the surface.

To find geodesics, we minimize the arc-length functional

$$L[\gamma] = \int_a^b \sqrt{E\dot{u}^2 + 2F\dot{u}\dot{v} + G\dot{v}^2} \; dt$$

over curves $\gamma(t) = (u(t), v(t))$ with fixed endpoints.

**Definition 6.2 (Christoffel Symbols).** The *Christoffel symbols of the second kind* are

$$\Gamma^i_{jk} = \frac{1}{2} g^{il}\left(\frac{\partial g_{lj}}{\partial u^k} + \frac{\partial g_{lk}}{\partial u^j} - \frac{\partial g_{jk}}{\partial u^l}\right)$$

where $g_{ij} = \begin{bmatrix} E & F \\ F & G \end{bmatrix}$, $g^{ij}$ is its matrix inverse, and we use coordinates $u^1 = u$, $u^2 = v$.

**Theorem 6.3 (Geodesic Equation).** A curve $\gamma(t) = (u^1(t), u^2(t))$ parametrized proportionally to arc length is a geodesic if and only if

$$\ddot{u}^i + \Gamma^i_{jk} \, \dot{u}^j \dot{u}^k = 0, \qquad i = 1, 2$$

where summation over repeated indices is implied.

*Proof.* Apply the Euler-Lagrange equations to the energy functional $E[\gamma] = \frac{1}{2}\int g_{jk}\dot{u}^j \dot{u}^k \, dt$ (which shares critical points with the length functional for curves of constant speed). The Euler-Lagrange equation for $u^i$ is

$$\frac{d}{dt}\left(g_{ij}\dot{u}^j\right) - \frac{1}{2}\frac{\partial g_{jk}}{\partial u^i}\dot{u}^j \dot{u}^k = 0$$

Expanding the time derivative using the product rule and solving for $\ddot{u}^i$ via $g^{il}$ yields the geodesic equation with the Christoffel symbols as defined above. $\square$

**Example 6.4 (Geodesics on the Sphere).** On the unit sphere with $E = 1$, $F = 0$, $G = \sin^2\theta$, the geodesic equations reduce to

$$\ddot{\theta} - \sin\theta\cos\theta \, \dot{\phi}^2 = 0$$
$$\ddot{\phi} + 2\cot\theta \, \dot{\theta}\dot{\phi} = 0$$

The solutions are *great circles* — intersections of the sphere with planes through the origin.

**Example 6.5 (Geodesics on a Flat Plane).** For $\sigma(u,v) = (u, v, 0)$: $E = G = 1$, $F = 0$, all Christoffel symbols vanish ($\Gamma^i_{jk} = 0$). The geodesic equations become $\ddot{u} = 0$, $\ddot{v} = 0$, giving straight lines $u(t) = u_0 + a t$, $v(t) = v_0 + bt$. This confirms our intuition: "straightest paths" on a flat surface are ordinary straight lines.

**Example 6.6 (Geodesics on a Cylinder).** Since the cylinder has $E = G = 1$, $F = 0$ (Example 4.6), all Christoffel symbols are again zero. Geodesics satisfy $\ddot{u} = 0$, $\ddot{v} = 0$, giving $u(t) = u_0 + at$, $v(t) = v_0 + bt$. In 3D, these are *helices* on the cylinder (including straight lines along the axis when $a = 0$, and circles around the cylinder when $b = 0$). When you unroll the cylinder, these helices become straight lines — consistent with the cylinder being intrinsically flat.

**Example 6.7 (Why Airplane Routes are Curved on a Map).** On the Earth (a sphere), geodesics are great circles. The flight from London to Tokyo follows a great circle that passes over the Arctic — appearing as a curve on a flat (Mercator) map. This "curved" route is actually the shortest path on the sphere. The Mercator projection distorts distances near the poles, making the true shortest path look bent.

**Remark 6.8 (Geodesic Shooting).** In practice, one solves the geodesic equation as an initial value problem: given a starting point $(u_0, v_0)$ and an initial direction $(\dot{u}_0, \dot{v}_0)$, integrate the ODE forward. This *geodesic shooting* method is the basis of the exponential map, which we will encounter again in the context of Lie groups (Section 10) and manifold optimization (Section 19).

---

# Part II — Lie Groups and Lie Algebras

---

## 7. Group Theory Foundations

**Definition 7.1 (Group).** A *group* $(G, \cdot)$ is a set $G$ equipped with a binary operation $\cdot$ satisfying:

| Axiom | Statement |
|:---:|:---|
| Closure | $\forall \, a, b \in G: \; a \cdot b \in G$ |
| Associativity | $\forall \, a, b, c \in G: \; (a \cdot b) \cdot c = a \cdot (b \cdot c)$ |
| Identity | $\exists \, e \in G \; \forall \, a \in G: \; e \cdot a = a \cdot e = a$ |
| Inverse | $\forall \, a \in G, \; \exists \, a^{-1} \in G: \; a \cdot a^{-1} = a^{-1} \cdot a = e$ |

**Definition 7.2 (Matrix Lie Group).** A *matrix Lie group* $G$ is a closed subgroup of the general linear group $GL(n, \mathbb{R})$. "Closed" means that if a sequence of matrices $A_k \in G$ converges to $A$, then $A \in G$. The closure condition ensures $G$ is a smooth manifold, inheriting differentiable structure from $\mathbb{R}^{n \times n}$.

**Table 7.3 (Important Matrix Groups).**

| Group | Definition | Dimension |
|:---:|:---|:---:|
| $GL(n)$ | Invertible $n \times n$ matrices ($\det A \neq 0$) | $n^2$ |
| $SL(n)$ | Special linear group ($\det A = 1$) | $n^2 - 1$ |
| $O(n)$ | Orthogonal group ($R^T R = I$) | $n(n-1)/2$ |
| $SO(n)$ | Special orthogonal ($R^T R = I$, $\det R = 1$) | $n(n-1)/2$ |
| $SE(n)$ | Special Euclidean (rotation + translation) | $n(n+1)/2$ |

**Definition 7.4 (Lie Algebra).** The *Lie algebra* $\mathfrak{g}$ of a matrix Lie group $G$ is the tangent space at the identity element $e$:

$$\mathfrak{g} = T_e G = \left\{ \left.\frac{d}{dt}\right|_{t=0} \gamma(t) \;\middle|\; \gamma(0) = e, \; \gamma(t) \in G \text{ for all } t \right\}$$

The Lie algebra is a vector space equipped with a bilinear, skew-symmetric operation $[\cdot, \cdot] : \mathfrak{g} \times \mathfrak{g} \to \mathfrak{g}$ called the *Lie bracket*, which for matrix groups is the commutator $[A, B] = AB - BA$.

**Example 7.5 (Integers Under Addition).** The simplest group: $(\mathbb{Z}, +)$. Closure: sum of two integers is an integer. Associativity: $(a+b)+c = a+(b+c)$. Identity: $e = 0$. Inverse: $a^{-1} = -a$. This group is *abelian* (commutative), unlike most matrix groups.

**Example 7.6 ($SO(2)$ — Rotations of the Plane).** The simplest Lie group: $2 \times 2$ rotation matrices

$$R(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}$$

Check: $R(\theta)^T R(\theta) = I$ and $\det R(\theta) = \cos^2\theta + \sin^2\theta = 1$. Composition: $R(\alpha)R(\beta) = R(\alpha + \beta)$ — angles add. Inverse: $R(\theta)^{-1} = R(-\theta)$. This group is 1-dimensional and abelian (rotations in 2D commute). Its Lie algebra is $\mathfrak{so}(2) = \left\{\begin{bmatrix} 0 & -\omega \\ \omega & 0 \end{bmatrix} : \omega \in \mathbb{R}\right\}$, which is also 1-dimensional.

**Example 7.7 (Why Rotations in 3D Don't Commute).** Take two rotations in $SO(3)$: $R_x(90°)$ (rotate 90° about $x$-axis) and $R_z(90°)$ (rotate 90° about $z$-axis). Apply to the unit vector $\hat{e}_x = (1, 0, 0)$:

- $R_z(90°)$ then $R_x(90°)$: $(1,0,0) \to (0,1,0) \to (0,0,1)$
- $R_x(90°)$ then $R_z(90°)$: $(1,0,0) \to (1,0,0) \to (0,1,0)$

Different results! This non-commutativity is encoded in the Lie bracket: $[[\omega_1]_\times, [\omega_2]_\times] \neq 0$ in general.

---

## 8. SO(3): The Rotation Group

**Definition 8.1.** The *special orthogonal group* in three dimensions is

$$SO(3) = \left\{ R \in \mathbb{R}^{3 \times 3} \;\middle|\; R^T R = I, \; \det R = 1 \right\}$$

Elements of $SO(3)$ represent orientation-preserving rotations of $\mathbb{R}^3$.

**Proposition 8.2 (Dimension).** The matrix $R$ has 9 entries. The constraint $R^T R = I$ is a symmetric matrix equation, giving $\frac{3 \cdot 4}{2} = 6$ independent scalar equations. Hence $\dim SO(3) = 9 - 6 = 3$.

**Definition 8.3 (Axis-Angle Representation).** Every rotation $R \in SO(3)$ (other than the identity) can be written as rotation by angle $\theta \in (0, \pi]$ about a unit axis $\hat{\omega} \in S^2$. We write $R = R(\hat{\omega}, \theta)$.

**Proposition 8.4 (Rotation as a Matrix Exponential).** Consider a point $p$ rotating with constant angular velocity $\omega \in \mathbb{R}^3$. Its trajectory satisfies

$$\dot{p}(t) = \omega \times p(t) = [\omega]_\times \, p(t)$$

The solution is $p(t) = R(t) \, p(0)$ where $R(t)$ satisfies the matrix ODE

$$\dot{R}(t) = [\omega]_\times \, R(t), \qquad R(0) = I$$

By the theory of linear ODEs, $R(t) = e^{[\omega]_\times t}$. This motivates the *exponential map* connecting the Lie algebra to the group.

**Example 8.5 (90° Rotation About the $z$-Axis).** Rotation by $\theta = 90°$ about $\hat{\omega} = (0, 0, 1)$:

$$R_z(90°) = \begin{bmatrix} \cos 90° & -\sin 90° & 0 \\ \sin 90° & \cos 90° & 0 \\ 0 & 0 & 1 \end{bmatrix} = \begin{bmatrix} 0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

**Verify:** $R^T R = I$, $\det R = 1$, and $R(1, 0, 0)^T = (0, 1, 0)^T$ — the $x$-axis maps to the $y$-axis, as expected.

**Dimension count:** This matrix has 9 entries but only 1 free parameter ($\theta$). The 6 constraints from $R^T R = I$ plus the 2 constraints from fixing the axis leave $9 - 6 = 3$ degrees of freedom for a general rotation, but choosing a specific axis uses up 2 of those.

**Example 8.6 (180° Rotation About $(1, 1, 0)/\sqrt{2}$).** The axis $\hat{\omega} = (1/\sqrt{2}, \, 1/\sqrt{2}, \, 0)$ and $\theta = \pi$ give:

$$R = 2\hat{\omega}\hat{\omega}^T - I = 2 \begin{bmatrix} 1/2 & 1/2 & 0 \\ 1/2 & 1/2 & 0 \\ 0 & 0 & 0 \end{bmatrix} - I = \begin{bmatrix} 0 & 1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & -1 \end{bmatrix}$$

This swaps the $x$ and $y$ axes and flips $z$ — a 180° flip about the diagonal in the $xy$-plane.

---

## 9. The Lie Algebra $\mathfrak{so}(3)$

**Definition 9.1.** The Lie algebra of $SO(3)$ is

$$\mathfrak{so}(3) = \left\{ \Omega \in \mathbb{R}^{3 \times 3} \;\middle|\; \Omega^T = -\Omega \right\}$$

the vector space of $3 \times 3$ skew-symmetric matrices. It has dimension 3.

*Derivation.* Differentiating $R(t)^T R(t) = I$ at $t = 0$ gives $\dot{R}(0)^T + \dot{R}(0) = 0$, so $\dot{R}(0) \in \mathfrak{so}(3)$.

**Definition 9.2 (Hat and Vee Operators).** The *hat operator* $[\cdot]_\times : \mathbb{R}^3 \to \mathfrak{so}(3)$ maps a vector to a skew-symmetric matrix:

$$\omega = \begin{bmatrix} \omega_1 \\ \omega_2 \\ \omega_3 \end{bmatrix} \quad \mapsto \quad [\omega]_\times = \begin{bmatrix} 0 & -\omega_3 & \omega_2 \\ \omega_3 & 0 & -\omega_1 \\ -\omega_2 & \omega_1 & 0 \end{bmatrix}$$

The *vee operator* $(\cdot)^\vee : \mathfrak{so}(3) \to \mathbb{R}^3$ is its inverse.

**Proposition 9.3.** The hat operator satisfies $[\omega]_\times v = \omega \times v$ for all $\omega, v \in \mathbb{R}^3$. That is, matrix-vector multiplication with a skew-symmetric matrix is equivalent to the cross product.

**Definition 9.4 (Lie Bracket).** For $A, B \in \mathfrak{so}(3)$, the *Lie bracket* is the matrix commutator

$$[A, B] = AB - BA$$

**Proposition 9.5.** The Lie bracket on $\mathfrak{so}(3)$ corresponds to the cross product on $\mathbb{R}^3$:

$$\big[[\omega_1]_\times, \, [\omega_2]_\times\big] = [\omega_1 \times \omega_2]_\times$$

*Proof.* Direct computation. Expand $[\omega_1]_\times [\omega_2]_\times - [\omega_2]_\times [\omega_1]_\times$ using the identity $[a]_\times [b]_\times = b a^T - (a^T b) I$, and verify the result equals $[a \times b]_\times$. $\square$

**Example 9.7 (Hat Operator in Action).** Take $\omega = (1, 2, 3)$. The hat map gives:

$$[\omega]_\times = \begin{bmatrix} 0 & -3 & 2 \\ 3 & 0 & -1 \\ -2 & 1 & 0 \end{bmatrix}$$

Now let $v = (1, 0, 0)$. We can verify that $[\omega]_\times v = \omega \times v$:

$$[\omega]_\times v = \begin{bmatrix} 0 \\ 3 \\ -2 \end{bmatrix}, \qquad \omega \times v = \det\begin{bmatrix} e_1 & e_2 & e_3 \\ 1 & 2 & 3 \\ 1 & 0 & 0 \end{bmatrix} = (0, 3, -2)$$

They match. The hat operator converts the cross product into matrix multiplication.

**Example 9.8 (Lie Bracket = Cross Product).** Take $\omega_1 = (1, 0, 0)$ and $\omega_2 = (0, 1, 0)$. Their cross product is $\omega_1 \times \omega_2 = (0, 0, 1)$.

$$[\omega_1]_\times = \begin{bmatrix} 0 & 0 & 0 \\ 0 & 0 & -1 \\ 0 & 1 & 0 \end{bmatrix}, \quad [\omega_2]_\times = \begin{bmatrix} 0 & 0 & 1 \\ 0 & 0 & 0 \\ -1 & 0 & 0 \end{bmatrix}$$

Computing the commutator $[\omega_1]_\times [\omega_2]_\times - [\omega_2]_\times [\omega_1]_\times$:

$$= \begin{bmatrix} 0 & 0 & 0 \\ -1 & 0 & 0 \\ 0 & 0 & 0 \end{bmatrix} - \begin{bmatrix} 0 & 0 & 0 \\ 0 & 0 & 0 \\ 0 & -1 & 0 \end{bmatrix} = \begin{bmatrix} 0 & 0 & 0 \\ -1 & 0 & 0 \\ 0 & 1 & 0 \end{bmatrix} \quad \text{— but this needs correction...}$$

Actually, carrying out the full multiplication: $[\omega_1]_\times [\omega_2]_\times = \begin{bmatrix} 0 & 0 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 0 \end{bmatrix}$ and $[\omega_2]_\times [\omega_1]_\times = \begin{bmatrix} 0 & 0 & 0 \\ 0 & 0 & 0 \\ 0 & -1 & 0 \end{bmatrix}$. The bracket is not immediately obvious from these partial results; the full $3 \times 3$ commutator yields $\begin{bmatrix} 0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 0 \end{bmatrix} = [(0, 0, 1)]_\times$, confirming the bracket equals $[\omega_1 \times \omega_2]_\times$.

**Remark 9.9.** Physically, elements of $\mathfrak{so}(3)$ represent *angular velocities*. Given a time-varying rotation $R(t)$, the *body-frame angular velocity* is $\Omega^b = R^T \dot{R} \in \mathfrak{so}(3)$ and the *spatial-frame angular velocity* is $\Omega^s = \dot{R} R^T \in \mathfrak{so}(3)$.

---

## 10. Exponential and Logarithmic Maps for SO(3)

The exponential map $\exp : \mathfrak{so}(3) \to SO(3)$ sends an angular velocity (Lie algebra element) to the corresponding rotation (group element).

**Theorem 10.1 (Rodrigues' Formula).** For $\omega \in \mathbb{R}^3$ with $\theta = \|\omega\|$ and $K = [\hat{\omega}]_\times$ where $\hat{\omega} = \omega / \theta$:

$$e^{[\omega]_\times} = I + \sin\theta \, K + (1 - \cos\theta) \, K^2$$

Or equivalently, in terms of $\omega$ directly:

$$e^{[\omega]_\times} = I + \frac{\sin\theta}{\theta} [\omega]_\times + \frac{1 - \cos\theta}{\theta^2} [\omega]_\times^2$$

*Proof.* Let $K = [\hat{\omega}]_\times$ with $\|\hat{\omega}\| = 1$. We first establish a cyclic property:

$$K^2 = \hat{\omega}\hat{\omega}^T - I, \qquad K^3 = -K$$

The second identity follows from $K^3 = K \cdot K^2 = K(\hat{\omega}\hat{\omega}^T - I) = -K$ (since $K\hat{\omega} = \hat{\omega} \times \hat{\omega} = 0$). This gives the recurrence $K^4 = -K^2$, $K^5 = K$, $K^6 = K^2$, etc. Now expand the matrix exponential:

$$e^{\theta K} = I + \theta K + \frac{\theta^2}{2!}K^2 + \frac{\theta^3}{3!}K^3 + \frac{\theta^4}{4!}K^4 + \cdots$$

Substituting $K^3 = -K$, $K^4 = -K^2$, $K^5 = K, \ldots$:

$$= I + \left(\theta - \frac{\theta^3}{3!} + \frac{\theta^5}{5!} - \cdots\right)K + \left(\frac{\theta^2}{2!} - \frac{\theta^4}{4!} + \frac{\theta^6}{6!} - \cdots\right)K^2$$

Recognizing the Taylor series for $\sin\theta$ and $1 - \cos\theta$:

$$= I + \sin\theta \, K + (1 - \cos\theta) \, K^2 \qquad \square$$

**Theorem 10.2 (Logarithmic Map).** Given $R \in SO(3)$, $R \neq I$, the inverse of the exponential map is:

$$\theta = \arccos\!\left(\frac{\mathrm{tr}(R) - 1}{2}\right)$$

$$[\omega]_\times = \frac{\theta}{2\sin\theta}(R - R^T)$$

*Proof.* From Rodrigues' formula, $R + R^T = 2I + 2(1-\cos\theta)K^2 = 2\cos\theta \, I + 2(1-\cos\theta)\hat{\omega}\hat{\omega}^T$. Taking the trace: $\mathrm{tr}(R) = 2\cos\theta + 1$. Also, $R - R^T = 2\sin\theta \, K$, from which $K = (R - R^T)/(2\sin\theta)$. $\square$

**Example 10.3 (Rodrigues' Formula — 90° About $z$-Axis).** Let $\omega = (0, 0, \pi/2)$, so $\theta = \pi/2$ and $\hat{\omega} = (0, 0, 1)$.

$$K = [\hat{\omega}]_\times = \begin{bmatrix} 0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 0 \end{bmatrix}, \qquad K^2 = \begin{bmatrix} -1 & 0 & 0 \\ 0 & -1 & 0 \\ 0 & 0 & 0 \end{bmatrix}$$

Applying Rodrigues': $e^{\theta K} = I + \sin(\pi/2) K + (1 - \cos(\pi/2)) K^2 = I + K + K^2$

$$= \begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix} + \begin{bmatrix} 0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 0 \end{bmatrix} + \begin{bmatrix} -1 & 0 & 0 \\ 0 & -1 & 0 \\ 0 & 0 & 0 \end{bmatrix} = \begin{bmatrix} 0 & -1 & 0 \\ 1 & 0 & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

This is exactly $R_z(90°)$ from Example 8.5.

**Example 10.4 (Logarithmic Map — Recovering the Angle).** Given $R_z(90°)$ above:

$$\theta = \arccos\!\left(\frac{\mathrm{tr}(R) - 1}{2}\right) = \arccos\!\left(\frac{0 + 0 + 1 - 1}{2}\right) = \arccos(0) = \frac{\pi}{2}$$

$$[\omega]_\times = \frac{\frac{\pi}{2}}{2\sin\!\left(\frac{\pi}{2}\right)}(R - R^T) = \frac{\pi}{4} \begin{bmatrix} 0 & -2 & 0 \\ 2 & 0 & 0 \\ 0 & 0 & 0 \end{bmatrix} = \begin{bmatrix} 0 & -\frac{\pi}{2} & 0 \\[4pt] \frac{\pi}{2} & 0 & 0 \\[4pt] 0 & 0 & 0 \end{bmatrix}$$

So $\omega = (0, 0, \pi/2)$ — we recover the original axis and angle.

**Remark 10.5.** Rodrigues' formula provides a *closed-form* matrix exponential — no infinite series or eigendecomposition required. This makes it computationally efficient and central to robotics implementations.

**Remark 10.4.** The exponential map for SO(3) is a special case of the *Riemannian exponential map* on a Riemannian manifold (cf. Section 6, geodesics). The geodesics on $SO(3)$ (with the bi-invariant metric) are precisely the one-parameter subgroups $t \mapsto R \exp(t\Omega)$.

---

## 11. SE(3): The Rigid Body Motion Group

**Definition 11.1.** The *special Euclidean group* in three dimensions is

$$SE(3) = \left\{ T = \begin{bmatrix} R & p \\ 0 & 1 \end{bmatrix} \;\middle|\; R \in SO(3), \; p \in \mathbb{R}^3 \right\} \subset \mathbb{R}^{4 \times 4}$$

Elements of $SE(3)$ represent rigid-body transformations: a rotation $R$ followed by a translation $p$. Using homogeneous coordinates, the action on a point $x \in \mathbb{R}^3$ is

$$\begin{bmatrix} R & p \\ 0 & 1 \end{bmatrix} \begin{bmatrix} x \\ 1 \end{bmatrix} = \begin{bmatrix} Rx + p \\ 1 \end{bmatrix}$$

**Proposition 11.2 (Group Operations).**

$$T_1 T_2 = \begin{bmatrix} R_1 R_2 & R_1 p_2 + p_1 \\ 0 & 1 \end{bmatrix}, \qquad T^{-1} = \begin{bmatrix} R^T & -R^T p \\ 0 & 1 \end{bmatrix}$$

**Proposition 11.3 (Dimension).** $\dim SE(3) = \dim SO(3) + \dim \mathbb{R}^3 = 3 + 3 = 6$.

**Definition 11.4 (The Lie Algebra $\mathfrak{se}(3)$).** The Lie algebra of $SE(3)$ is

$$\mathfrak{se}(3) = \left\{ \begin{bmatrix} [\omega]_\times & v \\ 0 & 0 \end{bmatrix} \;\middle|\; \omega, v \in \mathbb{R}^3 \right\} \subset \mathbb{R}^{4 \times 4}$$

**Definition 11.5 (Twist).** A *twist* $\xi = (v, \omega) \in \mathbb{R}^6$ combines a linear velocity $v$ and an angular velocity $\omega$. The *wedge operator* $[\cdot]_\wedge : \mathbb{R}^6 \to \mathfrak{se}(3)$ maps

$$\xi = \begin{bmatrix} v \\ \omega \end{bmatrix} \mapsto [\xi]_\wedge = \begin{bmatrix} [\omega]_\times & v \\ 0 & 0 \end{bmatrix}$$

Physically, a twist describes the instantaneous velocity of a rigid body. The pair $(v, \omega)$ can be interpreted as an infinitesimal screw motion.

**Example 11.6 (A Simple Rigid-Body Transform).** A robot gripper is rotated 90° about the $z$-axis and then translated by $(1, 0, 0.5)$:

$$T = \begin{bmatrix} 0 & -1 & 0 & 1 \\ 1 & 0 & 0 & 0 \\ 0 & 0 & 1 & 0.5 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

Acting on a point $x = (2, 0, 0)$:

$$T \begin{bmatrix} 2 \\ 0 \\ 0 \\ 1 \end{bmatrix} = \begin{bmatrix} 0 \cdot 2 + (-1) \cdot 0 + 0 \cdot 0 + 1 \\ 1 \cdot 2 + 0 + 0 + 0 \\ 0 + 0 + 1 \cdot 0 + 0.5 \\ 1 \end{bmatrix} = \begin{bmatrix} 1 \\ 2 \\ 0.5 \\ 1 \end{bmatrix}$$

The point is rotated to $(0, 2, 0)$ and then translated to $(1, 2, 0.5)$.

**Example 11.7 (Composing Transforms).** If $T_1$ moves the arm base-to-elbow and $T_2$ moves elbow-to-hand, then $T_1 T_2$ gives the base-to-hand transform. The inverse $T^{-1}$ answers: "given a point in the gripper's frame, where is it in the world frame?" For our example:

$$T^{-1} = \begin{bmatrix} 0 & 1 & 0 & 0 \\ -1 & 0 & 0 & 1 \\ 0 & 0 & 1 & -0.5 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

Note $R^T = \begin{bmatrix} 0 & 1 & 0 \\ -1 & 0 & 0 \\ 0 & 0 & 1 \end{bmatrix}$ and $-R^T p = -(0, -1, 0.5)^T = (0, 1, -0.5)^T$.

---

## 12. Exponential Map for SE(3)

**Theorem 12.1.** For a twist $\xi = (v, \omega) \in \mathbb{R}^6$ with $\theta = \|\omega\| \neq 0$:

$$\exp([\xi]_\wedge) = \begin{bmatrix} e^{[\omega]_\times} & Gv \\ 0 & 1 \end{bmatrix}$$

where $G$ is the *left Jacobian of SO(3)*:

$$G = I + \frac{1 - \cos\theta}{\theta^2} [\omega]_\times + \frac{\theta - \sin\theta}{\theta^3} [\omega]_\times^2$$

When $\omega = 0$ (pure translation): $\exp([\xi]_\wedge) = \begin{bmatrix} I & v \\ 0 & 1 \end{bmatrix}$.

*Proof sketch.* Expand the matrix exponential of $[\xi]_\wedge$ using the block structure. The rotation block gives $e^{[\omega]_\times}$ as before. The translation block involves the series

$$\sum_{k=0}^{\infty} \frac{[\omega]_\times^k}{(k+1)!} \cdot v$$

which, using the cyclic property $[\omega]_\times^3 = -\theta^2 [\omega]_\times$, collapses to $Gv$. $\square$

**Remark 12.2 (Screw Theory and Chasles' Theorem).** By Chasles' theorem, every rigid-body displacement can be realized as a rotation about some axis combined with a translation along that axis — a *screw motion*. The twist $\xi$ encodes:

- **Screw axis direction:** $\hat{\omega}$
- **Point on axis:** $q = \hat{\omega} \times v / \theta$
- **Pitch:** $h = \hat{\omega}^T v / \theta$ (ratio of translational to rotational displacement)
- **Magnitude:** $\theta$

Pure rotations have $h = 0$; pure translations correspond to $\omega = 0$ (infinite pitch).

**Theorem 12.3 (Logarithmic Map for SE(3)).** Given $T = \begin{bmatrix} R & p \\ 0 & 1 \end{bmatrix} \in SE(3)$:

1. Compute $[\omega]_\times = \log(R)$ using the SO(3) logarithm (Theorem 10.2).
2. Compute $G$ from $\omega$.
3. Recover $v = G^{-1} p$.

The inverse of the left Jacobian is

$$G^{-1} = I - \frac{1}{2}[\omega]_\times + \frac{1}{\theta^2}\left(1 - \frac{\theta\sin\theta}{2(1-\cos\theta)}\right)[\omega]_\times^2$$

**Example 12.4 (Pure Translation).** If $\omega = (0, 0, 0)$ and $v = (1, 2, 3)$, the twist represents pure translation:

$$\exp([\xi]_\wedge) = \begin{bmatrix} I & v \\ 0 & 1 \end{bmatrix} = \begin{bmatrix} 1 & 0 & 0 & 1 \\ 0 & 1 & 0 & 2 \\ 0 & 0 & 1 & 3 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

No rotation, just a shift by $(1, 2, 3)$.

**Example 12.5 (Pure Rotation About $z$).** If $\omega = (0, 0, \pi/2)$ and $v = (0, 0, 0)$:

The left Jacobian $G$ is not needed for the rotation part (it is $e^{[\omega]_\times} = R_z(90°)$), and $Gv = G \cdot 0 = 0$. So:

$$\exp([\xi]_\wedge) = \begin{bmatrix} 0 & -1 & 0 & 0 \\ 1 & 0 & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

Pure rotation, no translation — a screw with zero pitch.

**Example 12.6 (Screw Motion — Door Hinge).** A door hinge is a screw axis along the $z$-axis at position $(1, 0, 0)$ with zero pitch. The screw axis is $\hat{\omega} = (0, 0, 1)$ and $v = -\hat{\omega} \times q = -(0, 0, 1) \times (1, 0, 0) = (0, -1, 0)$. So the twist is $\xi = (0, -1, 0, 0, 0, 1)$ (with appropriate scaling by $\theta$). As $\theta$ increases, the door swings open — rotating about the hinge axis without translating along it.

---

## 13. Adjoint Representation and Product of Exponentials

**Definition 13.1 (Adjoint Representation).** For $T = \begin{bmatrix} R & p \\ 0 & 1 \end{bmatrix} \in SE(3)$, the *adjoint* is the $6 \times 6$ matrix

$$\mathrm{Ad}_T = \begin{bmatrix} R & [p]_\times R \\ 0 & R \end{bmatrix} \in \mathbb{R}^{6 \times 6}$$

The adjoint acts on twists: if $\mathcal{V}_b$ is a twist expressed in the body frame, then $\mathcal{V}_s = \mathrm{Ad}_T \, \mathcal{V}_b$ is the same twist expressed in the spatial frame.

**Proposition 13.2 (Properties of the Adjoint).**

1. *Group homomorphism:* $\mathrm{Ad}_{T_1 T_2} = \mathrm{Ad}_{T_1} \, \mathrm{Ad}_{T_2}$
2. *Inverse:* $\mathrm{Ad}_{T^{-1}} = (\mathrm{Ad}_T)^{-1}$
3. *Twist transformation:* $T [\xi]_\wedge T^{-1} = [\mathrm{Ad}_T \xi]_\wedge$

**Theorem 13.3 (Product of Exponentials Formula).** Consider an open kinematic chain with $n$ joints. Let $M \in SE(3)$ be the end-effector pose in the home configuration, and let $\mathcal{S}_i = (v_i, \omega_i)$ be the screw axis of joint $i$ in the spatial frame at the home configuration. Then the forward kinematics is

$$T_{sb}(\theta) = e^{[\mathcal{S}_1]\theta_1} \, e^{[\mathcal{S}_2]\theta_2} \cdots e^{[\mathcal{S}_n]\theta_n} \, M$$

Each factor $e^{[\mathcal{S}_i]\theta_i}$ represents the rigid-body displacement due to joint $i$ moving by $\theta_i$.

**Definition 13.4 (Spatial and Body Jacobians).** The *spatial Jacobian* $J_s(\theta) \in \mathbb{R}^{6 \times n}$ relates joint velocities to the spatial twist:

$$\mathcal{V}_s = J_s(\theta) \, \dot{\theta}$$

Its columns are

$$J_s = \begin{bmatrix} \mathcal{S}_1 & \mathrm{Ad}_{e^{[\mathcal{S}_1]\theta_1}} \mathcal{S}_2 & \mathrm{Ad}_{e^{[\mathcal{S}_1]\theta_1}e^{[\mathcal{S}_2]\theta_2}} \mathcal{S}_3 & \cdots \end{bmatrix}$$

The *body Jacobian* is $J_b = \mathrm{Ad}_{T_{sb}^{-1}} J_s$.

**Example 13.6 (2-Link Planar Arm — PoE).** Consider a 2-link planar arm in the $xy$-plane with link lengths $L_1$ and $L_2$. Both joints rotate about the $z$-axis.

*Home configuration* ($\theta_1 = \theta_2 = 0$): the arm extends along the $x$-axis, with end-effector at $(L_1 + L_2, 0, 0)$:

$$M = \begin{bmatrix} 1 & 0 & 0 & L_1 + L_2 \\ 0 & 1 & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$$

*Screw axes:*
- Joint 1 at the origin: $\mathcal{S}_1 = (0, 0, 0, 0, 0, 1)$ — pure rotation about $z$ through the origin
- Joint 2 at $(L_1, 0, 0)$: $\omega_2 = (0, 0, 1)$, $v_2 = -\omega_2 \times q_2 = -(0, 0, 1) \times (L_1, 0, 0) = (0, -L_1, 0)$, so $\mathcal{S}_2 = (0, -L_1, 0, 0, 0, 1)$

*Forward kinematics:* $T(\theta_1, \theta_2) = e^{[\mathcal{S}_1]\theta_1} \, e^{[\mathcal{S}_2]\theta_2} \, M$

For $\theta_1 = 90°$, $\theta_2 = 0$: $e^{[\mathcal{S}_1]\pi/2} = \begin{bmatrix} 0 & -1 & 0 & 0 \\ 1 & 0 & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$, so $T = e^{[\mathcal{S}_1]\pi/2} M = \begin{bmatrix} 0 & -1 & 0 & 0 \\ 1 & 0 & 0 & L_1 + L_2 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}$

The end-effector is at $(0, L_1 + L_2, 0)$ — the arm has swung 90° counterclockwise, as expected.

**Remark 13.7.** The PoE formula has several advantages over the classical Denavit-Hartenberg convention: it requires no intermediate frame assignments, handles arbitrary joint types uniformly, and the screw axes $\mathcal{S}_i$ have clear geometric meaning.

---

## 14. Advanced Topics in Lie Theory

### 14.1. The Baker-Campbell-Hausdorff Formula

When composing two rotations via their Lie algebra elements, the result is generally *not* the sum of the elements.

**Theorem 14.1 (BCH Formula).** For $X, Y \in \mathfrak{g}$:

$$\log(e^X e^Y) = X + Y + \frac{1}{2}[X, Y] + \frac{1}{12}\big([X, [X, Y]] - [Y, [X, Y]]\big) + \cdots$$

The series involves nested Lie brackets of increasing depth.

**Example 14.2 (BCH for Small Rotations).** Suppose we compose two small rotations: $5°$ about $x$ and $5°$ about $y$. In radians, $\theta \approx 0.087$.

- $X = [(0.087, 0, 0)]_\times$, $Y = [(0, 0.087, 0)]_\times$
- First-order: $\log(e^X e^Y) \approx X + Y = [(0.087, 0.087, 0)]_\times$ — rotation about the diagonal in the $xy$-plane
- Second-order correction: $\frac{1}{2}[X, Y] = \frac{1}{2}[(0.087)(0.087)(0, 0, 1)]_\times = [(0, 0, 0.0038)]_\times$

So the composed rotation has a small $z$-component ($\approx 0.2°$) that wouldn't exist if rotations commuted. For large angles, higher-order BCH terms become significant.

**Remark 14.3.** For small perturbations, the first-order approximation $\log(e^X e^Y) \approx X + Y$ treats the group as a vector space — valid only locally. The second-order correction $\frac{1}{2}[X,Y]$ captures the non-commutativity of the group. For $SO(3)$, this corresponds to the fact that the order of rotations matters.

### 14.2. Lie Group Integration

Standard numerical integrators (Euler, Runge-Kutta) applied to rotation matrices do not preserve the constraint $R^T R = I$. *Lie group integrators* respect the group structure.

**Algorithm (Exponential Euler Method).** Given $\dot{R} = R \, [\omega(t)]_\times$:

$$R_{k+1} = R_k \cdot \exp\!\big([\omega_k]_\times \, \Delta t\big)$$

Since both $R_k$ and $\exp([\omega_k]_\times \Delta t)$ are in $SO(3)$, their product $R_{k+1}$ is guaranteed to remain in $SO(3)$ — the group structure is preserved exactly, regardless of step size.

### 14.3. Quaternions as Double Cover of SO(3)

**Definition 14.3.** A *unit quaternion* is $q = (w, x, y, z) \in \mathbb{R}^4$ with $\|q\| = 1$. The set of unit quaternions forms the 3-sphere $S^3$ with quaternion multiplication as the group operation.

**Proposition 14.4.** The quaternion corresponding to rotation by angle $\theta$ about axis $\hat{\omega}$ is

$$q = \left(\cos\frac{\theta}{2}, \; \hat{\omega}\sin\frac{\theta}{2}\right)$$

The rotation action on $v \in \mathbb{R}^3$ (identified with a pure quaternion) is $v \mapsto q v q^{-1}$.

**Proposition 14.5 (Double Cover).** The map $\phi : S^3 \to SO(3)$ defined by $\phi(q) = $ the rotation matrix corresponding to $q$ is a surjective group homomorphism with kernel $\{+1, -1\}$. That is, $\phi(q) = \phi(-q)$ — both $q$ and $-q$ represent the same rotation.

**Example 14.7 (Quaternion for 90° About $z$).** Axis $\hat{\omega} = (0, 0, 1)$, angle $\theta = \pi/2$:

$$q = \left(\cos\frac{\pi}{4}, \; (0, 0, 1)\sin\frac{\pi}{4}\right) = \left(\frac{\sqrt{2}}{2}, \; 0, \; 0, \; \frac{\sqrt{2}}{2}\right)$$

Verify $\|q\| = 1/2 + 0 + 0 + 1/2 = 1$. The antipodal quaternion $-q = (-\sqrt{2}/2, \, 0, \, 0, \, -\sqrt{2}/2)$ represents the *same* rotation — this is the double cover.

**Example 14.8 (Identity Rotation as Quaternion).** $\theta = 0$ gives $q = (1, 0, 0, 0)$. Both $q = (1, 0, 0, 0)$ and $q = (-1, 0, 0, 0)$ map to $R = I$, the identity rotation.

**Example 14.9 (SLERP — Interpolating Rotations).** Given two quaternions $q_0$ and $q_1$, the spherical linear interpolation (SLERP) produces a smooth path:

$$\mathrm{SLERP}(q_0, q_1, t) = \frac{\sin((1-t)\Omega)}{\sin\Omega} q_0 + \frac{\sin(t\Omega)}{\sin\Omega} q_1$$

where $\cos\Omega = q_0 \cdot q_1$. At $t = 0$ we get $q_0$; at $t = 1$ we get $q_1$. The interpolation follows the shortest arc on $S^3$, producing constant angular velocity — ideal for smooth camera motions or robot joint interpolation.

**Remark 14.10.** Quaternions avoid gimbal lock, use only 4 parameters (versus 9 for rotation matrices), and are efficient for interpolation (SLERP). The double cover is topologically unavoidable: $SO(3) \cong \mathbb{R}P^3$ is not simply connected, while $S^3$ is its universal cover.

---

# Part III — Optimization on Matrix Manifolds

---

## 15. Smooth Manifolds and Tangent Spaces (Revisited)

In Part I, we studied surfaces — 2-dimensional manifolds embedded in $\mathbb{R}^3$. In Part II, we encountered $SO(3)$ and $SE(3)$ as smooth manifolds with group structure. Now we develop the general framework for *optimization* on these spaces.

**Definition 15.1 (Smooth Manifold).** A *smooth manifold* $\mathcal{M}$ of dimension $d$ is a set that is locally diffeomorphic to $\mathbb{R}^d$. More precisely, for each point $x \in \mathcal{M}$, there exists an open neighborhood $U \ni x$ and a smooth bijection (chart) $\varphi : U \to \mathbb{R}^d$ with smooth inverse. Different charts are required to be *smoothly compatible* on overlaps.

**Definition 15.2 (Tangent Space).** The *tangent space* $T_x \mathcal{M}$ at $x \in \mathcal{M}$ is the $d$-dimensional vector space of directions "tangent to" $\mathcal{M}$ at $x$. For an embedded submanifold $\mathcal{M} \subset \mathbb{R}^N$:

$$T_x \mathcal{M} = \left\{ \gamma'(0) \;\middle|\; \gamma : (-\varepsilon, \varepsilon) \to \mathcal{M}, \; \gamma(0) = x \right\}$$

**Remark 15.3 (Unification).** The machinery of smooth manifolds unifies our earlier examples:

| Manifold | Points | Tangent vectors | Dimension |
|:---:|:---|:---|:---:|
| $S^{n-1}$ | Unit vectors $x$ | $\{v : x^T v = 0\}$ | $n-1$ |
| Surface $\sigma(U)$ | Points on surface | $\mathrm{span}\{\sigma_u, \sigma_v\}$ | 2 |
| $SO(3)$ | Rotation matrices $R$ | $\{R\Omega : \Omega^T = -\Omega\}$ | 3 |
| $SE(3)$ | Rigid transforms $T$ | $\{T[\xi]_\wedge : \xi \in \mathbb{R}^6\}$ | 6 |

**Example 15.4 (The Unit Circle $S^1$ as a Manifold).** The simplest non-trivial manifold: $S^1 = \{(x, y) \in \mathbb{R}^2 : x^2 + y^2 = 1\}$.

- *Charts:* Near the "north" point $(0, 1)$, we can use $x$ as a coordinate: $\varphi(x, y) = x$, mapping to $(-1, 1) \subset \mathbb{R}$. Near $(1, 0)$, use $y$ as a coordinate. No single chart covers all of $S^1$ (it's topologically a circle, not a line), but two charts suffice.
- *Tangent space:* At $(1, 0)$, differentiating $x^2 + y^2 = 1$ gives $2x\dot{x} + 2y\dot{y} = 0$, so $\dot{x} = 0$ at this point. The tangent space is $T_{(1,0)}S^1 = \{(0, v) : v \in \mathbb{R}\}$ — the vertical line through $(1, 0)$.
- *Dimension:* 1, as expected.

**Example 15.5 (The Unit Sphere $S^2$ — Tangent Plane).** At the "north pole" $p = (0, 0, 1) \in S^2$:

$$T_p S^2 = \{v \in \mathbb{R}^3 : p^T v = 0\} = \{(v_1, v_2, 0) : v_1, v_2 \in \mathbb{R}\}$$

This is the horizontal plane touching the sphere at the top — exactly what "tangent plane" means geometrically. Any tangent vector at the north pole has no $z$-component.

---

## 16. The Stiefel Manifold

**Definition 16.1.** The *Stiefel manifold* is

$$\mathrm{St}(n, p) = \left\{ X \in \mathbb{R}^{n \times p} \;\middle|\; X^T X = I_p \right\}$$

Its elements are *orthonormal $p$-frames* in $\mathbb{R}^n$: ordered sets of $p$ orthonormal vectors.

**Proposition 16.2 (Special Cases).**

- $\mathrm{St}(n, 1) = S^{n-1}$ (the unit sphere)
- $\mathrm{St}(n, n) = O(n)$ (the orthogonal group)

**Proposition 16.3 (Dimension).**

$$\dim \mathrm{St}(n, p) = np - \frac{p(p+1)}{2}$$

*Proof.* The ambient space $\mathbb{R}^{n \times p}$ has dimension $np$. The constraint $X^T X = I_p$ is a symmetric $p \times p$ matrix equation, imposing $p(p+1)/2$ independent scalar constraints. $\square$

**Proposition 16.4 (Tangent Space).** Differentiating $X^T X = I_p$ along a curve $X(t)$ gives $\dot{X}^T X + X^T \dot{X} = 0$. Hence:

$$T_X \mathrm{St}(n, p) = \left\{ Z \in \mathbb{R}^{n \times p} \;\middle|\; X^T Z + Z^T X = 0 \right\}$$

That is, $X^T Z$ must be *skew-symmetric*.

**Proposition 16.5 (Orthogonal Projection).** The projection of $V \in \mathbb{R}^{n \times p}$ onto $T_X \mathrm{St}(n, p)$ is

$$\mathrm{proj}_X(V) = V - X \, \mathrm{sym}(X^T V)$$

where $\mathrm{sym}(A) = \tfrac{1}{2}(A + A^T)$.

*Proof.* Write $V = \mathrm{proj}_X(V) + X \, \mathrm{sym}(X^T V)$. The first term satisfies $X^T(V - X\,\mathrm{sym}(X^TV)) = X^TV - \mathrm{sym}(X^TV)$, which is skew-symmetric, hence in $T_X\mathrm{St}$. The second term is in the normal space. $\square$

**Example 16.6 ($\mathrm{St}(3, 1) = S^2$ — the Familiar Sphere).** With $n = 3$, $p = 1$, the Stiefel manifold is just unit vectors in $\mathbb{R}^3$. Take $x = (1, 0, 0)^T$:

- *Tangent space:* $\{z \in \mathbb{R}^3 : x^T z = 0\} = \{(0, z_2, z_3)\}$ — vectors perpendicular to $x$.
- *Projection:* For $v = (3, 1, 2)^T$: $\mathrm{sym}(x^T v) = 3$, so $\mathrm{proj}_x(v) = (3, 1, 2) - 3(1, 0, 0) = (0, 1, 2)$.

This is ordinary orthogonal projection $v - (x^T v) x$, removing the component along $x$.

**Example 16.7 ($\mathrm{St}(3, 2)$ — Pairs of Orthonormal Vectors).** Elements are $3 \times 2$ matrices $X$ with $X^T X = I_2$. For example:

$$X = \begin{bmatrix} 1 & 0 \\ 0 & 1 \\ 0 & 0 \end{bmatrix}$$

represents the orthonormal pair $\{e_1, e_2\}$. Dimension: $3 \cdot 2 - 2 \cdot 3/2 = 3$. The tangent space requires $X^T Z + Z^T X = 0$, meaning:

$$\begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \end{bmatrix} Z + Z^T \begin{bmatrix} 1 & 0 \\ 0 & 1 \\ 0 & 0 \end{bmatrix} = 0$$

The top-left $2 \times 2$ block of $Z$ must be skew-symmetric (1 free parameter), and the bottom row of $Z$ is unconstrained (2 free parameters). Total: 3 parameters, matching the dimension.

---

## 17. The Grassmann Manifold

**Definition 17.1.** The *Grassmann manifold* $\mathrm{Gr}(n, p)$ is the set of all $p$-dimensional linear subspaces of $\mathbb{R}^n$.

While the Stiefel manifold parameterizes orthonormal *bases* of a subspace, the Grassmannian parameterizes the *subspace itself*, irrespective of the choice of basis.

**Proposition 17.2 (Quotient Structure).**

$$\mathrm{Gr}(n, p) = \mathrm{St}(n, p) \, / \, O(p)$$

Two frames $X, \tilde{X} \in \mathrm{St}(n, p)$ represent the same subspace if and only if $\tilde{X} = XQ$ for some $Q \in O(p)$.

**Proposition 17.3 (Dimension).**

$$\dim \mathrm{Gr}(n, p) = p(n - p)$$

*Proof.* $\dim \mathrm{St}(n,p) - \dim O(p) = \left(np - \frac{p(p+1)}{2}\right) - \frac{p(p-1)}{2} = np - p^2 = p(n-p)$. $\square$

**Definition 17.4 (Vertical and Horizontal Spaces).** At a representative $X \in \mathrm{St}(n,p)$:

- The *vertical space* $\mathcal{V}_X = \{X\Omega : \Omega^T = -\Omega\}$ consists of tangent directions that change the basis but not the subspace.
- The *horizontal space* $\mathcal{H}_X = \{Z \in \mathbb{R}^{n \times p} : X^T Z = 0\}$ consists of directions that genuinely change the subspace.

The tangent space of the Grassmannian is identified with $\mathcal{H}_X$.

**Proposition 17.5 (Horizontal Projection).** The projection of $V \in \mathbb{R}^{n \times p}$ onto $\mathcal{H}_X$ is

$$\mathrm{proj}_X^{\mathrm{Gr}}(V) = (I - XX^T) V$$

This removes all components in the column space of $X$.

**Example 17.6 (Lines Through the Origin = $\mathrm{Gr}(2, 1)$).** The simplest Grassmannian: 1-dimensional subspaces of $\mathbb{R}^2$. Each "point" is a line through the origin (not a vector — the line spanned by $(1, 1)$ is the same as the line spanned by $(-2, -2)$).

The representatives $x = (1/\sqrt{2}, \, 1/\sqrt{2})^T$ and $\tilde{x} = (-1/\sqrt{2}, \, -1/\sqrt{2})^T$ both represent the same line. They are related by $\tilde{x} = x \cdot (-1)$, where $-1 \in O(1) = \{+1, -1\}$.

Dimension: $1 \cdot (2 - 1) = 1$. Topologically, $\mathrm{Gr}(2, 1) \cong \mathbb{R}P^1 \cong S^1$ — it's a circle!

**Example 17.7 (Planes in $\mathbb{R}^3$ = $\mathrm{Gr}(3, 2)$).** Points are 2-dimensional subspaces (planes through the origin) in $\mathbb{R}^3$. Each plane is represented by an orthonormal basis, but the basis choice doesn't matter — only the plane itself. Dimension: $2(3 - 2) = 2$.

Consider the $xy$-plane, represented by $X = \begin{bmatrix} 1 & 0 \\ 0 & 1 \\ 0 & 0 \end{bmatrix}$. Any rotation of this basis within the $xy$-plane (i.e., $X Q$ for $Q \in O(2)$) represents the same subspace. A tangent direction that *changes* the plane (horizontal) would tilt it, e.g., $Z = \begin{bmatrix} 0 & 0 \\ 0 & 0 \\ 1 & 0 \end{bmatrix}$ (note: $X^T Z = 0$). A tangent direction that merely rotates the basis within the plane (vertical) would be $Z = \begin{bmatrix} 0 & -1 \\ 1 & 0 \\ 0 & 0 \end{bmatrix} = X\Omega$ with $\Omega = \begin{bmatrix} 0 & -1 \\ 1 & 0 \end{bmatrix}$.

---

## 18. Riemannian Gradient

**Definition 18.1 (Riemannian Metric).** A *Riemannian metric* on $\mathcal{M}$ assigns to each tangent space $T_x\mathcal{M}$ an inner product $\langle \cdot, \cdot \rangle_x$ that varies smoothly with $x$. For matrix manifolds embedded in $\mathbb{R}^{n \times p}$, we typically use the inherited Euclidean (Frobenius) inner product $\langle A, B \rangle = \mathrm{tr}(A^T B)$.

**Definition 18.2 (Riemannian Gradient).** The *Riemannian gradient* $\mathrm{grad}\, f(X) \in T_X\mathcal{M}$ is the unique tangent vector satisfying

$$\langle \mathrm{grad}\, f(X), \, Z \rangle = Df(X)[Z] \qquad \text{for all } Z \in T_X\mathcal{M}$$

where $Df(X)[Z]$ is the directional derivative of $f$ at $X$ in direction $Z$.

**Theorem 18.3.** For an embedded submanifold $\mathcal{M} \subset \mathbb{R}^{n \times p}$ with the inherited metric, the Riemannian gradient is the *orthogonal projection* of the Euclidean gradient onto the tangent space:

$$\mathrm{grad}\, f(X) = \mathrm{proj}_{T_X\mathcal{M}}\!\big(\nabla f(X)\big)$$

where $\nabla f(X) \in \mathbb{R}^{n \times p}$ is the Euclidean gradient (matrix of partial derivatives).

*Proof.* For any $Z \in T_X\mathcal{M}$, write $\nabla f = \mathrm{proj}_{T_X}(\nabla f) + \mathrm{proj}_{N_X}(\nabla f)$ where $N_X$ is the normal space. Then $\langle \nabla f, Z \rangle = \langle \mathrm{proj}_{T_X}(\nabla f), Z \rangle$ since the normal component is orthogonal to $Z$. By definition, $\langle \nabla f, Z \rangle = Df(X)[Z]$. So $\mathrm{proj}_{T_X}(\nabla f)$ satisfies the defining property of $\mathrm{grad}\, f$. $\square$

**Proposition 18.4 (Riemannian Gradient on the Stiefel Manifold).**

$$\mathrm{grad}\, f(X) = \nabla f(X) - X \, \mathrm{sym}\!\big(X^T \nabla f(X)\big)$$

**Proposition 18.5 (Riemannian Gradient on the Grassmann Manifold).**

$$\mathrm{grad}\, f(X) = (I - XX^T) \, \nabla f(X)$$

**Example 18.7 (Riemannian Gradient on $S^2$).** Minimize $f(x) = \|x - a\|^2$ over the unit sphere $S^2$ (find the point on the sphere closest to $a$).

- Euclidean gradient: $\nabla f(x) = 2(x - a)$
- Projection onto $T_x S^2$: $\mathrm{grad}\,f(x) = 2(x - a) - 2(x^T(x - a))x = 2(x - a) - 2(1 - x^T a)x = -2a + 2(x^T a)x$

Simplifying: $\mathrm{grad}\,f(x) = -2(a - (x^T a)x) = -2\,\mathrm{proj}_{T_x}(a)$. The Riemannian gradient points from $x$ toward $a$, but projected onto the sphere's tangent plane. Setting $\mathrm{grad}\,f = 0$ gives $a = (x^T a)x$, meaning $x = a/\|a\|$ — the closest point on the sphere to $a$ is in the direction of $a$, as expected.

**Example 18.8 (Rayleigh Quotient on Stiefel).** The Rayleigh quotient $f(x) = x^T A x$ for symmetric $A$, minimized over $x \in S^{n-1}$ (the Stiefel manifold $\mathrm{St}(n, 1)$), finds the smallest eigenvalue.

- Euclidean gradient: $\nabla f = 2Ax$
- Riemannian gradient: $\mathrm{grad}\,f = 2Ax - 2(x^T A x)x = 2(Ax - \lambda x)$ where $\lambda = x^T Ax$

Setting $\mathrm{grad}\,f = 0$ gives $Ax = \lambda x$ — the critical points are exactly the eigenvectors of $A$! Gradient descent on the sphere naturally converges to an eigenvector.

**Remark 18.9.** The key insight of manifold optimization: by projecting the gradient onto the tangent space, we transform a *constrained* optimization problem (minimize $f$ subject to $X \in \mathcal{M}$) into an *unconstrained* problem on the manifold. No Lagrange multipliers, no penalty terms — the constraint is handled by the geometry.

---

## 19. Retraction Maps

In Euclidean space, a gradient step $X_{k+1} = X_k - \alpha \nabla f(X_k)$ stays in $\mathbb{R}^{n \times p}$. On a manifold, the tangent step $X_k - \alpha \, \mathrm{grad}\, f(X_k)$ generally leaves $\mathcal{M}$. A *retraction* maps back.

**Definition 19.1 (Retraction).** A *retraction* on $\mathcal{M}$ is a smooth map $R_X : T_X\mathcal{M} \to \mathcal{M}$ satisfying:

1. $R_X(0) = X$ (centering)
2. $\frac{d}{dt} R_X(tZ)\big|_{t=0} = Z$ (first-order agreement with the identity on the tangent space)

Condition 2 ensures that for small steps, the retraction approximates the exponential map (geodesic).

**Proposition 19.2 (QR Retraction).** For the Stiefel manifold:

$$R_X^{\mathrm{QR}}(Z) = \mathrm{qf}(X + Z)$$

where $\mathrm{qf}(\cdot)$ extracts the $Q$ factor from a thin QR decomposition (with positive diagonal entries of $R$). Cost: $O(np^2)$.

**Proposition 19.3 (Polar Retraction).** For the Stiefel manifold:

$$R_X^{\mathrm{polar}}(Z) = (X + Z)(I + Z^T Z)^{-1/2}$$

Equivalently, if $X + Z = U\Sigma V^T$ is the thin SVD, then $R_X^{\mathrm{polar}}(Z) = UV^T$. The polar retraction has *second-order* agreement with the geodesic. Cost: $O(np^2 + p^3)$.

**Proposition 19.4 (Exponential Map on SO(n)).** For $R \in SO(n)$ and $R\Omega \in T_R SO(n)$:

$$\exp_R(R\Omega) = R \cdot \mathrm{expm}(\Omega)$$

where $\mathrm{expm}$ is the matrix exponential. This follows the geodesic exactly but costs $O(n^3)$.

**Example 19.5 (QR Retraction on $S^2$).** Start at $x = (1, 0, 0)^T$ on the unit sphere. Tangent vector $z = (0, 0.5, 0)^T$ (note $x^T z = 0$). The QR retraction normalizes:

$$R_x^{\mathrm{QR}}(z) = \frac{x + z}{\|x + z\|} = \frac{(1, 0.5, 0)}{\sqrt{1.25}} = (0.894, 0.447, 0)$$

For $S^{n-1}$, QR retraction is just normalization — the simplest possible retraction.

**Example 19.6 (Why We Need Retractions).** Consider gradient descent for minimizing $f(x) = x^T A x$ on $S^2$. Starting at $x_0 = (1, 0, 0)^T$ with gradient $g = (0, -0.3, 0)^T$:

- *Euclidean step:* $x_0 - \alpha g = (1, 0.3, 0)$ — this has norm $\sqrt{1.09} \neq 1$, so it left the sphere!
- *With retraction:* normalize $(1, 0.3, 0) \to (0.958, 0.287, 0)$ — back on the sphere.

Without a retraction, iterates would drift away from the constraint set. The retraction "pulls back" to the manifold after each step.

**Remark 19.7.** In practice, the QR retraction is preferred for its low cost and simplicity. The polar retraction is used when higher accuracy is needed. The exponential map is reserved for problems where exact geodesics matter (e.g., computing the Fréchet mean).

---

## 20. Riemannian Gradient Descent and Applications

**Algorithm 20.1 (Riemannian Gradient Descent).**

> **Input:** $f : \mathcal{M} \to \mathbb{R}$, initial point $X_0 \in \mathcal{M}$, step sizes $\{\alpha_k\}$, retraction $R$.
>
> **For** $k = 0, 1, 2, \ldots$:
> 1. Compute $\mathrm{grad}\, f(X_k) = \mathrm{proj}_{T_{X_k}}\big(\nabla f(X_k)\big)$
> 2. Update $X_{k+1} = R_{X_k}\!\big(-\alpha_k \, \mathrm{grad}\, f(X_k)\big)$

Each iterate $X_k$ lies exactly on $\mathcal{M}$ — the constraint $X^TX = I$ is satisfied at every step, not just in the limit.

**Theorem 20.2 (Convergence).** Under standard assumptions (geodesic $L$-smoothness of $f$, step size $\alpha_k = 1/L$):

- *General (non-convex):* $\displaystyle\min_{0 \le k \le K} \|\mathrm{grad}\, f(X_k)\| \le O(1/\sqrt{K})$
- *Geodesically convex:* $f(X_K) - f^* \le O(1/K)$

These rates mirror their Euclidean counterparts.

### Application: Rotation Averaging on SO(3)

A natural application that bridges Lie theory (Part II) and manifold optimization (Part III).

**Problem 20.3 (Karcher/Fréchet Mean).** Given noisy rotation measurements $R_1, \ldots, R_N \in SO(3)$, find the rotation that minimizes the sum of squared geodesic distances:

$$\bar{R} = \arg\min_{R \in SO(3)} \sum_{i=1}^N d^2(R, R_i)$$

where the geodesic distance on $SO(3)$ is $d(R, R_i) = \|\log(R^T R_i)\|_F / \sqrt{2}$ and $\log$ is the SO(3) logarithm from Theorem 10.2.

**Proposition 20.4 (Riemannian Gradient).** The Riemannian gradient of $f(R) = \sum_i \|\log(R^T R_i)\|_F^2$ is

$$\mathrm{grad}\, f(R) = -R \sum_{i=1}^N \log(R^T R_i)$$

**Algorithm 20.5 (Rotation Averaging).** Using the exponential map as retraction:

$$R_{k+1} = R_k \cdot \exp\!\left(\alpha \sum_{i=1}^N \log(R_k^T R_i)\right)$$

This is precisely Riemannian gradient descent on $SO(3)$. The update stays in $SO(3)$ by construction (product of elements in $SO(3)$), and converges to the Fréchet mean.

**Example 20.7 (Riemannian GD on $S^1$ — Finding the Closest Point).** Minimize $f(x) = \|x - a\|^2$ on the unit circle $S^1 \subset \mathbb{R}^2$, with $a = (2, 1)$.

*Step 0:* $x_0 = (1, 0)$.
- Euclidean gradient: $\nabla f = 2(x_0 - a) = (-2, -2)$
- Riemannian gradient: $\mathrm{grad}\,f = \nabla f - (x_0^T \nabla f) x_0 = (-2, -2) - (-2)(1, 0) = (0, -2)$
- Step: $x_0 - 0.1 \cdot \mathrm{grad}\,f = (1, 0.2)$
- Retract (normalize): $x_1 = (1, 0.2)/\sqrt{1.04} \approx (0.981, 0.196)$

*Step 1:* $x_1 \approx (0.981, 0.196)$.
- Continue iterating... The iterates move along the circle toward $a/\|a\| = (2/\sqrt{5}, \, 1/\sqrt{5}) \approx (0.894, 0.447)$.

After convergence, we find the point on $S^1$ closest to $a$ — which is simply $a$ normalized.

**Example 20.8 (Rotation Averaging — Two Measurements).** Suppose we have two noisy rotation measurements:

- $R_1 = R_z(10°)$ — measured 10° about $z$
- $R_2 = R_z(20°)$ — measured 20° about $z$

The Fréchet mean should be $\bar{R} \approx R_z(15°)$ (the "midpoint"). Starting from $R_0 = I$:

- $\log(R_0^T R_1) = \log(R_1) = [(0, 0, 10° \cdot \pi/180)]_\times$
- $\log(R_0^T R_2) = \log(R_2) = [(0, 0, 20° \cdot \pi/180)]_\times$
- Sum of logs (vee'd): $(0, 0, 0.175 + 0.349) = (0, 0, 0.524)$ rad
- Update: $R_1 = I \cdot \exp(\alpha \cdot [(0, 0, 0.524)]_\times)$

With appropriate step size, this converges to $R_z(15°)$ — the geodesic midpoint of the two measurements.

**Remark 20.9.** This algorithm appears in computer vision (averaging camera orientations), robotics (sensor fusion), and structural biology (averaging molecular conformations). The Riemannian perspective makes the algorithm principled: we are doing gradient descent on the correct geometry.

---

## 21. Connections and Outlook

The three parts of this document — differential geometry, Lie groups, and matrix manifolds — are facets of a single geometric viewpoint. The table below summarizes the parallel structures.

| | Curves/Surfaces | Lie Groups | Matrix Manifolds |
|:---|:---|:---|:---|
| **Space** | Surface $\sigma(U)$ | $SO(3)$, $SE(3)$ | $\mathrm{St}(n,p)$, $\mathrm{Gr}(n,p)$ |
| **Point** | $\sigma(u,v)$ | Rotation $R$ | Orthonormal frame $X$ |
| **Tangent vector** | $\sigma_u \dot{u} + \sigma_v \dot{v}$ | $R\Omega$, $\Omega \in \mathfrak{so}(3)$ | $Z$ with $X^TZ$ skew-sym. |
| **"Straight line"** | Geodesic | One-parameter subgroup | Geodesic on manifold |
| **Distance** | $\int \sqrt{g_{ij}\dot{u}^i\dot{u}^j}\,dt$ | $\|\log(R_1^T R_2)\|_F$ | $\|\log(X_1^T X_2)\|_F$ |
| **Return to manifold** | — | Matrix exponential | Retraction ($R^{\mathrm{QR}}$, $R^{\mathrm{polar}}$) |

**Example 21.1 (One Problem, Three Perspectives).** Consider a robot arm whose end-effector orientation must be optimized:

1. **Differential geometry lens:** The end-effector traces a curve on $SO(3)$. Its "turning rate" is curvature in the Lie group, governed by the Frenet-Serret-like equation $\dot{R} = R\Omega$.
2. **Lie group lens:** The forward kinematics is $T(\theta) = e^{[\mathcal{S}_1]\theta_1} \cdots e^{[\mathcal{S}_n]\theta_n} M$. The Jacobian $J_s$ maps joint velocities to spatial twists via the adjoint.
3. **Manifold optimization lens:** To find the joint angles that minimize a cost (e.g., distance to a target orientation), we do Riemannian gradient descent on $SO(3)$, using the log map to compute geodesic distances and the exp map to retract.

All three perspectives describe the same geometric reality — they are different windows into the rich structure of $SO(3)$.

### Directions for Further Study

- **Riemannian Conjugate Gradient.** Replaces the gradient with a conjugate direction, requiring a *vector transport* $\mathcal{T}_{X \to Y} : T_X\mathcal{M} \to T_Y\mathcal{M}$ to move the previous search direction to the new tangent space. Update: $\eta_{k+1} = -\mathrm{grad}\,f(X_{k+1}) + \beta_k \, \mathcal{T}_{X_k \to X_{k+1}}(\eta_k)$.

- **Riemannian Trust-Region Methods.** Solve a local subproblem $\min_{\eta \in T_X\mathcal{M}, \|\eta\| \le \Delta} \langle \mathrm{grad}\,f, \eta \rangle + \frac{1}{2}\langle \mathrm{Hess}\,f[\eta], \eta \rangle$ using the Riemannian Hessian. Achieves superlinear convergence near the optimum.

- **Low-Rank Manifold.** The set of $m \times n$ matrices of fixed rank $r$ forms a manifold of dimension $r(m + n - r)$. Manifold optimization here yields efficient methods for matrix completion, collaborative filtering, and PCA.

- **Parallel Transport and Connections.** The machinery for moving vectors between tangent spaces at different points. Essential for second-order methods and for defining curvature of the manifold itself.

- **Gauss-Bonnet and Global Geometry.** The theorem $\int_M K \, dA = 2\pi\chi(M)$ generalizes to higher dimensions via Chern-Gauss-Bonnet, connecting local curvature to global topology in deep ways.

- **Symplectic Geometry.** The natural setting for Hamiltonian mechanics and Lagrangian dynamics, where the phase space carries a symplectic (rather than Riemannian) structure.

---

## References

1. **M. P. do Carmo.** *Differential Geometry of Curves and Surfaces.* Prentice-Hall, 1976. The classic introduction to curves, surfaces, and the Gauss-Bonnet theorem.

2. **A. Pressley.** *Elementary Differential Geometry.* Springer, 2nd edition, 2010. An accessible treatment with many worked examples.

3. **R. M. Murray, Z. Li, S. S. Sastry.** *A Mathematical Introduction to Robotic Manipulation.* CRC Press, 1994. Lie groups and screw theory for robotics.

4. **K. M. Lynch, F. C. Park.** *Modern Robotics: Mechanics, Planning, and Control.* Cambridge University Press, 2017. Product of exponentials, spatial/body Jacobians, and the adjoint representation.

5. **P.-A. Absil, R. Mahony, R. Sepulchre.** *Optimization Algorithms on Matrix Manifolds.* Princeton University Press, 2008. The foundational reference for Riemannian optimization on Stiefel and Grassmann manifolds.

6. **N. Boumal.** *An Introduction to Optimization on Smooth Manifolds.* Cambridge University Press, 2023. A modern, accessible treatment with convergence theory.

7. **J. E. Marsden, T. S. Ratiu.** *Introduction to Mechanics and Symmetry.* Springer, 2nd edition, 1999. Lie groups in the context of geometric mechanics.
