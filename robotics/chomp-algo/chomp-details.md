At its core, CHOMP treats motion planning not as a discrete graph-search problem, but as a continuous problem in the **calculus of variations**. It seeks to find a trajectory function that minimizes a specific mathematical cost. 

Here is a simple but exhaustive breakdown of the core mathematical principles behind the CHOMP algorithm:

### 1. The Trajectory as a Function
Mathematically, CHOMP represents a robot's trajectory $\xi$ as a smooth continuous function mapping time $t \in$ to the robot's configuration space (its joint angles). Because the trajectory is a function, any cost we want to minimize must be expressed as an **objective functional**—a function that takes another function as its input and returns a single real number.

### 2. The Objective Functional ($U[\xi]$)
CHOMP's goal is to minimize an objective functional $U[\xi]$, which is the weighted sum of two terms: 
$U[\xi] = F_{obs}[\xi] + \lambda F_{smooth}[\xi]$.

*   **Smoothness Functional ($F_{smooth}$):** This measures the dynamical "effort" of the trajectory. Usually, it is defined as the integral of squared velocities over time: $\frac{1}{2}\int_{0}^{1} \| \frac{d}{dt}\xi(t) \|^2 dt$. Minimizing this naturally penalizes jerky movements.
*   **Obstacle Functional ($F_{obs}$):** This measures the collision cost. It is computed as a workspace line integral: $\int_{0}^{1} \int_{B} c(x(\xi(t), u)) \| \frac{d}{dt}x(\xi(t), u) \| du dt$. Here, $c$ is a cost field penalizing proximity to obstacles, $x$ is the 3D workspace position of a point $u$ on the robot's body $B$, and $\| \frac{d}{dt}x(\xi(t), u) \|$ transforms the integral into an **arc-length parametrization**. This ensures the obstacle cost is based strictly on the geometric shape of the path through space, invariant to how fast the robot is moving along it.

### 3. Functional Gradients
To minimize $U[\xi]$, CHOMP uses gradient descent. However, because $\xi$ is a function rather than a simple vector, CHOMP must compute a **functional gradient** (denoted $\bar{\nabla}U$). 

Using the Euler-Lagrange equations from the calculus of variations, the direction of steepest descent for a functional shaped like $\int v(\xi, \xi') dt$ is given by:
$\bar{\nabla}U[\xi] = \frac{\partial v}{\partial \xi} - \frac{d}{dt}\frac{\partial v}{\partial \xi'}$.
This tells the algorithm how to perturb the entire continuous trajectory to maximally decrease the cost.

### 4. Covariant Gradients and the Metric $A$
A naive Euclidean gradient update would treat the trajectory simply as a list of abstract parameters. If the algorithm finds a collision at one specific time step, a standard gradient descent would just aggressively yank that single waypoint away from the obstacle, destroying the smoothness of the trajectory.

To solve this, CHOMP uses a **covariant gradient**. It defines a custom Riemannian metric, represented by an operator $A$, that measures the size of a trajectory perturbation strictly in terms of physical dynamics (e.g., how much acceleration it adds) rather than simple Euclidean distance. By mapping the raw functional gradient through the inverse of this metric ($A^{-1}$), CHOMP ensures that any perturbation applied to the trajectory avoids introducing unnecessary accelerations. Physically, $A^{-1}$ acts as a smoothing operator that distributes a localized obstacle-avoidance push smoothly across the rest of the trajectory.

### 5. Waypoint Discretization and the Update Rule
To make this computable, CHOMP discretizes the continuous function $\xi$ into a finite vector of waypoints $\xi = (q_1, q_2, \dots, q_n)$. The smoothness metric $A$ becomes a matrix constructed using finite differencing $A = K^T K$ (where $K$ computes velocities/accelerations between waypoints). 

With this discretization, the core iterative update rule of CHOMP becomes:
**$\xi_{i+1} = \xi_i - \frac{1}{\eta} A^{-1} \bar{\nabla} U[\xi_i]$**.
*(Where $\eta$ is a step-size parameter, $A^{-1}$ is the smoothing matrix, and $\bar{\nabla} U[\xi_i]$ is the gradient of the objective cost evaluated at the current trajectory $\xi_i$)*. 

### 6. Hamiltonian Monte Carlo (Escaping Local Minima)
Because gradient descent only finds local minima, CHOMP incorporates the math of Hamiltonian physics to add random exploration. It introduces a momentum variable $\gamma$ for the trajectory and defines a Hamiltonian energy system:
$H(\xi, \gamma) = U(\xi) + K(\gamma)$.
Here, the objective cost $U(\xi)$ acts as "potential energy" and $K(\gamma)$ is "kinetic energy". By simulating physical dynamics—allowing the trajectory to "roll" with momentum—CHOMP can temporarily increase its cost to roll up and out of high-cost local minima, conserving total energy $H$ before settling into a better, lower-cost valley.