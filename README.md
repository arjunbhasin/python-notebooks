# Mathematics & Robotics Notebooks

A collection of from-scratch, self-contained Jupyter notebooks covering advanced topics in mathematics and robotics. Every notebook includes rigorous derivations, NumPy/SciPy implementations, and rich visualizations — no black-box libraries.

---

## Robotics

### Control

| Notebook | Objective |
|----------|-----------|
| [PID Control](robotics/pid-control/pid_control.ipynb) | Proportional-Integral-Derivative control: continuous/discrete formulations, Routh-Hurwitz stability, Ziegler-Nichols tuning, cart-pole and drone applications |
| [LQR Optimal Control](robotics/lqr-control/lqr_control.ipynb) | Linear Quadratic Regulator: Algebraic Riccati Equation derivation via HJB, iterative and direct ARE solvers, cart-pole balancing and quadrotor stabilization |
| [Lyapunov Stability & Control Barrier Functions](robotics/cbf-lyapunov/cbf_lyapunov.ipynb) | Lyapunov's direct method, Control Lyapunov Functions, Control Barrier Functions, CLF-CBF-QP synthesis for provably safe control, adaptive cruise control |
| [MPC for Motion Planning](robotics/mpc-planning/mpc_planning.ipynb) | Linear MPC as QP, nonlinear MPC via direct collocation, real-time iteration scheme, obstacle avoidance constraints, unicycle navigation and quadrotor tracking |

### Motion Planning

| Notebook | Objective |
|----------|-----------|
| [RRT & RRT* Path Planning](robotics/rrt-planning/rrt_planning.ipynb) | Sampling-based planning: configuration space, Rapidly-exploring Random Trees, probabilistic completeness, RRT* with rewiring for asymptotically optimal paths |
| [Artificial Potential Fields](robotics/potential-fields/potential_fields.ipynb) | Reactive navigation via attractive/repulsive potentials, gradient descent navigation, local minima solutions, navigation functions, multi-robot coordination |
| [CHOMP Algorithm](robotics/chomp-algo/chomp_3r_planar.ipynb) | Covariant Hamiltonian Optimization for Motion Planning: objective functionals, functional gradients, covariant updates for a 3-link planar arm with obstacles |
| [Trajectory Optimization](robotics/trajectory-optimization/trajectory_optimization.ipynb) | Direct methods for trajectory optimization: single shooting, multiple shooting, Hermite-Simpson collocation, cart-pole swing-up and minimum-time problems |

### Estimation & Dynamics

| Notebook | Objective |
|----------|-----------|
| [Forward & Inverse Kinematics](robotics/kinematics/kinematics.ipynb) | Homogeneous transforms, Denavit-Hartenberg convention, geometric Jacobian, analytical and numerical inverse kinematics, singularity analysis |
| [Lagrangian Dynamics](robotics/lagrangian-dynamics/lagrangian_dynamics.ipynb) | Euler-Lagrange equations for multi-body systems, standard manipulator equation $M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau$, computed torque control |
| [Kalman Filter & Sensor Fusion](robotics/kalman-filter/kalman_filter.ipynb) | Bayesian estimation, predict-update cycle derivation, Extended Kalman Filter for nonlinear systems, IMU+GPS sensor fusion |

---

## Mathematics

### Linear Algebra

| Notebook | Objective |
|----------|-----------|
| [Lie Groups & Lie Algebras](maths/linear-algebra/lie-groups/lie_groups.ipynb) | SO(3) and SE(3) for robotics: exponential/logarithmic maps, Rodrigues' formula, adjoint representation, differential kinematics on Lie groups |
| [Kronecker Products & Matrix Equations](maths/linear-algebra/kronecker-products/kronecker_products.ipynb) | Vectorization framework for solving Lyapunov, Sylvester, and coupled LTI systems with applications in control theory and estimation |
| [Multilinear Algebra & Tensors](maths/linear-algebra/tensors/tensors.ipynb) | Tensor fundamentals, CP and Tucker decompositions, HOSVD, Einstein summation, robotics applications (inertia tensors, manipulability) |
| [Spectral Graph Theory](maths/linear-algebra/spectral-graph-theory/spectral_graph_theory.ipynb) | Graph Laplacian eigenvalues, algebraic connectivity, spectral clustering, applications in communication networks and multi-robot formation control |
| [Optimization on Matrix Manifolds](maths/linear-algebra/matrix-manifolds/matrix_manifolds.ipynb) | Stiefel and Grassmann manifolds, Riemannian gradients, retraction maps, Riemannian gradient descent, rotation averaging on SO(3) |

### Optimization

| Notebook | Objective |
|----------|-----------|
| [Convex Optimization & Duality](maths/optimization/convex-optimization/convex_optimization.ipynb) | Convex sets and functions, Lagrangian duality, KKT conditions, gradient descent, proximal methods, QP and SOCP solvers from scratch |
| [SQP & Interior Point Methods](maths/optimization/sqp-interior-point/sqp_interior_point.ipynb) | Sequential Quadratic Programming and interior-point algorithms for constrained nonlinear optimization, trajectory planning applications |
| [Pontryagin's Maximum Principle](maths/optimization/pontryagin/pontryagin.ipynb) | Hamiltonian framework for optimal control, costate equations, bang-bang control, shooting methods, singular arcs |
| [Optimal Transport](maths/optimization/optimal-transport/optimal_transport.ipynb) | Monge and Kantorovich formulations, Wasserstein distances, entropic regularization, Sinkhorn algorithm, multi-robot task allocation |
| [Variational Integrators](maths/optimization/variational-integrators/variational_integrators.ipynb) | Structure-preserving numerical integration via discrete Lagrangian mechanics, symplecticity, discrete Noether's theorem |

### Calculus & Geometry

| Notebook | Objective |
|----------|-----------|
| [Calculus of Variations](maths/calculus-of-variations/calculus_of_variations.ipynb) | Functionals, Euler-Lagrange equation derivation, shortest path and Brachistochrone problems, numerical variational optimization |
| [Differential Geometry](maths/differential-geometry/differential_geometry.ipynb) | Curves (arc length, curvature, torsion, Frenet-Serret frame), surfaces (fundamental forms, Gaussian curvature), geodesics, trajectory smoothing |

### Probability & Stochastic Methods

| Notebook | Objective |
|----------|-----------|
| [Monte Carlo Methods & Particle Filters](maths/probability/monte-carlo/monte_carlo.ipynb) | Monte Carlo integration, importance sampling, MCMC (Metropolis-Hastings), particle filters for sequential Bayesian estimation |
| [Gaussian Processes](maths/probability/gaussian-processes/gaussian_processes.ipynb) | Kernel functions, GP regression, Bayesian nonparametric inference, hyperparameter optimization, uncertainty quantification |
| [Stochastic Calculus & Itô Theory](maths/probability/stochastic-calculus/stochastic_calculus.ipynb) | Brownian motion, Itô's lemma, stochastic differential equations, Euler-Maruyama and Milstein methods, Fokker-Planck equation |
| [Concentration Inequalities](maths/probability/concentration-inequalities/concentration_inequalities.ipynb) | Markov/Chebyshev/Chernoff/Hoeffding/Bernstein bounds, sub-Gaussian theory, matrix concentration, Johnson-Lindenstrauss lemma |
| [Information Geometry](maths/probability/information-geometry/information_geometry.ipynb) | Fisher information metric, statistical manifolds, natural gradient, differential-geometric structure of probability distributions |
