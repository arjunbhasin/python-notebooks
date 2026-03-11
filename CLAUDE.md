# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Educational Jupyter notebook collection with from-scratch implementations of advanced mathematics, robotics, and reinforcement learning. Each notebook includes rigorous derivations, NumPy/SciPy implementations, and visualizations — no black-box libraries.

## Running Notebooks

```bash
jupyter notebook <path/to/notebook.ipynb>
```

No build system, test framework, or CI/CD exists. Dependencies (numpy, scipy, matplotlib, stable-baselines3) are installed manually via pip.

## Repository Structure

- `maths/` — Linear algebra (Lie groups, tensors, spectral graph theory), optimization (convex, SQP, optimal transport), probability (GPs, stochastic calculus, information geometry), calculus of variations, differential geometry
- `robotics/` — Control (PID, LQR, CBF-Lyapunov, MPC), motion planning (RRT, potential fields, CHOMP, trajectory optimization), estimation & dynamics (kinematics, Lagrangian dynamics, Kalman filter)
- `reinforcement-learning/` — From grid-world DP through deep RL (DQN, actor-critic, SAC, DDPG/TD3, trust-region methods)

Each topic lives in its own subdirectory containing a single `.ipynb` notebook. Notebooks are self-contained and independent — no cross-notebook imports.

## Key Conventions

- **From-scratch implementations**: Algorithms are built with NumPy/SciPy, not wrapped from libraries. This is intentional for pedagogical clarity.
- **Self-contained notebooks**: Each notebook stands alone. Do not introduce shared utility modules or cross-notebook dependencies.
- **Structure within notebooks**: Problem statement → mathematical derivation → implementation → visualization → references.
- **Only shared code**: `robotics/planar-nullspace-rl/` is the sole exception with shared Python files (`anim_common.py`, `animate*.py`) for multi-policy animation comparison using trained SAC models.
- **Companion markdown**: Some topics include `.md` theory documents alongside notebooks (e.g., `chomp-details.md`, `kinematics-theory.md`).
