"""Shared kinematics, policies, drawing, and animation layout for all animate_*.py scripts."""

import os
import sys

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib import animation
from stable_baselines3 import SAC

# ── Constants (must match notebook) ──────────────────────────────────────────

LINK_LENGTHS = np.array([1.0, 0.8, 0.6, 0.4])
L = LINK_LENGTHS
N_JOINTS = 4
N_TASK = 2
DT = 0.05
EPISODE_STEPS = 200
CLIK_GAIN = 10.0
NULLSPACE_GAIN = 0.3
DAMPING_LAMBDA = 0.01
MAX_QDOT = 15.0
SEED = 42

C_BLUE, C_GOLD, C_RED = "steelblue", "goldenrod", "coral"
JOINT_COLORS = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3"]
JOINT_LABELS = ["q1", "q2", "q3", "q4"]

# ── Kinematics ───────────────────────────────────────────────────────────────

def forward_kinematics(q, L):
    phi = np.cumsum(q)
    return np.array([np.sum(L * np.cos(phi)), np.sum(L * np.sin(phi))])


def all_joint_positions(q, L):
    phi = np.cumsum(q)
    x = np.concatenate([[0.0], np.cumsum(L * np.cos(phi))])
    y = np.concatenate([[0.0], np.cumsum(L * np.sin(phi))])
    return np.column_stack([x, y])


def jacobian(q, L):
    n = len(q)
    phi = np.cumsum(q)
    J = np.zeros((2, n))
    for k in range(n):
        J[0, k] = -np.sum(L[k:] * np.sin(phi[k:]))
        J[1, k] = np.sum(L[k:] * np.cos(phi[k:]))
    return J


def damped_pinv(J, lam=DAMPING_LAMBDA):
    m = J.shape[0]
    return J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(m))


def null_space_projector(J, rank_tol=1e-8):
    U, S, Vt = np.linalg.svd(J, full_matrices=True)
    rank = np.sum(S > rank_tol)
    V_null = Vt[rank:]
    return V_null.T @ V_null


def manipulability(J):
    return np.sqrt(max(np.linalg.det(J @ J.T), 0.0))


# ── Sampling ─────────────────────────────────────────────────────────────────

def sample_nonsingular_config(L, rng, min_manip=0.05, max_attempts=100):
    for _ in range(max_attempts):
        q = rng.uniform(-0.6 * np.pi, 0.6 * np.pi, len(L))
        if manipulability(jacobian(q, L)) > min_manip:
            return q
    return q


# ── Baseline policies ────────────────────────────────────────────────────────

def manipulability_gradient(q, L, eps=1e-4):
    grad = np.zeros(len(q))
    for k in range(len(q)):
        dq = np.zeros(len(q))
        dq[k] = eps
        grad[k] = (manipulability(jacobian(q + dq, L))
                    - manipulability(jacobian(q - dq, L))) / (2 * eps)
    return grad


def zero_secondary(q, L):
    return np.zeros(len(q))


def wrap_baseline(fn):
    return lambda q, L, t, qdp, eee: fn(q, L)


def wrap_learned(model, task_conditioned):
    def policy_fn(q, L, t, q_dot_prev, e_ee):
        phase = 1.0 if (task_conditioned and t >= EPISODE_STEPS // 2) else 0.0
        obs = np.concatenate([q / np.pi, q_dot_prev, e_ee, [phase]]).astype(np.float32)
        action, _ = model.predict(obs, deterministic=True)
        return action
    return policy_fn


# ── Model loading ────────────────────────────────────────────────────────────

def load_sac_model(script_dir):
    model_dir = os.path.join(script_dir, "saved_models")
    conditioned_path = os.path.join(model_dir, "sac_conditioned.zip")
    if not os.path.exists(conditioned_path):
        print(f"ERROR: No saved model found at {conditioned_path}")
        print("Run the notebook first to train and save models.")
        sys.exit(1)
    print("Loading SAC (Conditioned) model ...")
    return SAC.load(os.path.join(model_dir, "sac_conditioned"))


def build_policies(model_conditioned):
    return {
        "Zero Baseline": (wrap_baseline(zero_secondary), C_BLUE),
        "Manip Gradient": (wrap_baseline(lambda q, L: manipulability_gradient(q, L)), C_GOLD),
        "SAC (Learned)": (wrap_learned(model_conditioned, True), C_RED),
    }


# ── Episode runner (generic waypoint tracking) ──────────────────────────────

def run_waypoint_episode(policy_fn, waypoints, seed=SEED):
    """Run one episode tracking a sequence of waypoints.

    Args:
        policy_fn: callable(q, L, t, q_dot_prev, e_ee) -> action
        waypoints: shape (EPISODE_STEPS + 1, 2)
        seed: random seed for initial config
    Returns:
        q_traj, manip_traj, ee_traj, ee_err_traj
    """
    rng_ep = np.random.default_rng(seed)
    q = sample_nonsingular_config(L, rng_ep)
    q_dot_prev = np.zeros(N_JOINTS)

    q_traj = [q.copy()]
    manip_traj = [manipulability(jacobian(q, L))]
    ee_traj = [forward_kinematics(q, L).copy()]
    ee_err_traj = [np.linalg.norm(waypoints[0] - forward_kinematics(q, L))]

    for t in range(EPISODE_STEPS):
        target_t = waypoints[t]
        J_t = jacobian(q, L)
        J_dag = damped_pinv(J_t)
        N_t = null_space_projector(J_t)
        ee = forward_kinematics(q, L)
        e_ee = target_t - ee

        action = policy_fn(q, L, t, q_dot_prev, e_ee)
        q_dot = J_dag @ (CLIK_GAIN * e_ee) + NULLSPACE_GAIN * (N_t @ action)
        q_dot = np.clip(q_dot, -MAX_QDOT, MAX_QDOT)
        q = np.clip(q + q_dot * DT, -np.pi, np.pi)
        q_dot_prev = q_dot

        q_traj.append(q.copy())
        manip_traj.append(manipulability(jacobian(q, L)))
        ee_now = forward_kinematics(q, L)
        ee_traj.append(ee_now.copy())
        ee_err_traj.append(np.linalg.norm(waypoints[t + 1] - ee_now))

    return np.array(q_traj), np.array(manip_traj), np.array(ee_traj), np.array(ee_err_traj)


# ── Drawing ──────────────────────────────────────────────────────────────────

def draw_arm(ax, q, L, target=None, color=C_BLUE, ghost_q=None):
    if ghost_q is not None:
        ghost_pos = all_joint_positions(ghost_q, L)
        ax.plot(ghost_pos[:, 0], ghost_pos[:, 1], "-o", color=color,
                linewidth=1.5, markersize=4, alpha=0.15, zorder=1)

    pos = all_joint_positions(q, L)
    ax.plot(pos[:, 0], pos[:, 1], "-o", color=color, linewidth=2.5,
            markersize=6, solid_capstyle="round", zorder=3)
    ax.plot(0, 0, "ks", markersize=8, zorder=4)
    ax.plot(pos[-1, 0], pos[-1, 1], "o", color="limegreen", markersize=10, zorder=5)
    if target is not None:
        ax.plot(target[0], target[1], "r*", markersize=14, zorder=5)
    ax.set_aspect("equal")
    ax.set_xlim(-3.2, 3.2)
    ax.set_ylim(-3.2, 3.2)


# ── Animation scaffold ──────────────────────────────────────────────────────

def animate_policies(trajs, manips, ee_trajs, ee_errs, waypoints,
                     policy_names, policy_colors, title, fps,
                     save_path=None, draw_path_fn=None):
    """Generic animation loop shared across all trajectory types.

    Args:
        trajs:         dict name -> (EPISODE_STEPS+1, N_JOINTS)
        manips:        dict name -> (EPISODE_STEPS+1,)
        ee_trajs:      dict name -> (EPISODE_STEPS+1, 2)
        ee_errs:       dict name -> (EPISODE_STEPS+1,)
        waypoints:     (EPISODE_STEPS+1, 2)
        policy_names:  list[str]
        policy_colors: list[str]
        title:         str  — figure suptitle
        fps:           int
        save_path:     str or None — if set, save mp4 instead of showing
        draw_path_fn:  callable(ax) — draws the reference path on each arm axes
    """
    n_policies = len(policy_names)
    steps = np.arange(EPISODE_STEPS + 1)

    fig = plt.figure(figsize=(18, 14))
    gs = GridSpec(3, n_policies, figure=fig,
                  height_ratios=[3, 1, 1.2],
                  hspace=0.30, wspace=0.30)

    ax_arms = [fig.add_subplot(gs[0, i]) for i in range(n_policies)]
    ax_bars = [fig.add_subplot(gs[1, i]) for i in range(n_policies)]
    ax_manip = fig.add_subplot(gs[2, 0:2])
    ax_eerr = fig.add_subplot(gs[2, 2])

    # Manipulability background
    manip_lines = []
    for name, color in zip(policy_names, policy_colors):
        ax_manip.plot(steps, manips[name], color=color, alpha=0.15, linewidth=1)
        (line,) = ax_manip.plot([], [], color=color, linewidth=2.5, label=name)
        manip_lines.append(line)
    ax_manip.axvline(EPISODE_STEPS // 2, color="gray", linestyle=":", alpha=0.5,
                     label="Phase switch")
    ax_manip.set_xlabel("Step"); ax_manip.set_ylabel("Manipulability w")
    ax_manip.set_title("Manipulability over Episode")
    ax_manip.legend(fontsize=8, loc="lower right")
    ax_manip.set_xlim(0, EPISODE_STEPS)
    ax_manip.set_ylim(0, max(m.max() for m in manips.values()) * 1.15)
    ax_manip.grid(True, alpha=0.3)

    # EE error background
    eerr_lines = []
    for name, color in zip(policy_names, policy_colors):
        ax_eerr.plot(steps, ee_errs[name], color=color, alpha=0.15, linewidth=1)
        (line,) = ax_eerr.plot([], [], color=color, linewidth=2.5, label=name)
        eerr_lines.append(line)
    ax_eerr.set_xlabel("Step"); ax_eerr.set_ylabel("EE Tracking Error")
    ax_eerr.set_title("EE Error (distance to waypoint)")
    ax_eerr.legend(fontsize=7, loc="upper right")
    ax_eerr.set_xlim(0, EPISODE_STEPS)
    ax_eerr.set_ylim(0, max(e.max() for e in ee_errs.values()) * 1.1)
    ax_eerr.grid(True, alpha=0.3)

    time_text = fig.text(0.5, 0.97, "", ha="center", fontsize=14,
                         fontweight="bold", transform=fig.transFigure)

    step_stride = 2
    frame_indices = list(range(0, EPISODE_STEPS + 1, step_stride))
    if frame_indices[-1] != EPISODE_STEPS:
        frame_indices.append(EPISODE_STEPS)
    n_frames = len(frame_indices)
    GHOST_LAG = 20

    def init():
        for ax in ax_arms + ax_bars:
            ax.clear()
        for line in manip_lines + eerr_lines:
            line.set_data([], [])
        time_text.set_text("")
        return []

    def update(frame_idx):
        t = frame_indices[frame_idx]
        ghost_t = max(0, t - GHOST_LAG)
        target_now = waypoints[t]

        for i, (name, ax_arm, ax_bar) in enumerate(zip(policy_names, ax_arms, ax_bars)):
            color = policy_colors[i]
            q_now = trajs[name][t]
            q_ghost = trajs[name][ghost_t] if t > GHOST_LAG else None

            ax_arm.clear()

            # Draw reference path
            if draw_path_fn is not None:
                draw_path_fn(ax_arm)

            # EE trail
            ee_trail = ee_trajs[name][:t + 1]
            if len(ee_trail) > 1:
                ax_arm.plot(ee_trail[:, 0], ee_trail[:, 1], "-", color=color,
                            alpha=0.35, linewidth=1.5, zorder=1)

            draw_arm(ax_arm, q_now, L, target=target_now, color=color, ghost_q=q_ghost)
            w = manips[name][t]
            ee_e = ee_errs[name][t]
            ax_arm.set_title(f"{name}\nw = {w:.3f}   ee = {ee_e:.4f}",
                             fontsize=11, fontweight="bold")
            ax_arm.grid(True, alpha=0.15)

            # Joint angle bars
            ax_bar.clear()
            angles_deg = np.degrees(q_now)
            bars = ax_bar.bar(JOINT_LABELS, angles_deg, color=JOINT_COLORS,
                              edgecolor="white", linewidth=0.5)
            ax_bar.set_ylim(-200, 200)
            ax_bar.axhline(0, color="gray", linewidth=0.5)
            ax_bar.axhline(180, color="red", linewidth=0.5, linestyle="--", alpha=0.3)
            ax_bar.axhline(-180, color="red", linewidth=0.5, linestyle="--", alpha=0.3)
            ax_bar.set_ylabel("deg", fontsize=9)
            ax_bar.grid(True, alpha=0.2, axis="y")
            for bar, val in zip(bars, angles_deg):
                ax_bar.text(bar.get_x() + bar.get_width() / 2,
                            val + (8 if val >= 0 else -16),
                            f"{val:.0f}", ha="center", fontsize=8, fontweight="bold")

        for j, (name, ml, el) in enumerate(zip(policy_names, manip_lines, eerr_lines)):
            ml.set_data(steps[:t + 1], manips[name][:t + 1])
            el.set_data(steps[:t + 1], ee_errs[name][:t + 1])

        pct = t / EPISODE_STEPS * 100
        phase = ("Phase 2 \u2014 joint-limit avoidance" if t >= EPISODE_STEPS // 2
                 else "Phase 1 \u2014 manipulability maximisation")
        time_text.set_text(f"Step {t}/{EPISODE_STEPS}  ({pct:.0f}%)  \u2014  {phase}")
        return []

    print(f"Animating {n_frames} frames at {fps} fps "
          f"(~{n_frames / fps:.0f}s per loop) ...")

    anim = animation.FuncAnimation(
        fig, update, init_func=init, frames=n_frames,
        interval=1000 // fps, blit=False, repeat=True,
    )

    if save_path:
        print(f"Saving to {save_path} ...")
        anim.save(save_path, writer="ffmpeg", fps=fps, dpi=150)
        print("Done!")
    else:
        fig.suptitle(title, fontsize=15, fontweight="bold", y=0.995)
        plt.show()
