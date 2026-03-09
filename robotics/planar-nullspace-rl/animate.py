"""
Animated comparison of null-space policies for a planar 4-DOF arm (fixed target).

Usage:
    python animate.py              # interactive window
    python animate.py --save       # save to animation.mp4
    python animate.py --fps 5      # slower playback
"""

import argparse
import os

import numpy as np

from anim_common import (
    L, EPISODE_STEPS, SEED, N_JOINTS, CLIK_GAIN, NULLSPACE_GAIN, MAX_QDOT, DT,
    forward_kinematics, jacobian, damped_pinv, null_space_projector, manipulability,
    sample_nonsingular_config,
    load_sac_model, build_policies, animate_policies,
)


def generate_reachable_target(L, rng):
    max_reach = float(np.sum(L)) * 0.7
    r = rng.uniform(0.5, max_reach)
    theta = rng.uniform(-np.pi, np.pi)
    return np.array([r * np.cos(theta), r * np.sin(theta)])


def run_fixed_target_episode(policy_fn, seed=SEED):
    """Run one episode with a fixed target, return data in waypoint-compatible format."""
    rng_ep = np.random.default_rng(seed)
    q = sample_nonsingular_config(L, rng_ep)
    target = generate_reachable_target(L, rng_ep)
    q_dot_prev = np.zeros(N_JOINTS)

    q_traj = [q.copy()]
    manip_traj = [manipulability(jacobian(q, L))]
    ee_traj = [forward_kinematics(q, L).copy()]
    ee_err_traj = [np.linalg.norm(target - forward_kinematics(q, L))]

    for t in range(EPISODE_STEPS):
        J_t = jacobian(q, L)
        J_dag = damped_pinv(J_t)
        N_t = null_space_projector(J_t)
        ee = forward_kinematics(q, L)
        e_ee = target - ee

        action = policy_fn(q, L, t, q_dot_prev, e_ee)
        q_dot = J_dag @ (CLIK_GAIN * e_ee) + NULLSPACE_GAIN * (N_t @ action)
        q_dot = np.clip(q_dot, -MAX_QDOT, MAX_QDOT)
        q = np.clip(q + q_dot * DT, -np.pi, np.pi)
        q_dot_prev = q_dot

        q_traj.append(q.copy())
        manip_traj.append(manipulability(jacobian(q, L)))
        ee_now = forward_kinematics(q, L)
        ee_traj.append(ee_now.copy())
        ee_err_traj.append(np.linalg.norm(target - ee_now))

    # Constant waypoints (fixed target repeated)
    waypoints = np.tile(target, (EPISODE_STEPS + 1, 1))
    return q_traj, manip_traj, ee_traj, ee_err_traj, waypoints


def main():
    parser = argparse.ArgumentParser(description="Animate fixed-target null-space policies")
    parser.add_argument("--save", action="store_true", help="Save to animation.mp4")
    parser.add_argument("--fps", type=int, default=10, help="Animation FPS (default: 10)")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    model = load_sac_model(script_dir)
    policies = build_policies(model)

    print("Running episodes ...")
    policy_names = list(policies.keys())
    policy_colors = [policies[n][1] for n in policy_names]
    trajs, manips, ee_trajs, ee_errs = {}, {}, {}, {}
    waypoints = None
    for name, (fn, _) in policies.items():
        q_traj, m_traj, ee_traj, e_traj, wp = run_fixed_target_episode(fn)
        trajs[name] = np.array(q_traj)
        manips[name] = np.array(m_traj)
        ee_trajs[name] = np.array(ee_traj)
        ee_errs[name] = np.array(e_traj)
        waypoints = wp

    save_path = os.path.join(script_dir, "animation.mp4") if args.save else None
    animate_policies(
        trajs, manips, ee_trajs, ee_errs, waypoints,
        policy_names, policy_colors,
        title="Fixed Target \u2014 Null-Space Policy Comparison",
        fps=args.fps, save_path=save_path,
    )


if __name__ == "__main__":
    main()
