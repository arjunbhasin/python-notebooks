"""
Animated comparison of null-space policies tracking a straight-line trajectory.

Uses the same SAC models trained on fixed-point targets (no retraining).

Usage:
    python animate_line.py                  # interactive window
    python animate_line.py --save           # save to animation_line.mp4
    python animate_line.py --fps 5          # slower playback
"""

import argparse
import os

import numpy as np

from anim_common import (
    L, EPISODE_STEPS, SEED,
    load_sac_model, build_policies, run_waypoint_episode, animate_policies,
)

# Safe annular zone (same as polygon script)
_MAX_REACH = float(np.sum(L))
_SAFE_INNER = 0.5
_SAFE_OUTER = _MAX_REACH * 0.55


def generate_line_trajectory(rng, n_steps):
    def sample_point():
        r = rng.uniform(_SAFE_INNER + 0.1, _SAFE_OUTER - 0.1)
        theta = rng.uniform(-np.pi, np.pi)
        return np.array([r * np.cos(theta), r * np.sin(theta)])

    # Keep sampling until every waypoint on the line stays in the safe zone
    for _ in range(200):
        start = sample_point()
        end = sample_point()
        if np.linalg.norm(end - start) < 0.6:
            continue

        t_param = np.linspace(0, 1, n_steps + 1)
        waypoints = start[None, :] + t_param[:, None] * (end - start)[None, :]
        dists = np.linalg.norm(waypoints, axis=1)
        if np.all(dists >= _SAFE_INNER) and np.all(dists <= _SAFE_OUTER):
            return waypoints, start, end

    raise RuntimeError("Could not generate a safe line trajectory — try a different seed")


def main():
    parser = argparse.ArgumentParser(description="Animate line-tracking with null-space policies")
    parser.add_argument("--save", action="store_true", help="Save to animation_line.mp4")
    parser.add_argument("--fps", type=int, default=10, help="Animation FPS (default: 10)")
    parser.add_argument("--seed", type=int, default=SEED, help="Random seed for trajectory")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    model = load_sac_model(script_dir)
    policies = build_policies(model)

    rng = np.random.default_rng(args.seed)
    waypoints, line_start, line_end = generate_line_trajectory(rng, EPISODE_STEPS)
    print(f"Line: ({line_start[0]:.2f}, {line_start[1]:.2f}) -> "
          f"({line_end[0]:.2f}, {line_end[1]:.2f})  "
          f"length={np.linalg.norm(line_end - line_start):.2f}")

    print("Running episodes ...")
    policy_names = list(policies.keys())
    policy_colors = [policies[n][1] for n in policy_names]
    trajs, manips, ee_trajs, ee_errs = {}, {}, {}, {}
    for name, (fn, _) in policies.items():
        q_traj, m_traj, ee_traj, e_traj = run_waypoint_episode(fn, waypoints, seed=args.seed)
        trajs[name] = q_traj
        manips[name] = m_traj
        ee_trajs[name] = ee_traj
        ee_errs[name] = e_traj

    def draw_path(ax):
        ax.plot(waypoints[:, 0], waypoints[:, 1], "--", color="gray",
                linewidth=1, alpha=0.4, zorder=0)
        ax.plot(line_start[0], line_start[1], "s", color="gray",
                markersize=6, alpha=0.5, zorder=0)
        ax.plot(line_end[0], line_end[1], "D", color="gray",
                markersize=6, alpha=0.5, zorder=0)

    save_path = os.path.join(script_dir, "animation_line.mp4") if args.save else None
    animate_policies(
        trajs, manips, ee_trajs, ee_errs, waypoints,
        policy_names, policy_colors,
        title="Line Tracking \u2014 Null-Space Policy Comparison (no retraining)",
        fps=args.fps, save_path=save_path, draw_path_fn=draw_path,
    )


if __name__ == "__main__":
    main()
