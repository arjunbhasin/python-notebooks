"""
Animated comparison of null-space policies tracking a closed polygon.

The EE traces a regular polygon (triangle, square, pentagon, etc.) inside
the workspace, returning to the start vertex. Uses the same SAC model
trained on fixed-point targets — no retraining.

Usage:
    python animate_polygon.py                    # default: pentagon
    python animate_polygon.py --sides 3          # triangle
    python animate_polygon.py --sides 6          # hexagon
    python animate_polygon.py --save             # save to animation_polygon.mp4
    python animate_polygon.py --fps 5            # slower
"""

import argparse
import os

import numpy as np

from anim_common import (
    L, EPISODE_STEPS, SEED,
    jacobian, manipulability, sample_nonsingular_config, forward_kinematics,
    load_sac_model, build_policies, run_waypoint_episode, animate_policies,
)

# Safe annular zone: not too close to base (fold-back) or boundary (full extension)
_MAX_REACH = float(np.sum(L))
_SAFE_INNER = 0.5                  # minimum distance from base
_SAFE_OUTER = _MAX_REACH * 0.55    # maximum distance from base (~1.54 for reach=2.8)


def _waypoint_is_safe(pt):
    """Check a waypoint is in the safe annular zone."""
    d = np.linalg.norm(pt)
    return _SAFE_INNER <= d <= _SAFE_OUTER


def generate_polygon_trajectory(n_sides, n_steps, rng, max_attempts=200):
    """Generate waypoints tracing a closed regular polygon.

    All waypoints are constrained to a safe annular zone well inside the
    reachable workspace, avoiding both near-base fold-back singularities
    and near-boundary full-extension singularities.

    Returns:
        waypoints: shape (n_steps + 1, 2)
        vertices:  shape (n_sides + 1, 2)  — closed (first vertex repeated)
    """
    mid_radius = (_SAFE_INNER + _SAFE_OUTER) / 2   # sweet spot ~1.02

    for _ in range(max_attempts):
        # Random centre: keep it close to mid_radius so polygon fits
        cr = rng.uniform(_SAFE_INNER + 0.1, mid_radius)
        ctheta = rng.uniform(-np.pi, np.pi)
        centre = np.array([cr * np.cos(ctheta), cr * np.sin(ctheta)])

        # Max polygon radius that keeps all vertices inside the safe zone
        # A vertex is at centre ± poly_radius, so worst-case distance from
        # origin is ||centre|| + poly_radius (must be <= _SAFE_OUTER) and
        # ||centre|| - poly_radius (must be >= _SAFE_INNER).
        c_norm = np.linalg.norm(centre)
        max_r = min(_SAFE_OUTER - c_norm, c_norm - _SAFE_INNER)
        if max_r < 0.3:
            continue

        poly_radius = rng.uniform(0.3, max_r)
        rot = rng.uniform(0, 2 * np.pi)

        # Vertices of the closed polygon
        angles = np.linspace(0, 2 * np.pi, n_sides + 1) + rot
        vertices = centre + poly_radius * np.column_stack([np.cos(angles), np.sin(angles)])

        # Validate every vertex is safe
        if not all(_waypoint_is_safe(v) for v in vertices):
            continue

        # Build waypoints along edges
        edge_lengths = np.linalg.norm(np.diff(vertices, axis=0), axis=1)
        total_length = edge_lengths.sum()
        steps_per_edge = np.round(edge_lengths / total_length * n_steps).astype(int)
        steps_per_edge[-1] += n_steps - steps_per_edge.sum()

        segments = []
        for i in range(n_sides):
            n_seg = steps_per_edge[i]
            t_seg = np.linspace(0, 1, n_seg, endpoint=False)
            seg = vertices[i][None, :] + t_seg[:, None] * (vertices[i + 1] - vertices[i])[None, :]
            segments.append(seg)

        waypoints = np.concatenate(segments, axis=0)
        waypoints = np.vstack([waypoints, vertices[0]])

        if len(waypoints) < n_steps + 1:
            waypoints = np.vstack([waypoints, np.tile(vertices[0], (n_steps + 1 - len(waypoints), 1))])
        waypoints = waypoints[:n_steps + 1]

        # Final check: every waypoint in safe zone
        dists = np.linalg.norm(waypoints, axis=1)
        if np.all(dists >= _SAFE_INNER) and np.all(dists <= _SAFE_OUTER):
            return waypoints, vertices

    raise RuntimeError("Could not generate a safe polygon trajectory — try a different seed")


def main():
    parser = argparse.ArgumentParser(description="Animate polygon-tracking with null-space policies")
    parser.add_argument("--sides", type=int, default=5, help="Number of polygon sides (default: 5)")
    parser.add_argument("--save", action="store_true", help="Save to animation_polygon.mp4")
    parser.add_argument("--fps", type=int, default=10, help="Animation FPS (default: 10)")
    parser.add_argument("--seed", type=int, default=SEED, help="Random seed")
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    model = load_sac_model(script_dir)
    policies = build_policies(model)

    # Generate polygon
    rng = np.random.default_rng(args.seed)
    waypoints, vertices = generate_polygon_trajectory(args.sides, EPISODE_STEPS, rng)

    shape_name = {3: "Triangle", 4: "Square", 5: "Pentagon",
                  6: "Hexagon", 8: "Octagon"}.get(args.sides, f"{args.sides}-gon")
    perimeter = np.sum(np.linalg.norm(np.diff(vertices, axis=0), axis=1))
    print(f"{shape_name}: {args.sides} sides, perimeter={perimeter:.2f}, "
          f"centre=({vertices[:-1].mean(0)[0]:.2f}, {vertices[:-1].mean(0)[1]:.2f})")

    # Run all policies
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

    # Path drawing callback: closed polygon + vertex markers
    def draw_path(ax):
        ax.plot(vertices[:, 0], vertices[:, 1], "--", color="gray",
                linewidth=1.2, alpha=0.5, zorder=0)
        ax.plot(vertices[:-1, 0], vertices[:-1, 1], "o", color="gray",
                markersize=5, alpha=0.5, zorder=0)
        # Mark start vertex distinctly
        ax.plot(vertices[0, 0], vertices[0, 1], "s", color="dimgray",
                markersize=7, alpha=0.7, zorder=0)

    save_path = os.path.join(script_dir, "animation_polygon.mp4") if args.save else None
    title = f"{shape_name} Tracking \u2014 Null-Space Policy Comparison (no retraining)"

    animate_policies(
        trajs, manips, ee_trajs, ee_errs, waypoints,
        policy_names, policy_colors,
        title=title, fps=args.fps,
        save_path=save_path, draw_path_fn=draw_path,
    )


if __name__ == "__main__":
    main()
