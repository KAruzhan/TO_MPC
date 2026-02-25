#!/usr/bin/env python3

import argparse
import numpy as np
import matplotlib.pyplot as plt


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot actual simulation joint position/velocity/acceleration."
    )
    parser.add_argument(
        "--csv",
        default="solver_stats_to_high.csv", # solver_stats_to_high
        help="Path to solver stats csv",
    )
    parser.add_argument(
        "--t-start",
        type=float,
        default=None,
        help="Optional start time [s] after initial timestamp",
    )
    parser.add_argument(
        "--t-end",
        type=float,
        default=None,
        help="Optional end time [s] after initial timestamp",
    )
    return parser.parse_args()


def load_actual_data(path):
    # Stats CSV can exist in two schemas:
    # A) sec,nsec,sqp,exec,ocp,kkt,nan,goal,q(6),qd(6),q_goal(6),vel_cmd(6),u_cmd(6) -> q starts at 8
    # B) sec,nsec,sqp,exec,ocp,nan,goal,q(6),qd(6),q_goal(6),vel_cmd(6),u_cmd(6)     -> q starts at 7
    # Also, each line may end with a trailing comma (extra NaN column).
    data = np.genfromtxt(path, delimiter=",", filling_values=np.nan, invalid_raise=False)
    if data.size == 0:
        raise RuntimeError(f"Empty or unreadable CSV: {path}")
    if data.ndim == 1:
        data = data.reshape(1, -1)

    # Drop all-NaN trailing columns from trailing comma.
    data = data[:, ~np.all(np.isnan(data), axis=0)]

    # With new logging (u_cmd(6)):
    # schema A has 38 columns, schema B has 37 columns (after dropping trailing NaN col).
    if data.shape[1] >= 38:
        q0 = 8
    elif data.shape[1] >= 37:
        q0 = 7
    else:
        raise RuntimeError(f"Unexpected CSV schema with {data.shape[1]} columns in {path}")

    # logger uses now.nanoseconds() (epoch nanoseconds), so build time from ns directly.
    ns = data[:, 1]
    t = 1e-9 * (ns - ns[0])
    q = data[:, q0 : q0 + 6]
    qd = data[:, q0 + 6 : q0 + 12]

    # Keep only fully valid rows and strictly increasing timestamps.
    valid = np.isfinite(t) & np.all(np.isfinite(q), axis=1) & np.all(np.isfinite(qd), axis=1)
    t = t[valid]
    q = q[valid]
    qd = qd[valid]
    if t.size < 3:
        raise RuntimeError(f"Not enough valid samples in {path}")
    inc = np.concatenate(([True], np.diff(t) > 0.0))
    t = t[inc]
    q = q[inc]
    qd = qd[inc]

    u0 = q0 + 24  # q(6), qd(6), q_goal(6), vel_cmd(6)
    u = None
    if data.shape[1] >= u0 + 6:
        u = data[:, u0 : u0 + 6]
        u = u[valid]
        u = u[inc]

    return t, q, qd, u


def apply_time_window(t, mats, t_start, t_end):
    mask = np.ones_like(t, dtype=bool)
    if t_start is not None:
        mask &= t >= t_start
    if t_end is not None:
        mask &= t <= t_end
    return t[mask], [m[mask] for m in mats]


def plot_group(ax, t, mat, ylabel):
    for j in range(6):
        ax.plot(t, mat[:, j], label=f"j{j+1}")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=3, fontsize=8)


def main():
    args = parse_args()
    t, q, qd, u = load_actual_data(args.csv)
    mats = [q, qd] if u is None else [q, qd, u]
    t, mats = apply_time_window(t, mats, args.t_start, args.t_end)
    if u is None:
        q, qd = mats
    else:
        q, qd, u = mats

    n_rows = 3 if u is not None else 2
    fig, axs = plt.subplots(n_rows, 1, figsize=(12, 10 if u is not None else 8), sharex=True)
    fig.suptitle("Actual Simulation Trajectory (all joints)")

    plot_group(axs[0], t, q, "position [rad]")
    plot_group(axs[1], t, qd, "velocity [rad/s]")
    if u is not None:
        plot_group(axs[2], t, u, "mpc u (accel) [rad/s^2]")
        axs[2].set_xlabel("time [s]")
    else:
        axs[1].set_xlabel("time [s]")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
