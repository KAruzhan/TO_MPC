#!/usr/bin/env python3
#python3 plot_mpc_horizon.py --row 120

import argparse
import numpy as np
import matplotlib.pyplot as plt


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot MPC horizon position/velocity/acceleration from solver_horizon_high.csv"
    )
    parser.add_argument(
        "--csv",
        default="solver_horizon_to_high.csv",
        help="Path to horizon csv file",
    )
    parser.add_argument(
        "--row",
        default="last",
        help="Row index to plot (0-based) or 'last'",
    )
    return parser.parse_args()


def load_row(csv_path, row_arg):
    raw = np.genfromtxt(csv_path, delimiter=",", names=True)
    if raw.size == 0:
        raise RuntimeError("CSV is empty.")
    if raw.shape == ():
        raw = np.array([raw], dtype=raw.dtype)

    if row_arg == "last":
        row_idx = raw.shape[0] - 1
    else:
        row_idx = int(row_arg)
        if row_idx < 0 or row_idx >= raw.shape[0]:
            raise IndexError(f"row={row_idx} out of range [0, {raw.shape[0]-1}]")

    return raw[row_idx], row_idx


def extract_horizon(row):
    n_steps = int(row["N"])
    q = np.zeros((n_steps, 6))
    qd = np.zeros((n_steps, 6))
    qdd = np.zeros((n_steps, 6))
    dt = np.zeros(n_steps)

    for k in range(n_steps):
        for j in range(6):
            q[k, j] = row[f"q{j+1}_k{k}"]
            qd[k, j] = row[f"qd{j+1}_k{k}"]
            qdd[k, j] = row[f"qdd{j+1}_k{k}"]
        dt[k] = row[f"dt_k{k}"]

    t = np.concatenate(([0.0], np.cumsum(dt[:-1])))
    return t, q, qd, qdd


def plot_group(ax, t, mat, ylabel):
    for j in range(6):
        ax.plot(t, mat[:, j], marker="o", label=f"j{j+1}")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=3, fontsize=8)


def main():
    args = parse_args()
    row, row_idx = load_row(args.csv, args.row)
    t, q, qd, qdd = extract_horizon(row)

    fig, axs = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    goal_id = int(row["goal_id"])
    sec = int(row["sec"])
    nsec = int(row["nsec"])
    fig.suptitle(
        f"MPC Horizon (row={row_idx}, goal_id={goal_id}, time={sec}.{nsec:09d})"
    )

    plot_group(axs[0], t, q, "position [rad]")
    plot_group(axs[1], t, qd, "velocity [rad/s]")
    plot_group(axs[2], t, qdd, "acceleration [rad/s^2]")
    axs[2].set_xlabel("horizon time [s]")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
