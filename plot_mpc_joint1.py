#!/usr/bin/env python3

import argparse
import numpy as np
import matplotlib.pyplot as plt


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot MPC horizon and closed-loop traces for all joints."
    )
    parser.add_argument("--row", default="last", help="Horizon row index or 'last'")
    parser.add_argument(
        "--horizon-csv",
        default="solver_horizon_to_high.csv",
        help="Path to horizon csv",
    )
    parser.add_argument(
        "--stats-csv",
        default="solver_stats_high.csv",
        help="Path to stats csv",
    )
    parser.add_argument(
        "--acc-limit",
        type=float,
        default=4.4,
        help="Acceleration limit reference lines",
    )
    parser.add_argument(
        "--active-joint",
        type=int,
        default=1,
        help="Joint expected to move in single-joint test [1..6]",
    )
    return parser.parse_args()


def load_horizon_row(path, row_arg):
    raw = np.genfromtxt(path, delimiter=",", names=True)
    if raw.size == 0:
        raise RuntimeError(f"Empty horizon file: {path}")
    if raw.shape == ():
        raw = np.array([raw], dtype=raw.dtype)

    idx = raw.shape[0] - 1 if row_arg == "last" else int(row_arg)
    if idx < 0 or idx >= raw.shape[0]:
        raise IndexError(f"row={idx} out of range [0, {raw.shape[0]-1}]")
    return raw[idx], idx


def extract_horizon_all(row):
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


def load_stats_all(path):
    # solver_stats_high.csv lines have a trailing comma; read only needed numeric columns.
    data = np.genfromtxt(
        path,
        delimiter=",",
        usecols=range(32),
        filling_values=np.nan,
        invalid_raise=False,
    )
    if data.ndim == 1:
        data = data.reshape(1, -1)

    t = data[:, 0] + 1e-9 * data[:, 1]
    t = t - t[0]
    q_curr = data[:, 8:14]
    q_goal = data[:, 20:26]
    vel_cmd = data[:, 26:32]
    return t, q_curr, q_goal, vel_cmd


def plot_all_joints(ax, t, mat, ylabel):
    for j in range(6):
        ax.plot(t, mat[:, j], marker="o", label=f"j{j+1}")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=3, fontsize=8)


def main():
    args = parse_args()
    if args.active_joint < 1 or args.active_joint > 6:
        raise ValueError("active-joint must be between 1 and 6")

    row, row_idx = load_horizon_row(args.horizon_csv, args.row)
    t_h, q_h, qd_h, qdd_h = extract_horizon_all(row)
    sec = int(row["sec"])
    nsec = int(row["nsec"])

    # Figure 1: same style as previous horizon plot (all joints).
    fig1, axs1 = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig1.suptitle(f"Horizon row={row_idx}, time={sec}.{nsec:09d}")

    plot_all_joints(axs1[0], t_h, q_h, "position [rad]")
    plot_all_joints(axs1[1], t_h, qd_h, "velocity [rad/s]")
    plot_all_joints(axs1[2], t_h, qdd_h, "acceleration [rad/s^2]")
    axs1[2].axhline(args.acc_limit, linestyle="--", color="r", linewidth=1)
    axs1[2].axhline(-args.acc_limit, linestyle="--", color="r", linewidth=1)
    axs1[2].set_xlabel("horizon time [s]")
    plt.tight_layout()

    # Figure 2: verify single-joint setup in closed loop.
    t_s, q_curr, q_goal, vel_cmd = load_stats_all(args.stats_csv)
    q_err = q_goal - q_curr

    fig2, axs2 = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig2.suptitle("Closed-loop check: all joints")

    for j in range(6):
        axs2[0].plot(t_s, q_curr[:, j], label=f"q{j+1}")
    axs2[0].set_ylabel("q current [rad]")
    axs2[0].grid(True, alpha=0.3)
    axs2[0].legend(ncol=3, fontsize=8)

    for j in range(6):
        axs2[1].plot(t_s, q_err[:, j], label=f"e{j+1}")
    axs2[1].set_ylabel("q error=goal-current [rad]")
    axs2[1].grid(True, alpha=0.3)
    axs2[1].legend(ncol=3, fontsize=8)

    for j in range(6):
        axs2[2].plot(t_s, vel_cmd[:, j], label=f"u{j+1}")
    axs2[2].set_ylabel("velocity cmd [rad/s]")
    axs2[2].set_xlabel("time [s]")
    axs2[2].grid(True, alpha=0.3)
    axs2[2].legend(ncol=3, fontsize=8)

    active = args.active_joint - 1
    rms_err = np.sqrt(np.mean(q_err * q_err, axis=0))
    other_mean = float(np.mean(np.delete(rms_err, active)))
    text = f"RMS error active j{args.active_joint}: {rms_err[active]:.4f} rad, others mean: {other_mean:.4f} rad"
    axs2[1].text(
        0.01,
        0.92,
        text,
        transform=axs2[1].transAxes,
        bbox=dict(facecolor="white", alpha=0.8, edgecolor="none"),
    )

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
