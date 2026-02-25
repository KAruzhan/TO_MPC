#!/usr/bin/env python3

import argparse
import numpy as np


def _safe_genfromtxt(path, usecols):
    data = np.genfromtxt(
        path,
        delimiter=",",
        usecols=usecols,
        filling_values=np.nan,
        invalid_raise=False,
    )
    if data.size == 0:
        raise RuntimeError(f"No readable rows in {path}")
    if data.ndim == 1:
        data = data.reshape(1, -1)
    return data


def _pick_goal_column(data):
    # mpc_high writes goal_id at col 6, mpc_to writes it at col 7.
    candidates = [6, 7]
    stats = []
    for c in candidates:
        g = data[:, c]
        valid = np.isfinite(g)
        if not np.any(valid):
            continue
        gi = np.rint(g[valid]).astype(int)
        if gi.size < 2:
            continue
        switches = int(np.sum(gi[1:] != gi[:-1]))
        in01 = np.mean((gi == 0) | (gi == 1))
        repeats = np.mean(gi[1:] == gi[:-1])
        score = 0.7 * in01 + 0.3 * repeats
        stats.append((c, switches, score))

    if not stats:
        raise RuntimeError("Could not detect goal_id column.")

    # First prefer a column that actually switches (needed for lap-time metric).
    positive_switch = [x for x in stats if x[1] > 0]
    if positive_switch:
        # Max switches first, then quality score.
        positive_switch.sort(key=lambda x: (x[1], x[2]), reverse=True)
        return positive_switch[0][0]

    # Fallback: best heuristic score.
    stats.sort(key=lambda x: x[2], reverse=True)
    return stats[0][0]


def load_stats_metrics(path):
    # Read enough cols to support both log formats.
    data = _safe_genfromtxt(path, usecols=tuple(range(8)))
    valid = np.isfinite(data).all(axis=1)
    data = data[valid]
    if data.shape[0] < 2:
        raise RuntimeError(f"Not enough valid rows in {path}")

    goal_col = _pick_goal_column(data)
    t = data[:, 0] + 1e-9 * data[:, 1]
    t_rel = t - t[0]
    exec_ms = data[:, 3]
    goal = np.rint(data[:, goal_col]).astype(int)

    switch_times = []
    switch_types = []
    for i in range(1, len(goal)):
        if goal[i] != goal[i - 1]:
            switch_times.append(t_rel[i])
            switch_types.append((goal[i - 1], goal[i]))
    switch_times = np.array(switch_times)
    switch_durations = np.diff(np.concatenate(([0.0], switch_times))) if switch_times.size else np.array([])

    by_dir = {}
    if switch_times.size:
        for k, tr in enumerate(switch_types):
            by_dir.setdefault(tr, []).append(switch_durations[k])
        for tr in by_dir:
            by_dir[tr] = np.array(by_dir[tr], dtype=float)

    # Full cycle metric: duration between a 0->1 switch and the next 1->0 switch.
    full_010_cycles = []
    pending_start = None
    for k, tr in enumerate(switch_types):
        t_sw = switch_times[k]
        if tr == (0, 1):
            pending_start = t_sw
        elif tr == (1, 0) and pending_start is not None:
            full_010_cycles.append(t_sw - pending_start)
            pending_start = None
    full_010_cycles = np.array(full_010_cycles, dtype=float)

    return {
        "rows": len(t_rel),
        "run_time": float(t_rel[-1]),
        "exec_ms": exec_ms,
        "num_switches": int(switch_times.size),
        "switch_durations": switch_durations,
        "by_dir": by_dir,
        "full_010_cycles": full_010_cycles,
        "goal_col": goal_col,
    }


def load_perf_metrics(path):
    # cols: 0 sec, 1 nsec, 2 task_counter
    data = _safe_genfromtxt(path, usecols=(0, 1, 2))
    valid = np.isfinite(data).all(axis=1)
    data = data[valid]
    if data.shape[0] == 0:
        return {"events": 0, "lap_durations": np.array([])}

    t = data[:, 0] + 1e-9 * data[:, 1]
    t_rel = t - t[0]
    lap = np.diff(t_rel) if len(t_rel) > 1 else np.array([])
    return {"events": len(t_rel), "lap_durations": lap}


def stat_triplet(x):
    if x.size == 0:
        return "n/a"
    return f"{np.mean(x):.3f} / {np.median(x):.3f} / {np.max(x):.3f}"


def print_summary(label, stats_m, perf_m):
    print(f"\n[{label}]")
    print(f"detected goal_id column: {stats_m['goal_col']}")
    print(f"rows: {stats_m['rows']}, run_time_s: {stats_m['run_time']:.3f}")
    print(f"solve_time_ms mean/median/max: {stat_triplet(stats_m['exec_ms'])}")
    print(
        "goal-switch time_s mean/median/max: "
        f"{stat_triplet(stats_m['switch_durations'])} (switches={stats_m['num_switches']})"
    )
    for tr, arr in sorted(stats_m["by_dir"].items()):
        print(f"  switch {tr[0]}->{tr[1]} mean/median/max: {stat_triplet(arr)}")
    print(
        "full 0->1->0 cycle_s mean/median/max: "
        f"{stat_triplet(stats_m['full_010_cycles'])} (cycles={stats_m['full_010_cycles'].size})"
    )
    print(
        f"lap events (solver_perf): {perf_m['events']}, "
        f"lap time_s mean/median/max: {stat_triplet(perf_m['lap_durations'])}"
    )


def main():
    p = argparse.ArgumentParser(description="Compare mpc_high vs mpc_to timing performance.")
    p.add_argument("--a-label", default="mpc_high")
    p.add_argument("--a-stats", required=True)
    p.add_argument("--a-perf", required=True)
    p.add_argument("--b-label", default="mpc_to")
    p.add_argument("--b-stats", required=True)
    p.add_argument("--b-perf", required=True)
    args = p.parse_args()

    a_stats = load_stats_metrics(args.a_stats)
    a_perf = load_perf_metrics(args.a_perf)
    b_stats = load_stats_metrics(args.b_stats)
    b_perf = load_perf_metrics(args.b_perf)

    print_summary(args.a_label, a_stats, a_perf)
    print_summary(args.b_label, b_stats, b_perf)

    # Primary time-optimal criterion: lower mean full 0->1->0 cycle duration.
    a_cyc = a_stats["full_010_cycles"]
    b_cyc = b_stats["full_010_cycles"]
    if a_cyc.size and b_cyc.size:
        a_mean = np.mean(a_cyc)
        b_mean = np.mean(b_cyc)
        if a_mean < b_mean:
            winner = args.a_label
            ratio = b_mean / a_mean
        else:
            winner = args.b_label
            ratio = a_mean / b_mean
        print(
            f"\nWinner on mean full 0->1->0 cycle time: {winner} "
            f"(speedup x{ratio:.3f})"
        )
    else:
        print("\nNot enough complete 0->1->0 cycles in one or both runs to declare a winner.")


if __name__ == "__main__":
    main()
