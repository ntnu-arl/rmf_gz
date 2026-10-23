#!/usr/bin/env python3
"""
run_experiment.py  —  Statistical analysis harness for the reactive controller.

Usage:
    python3 run_experiment.py <controller_script> [options]

Example:
    python3 run_experiment.py ~/ws/src/my_pkg/scripts/my_controller.py \
        --runs 50 --world random3d --headless --output results.json --radius 2.5

Termination conditions (per run):
    SUCCESS   : drone x-displacement > 60 m
    COLLISION : IMU sensor fires massive spike
    TIMEOUT   : < 3 m x-progress within any 15 s sliding window

Statistics reported:
    success_rate, collision_rate, timeout_rate,
    mean_velocity (m/s), max_velocity (m/s),
    mean_max_x (m), mean_duration (s), mean_intervention
"""

import argparse
import json
import math
import os
import sys
import time
import subprocess
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import List, Optional

# Ensure sim_runner is importable from the same directory
sys.path.insert(0, str(Path(__file__).parent))
from sim_runner import ExitCode, RunResult, run_single, _round


# ─────────────────────────────────────────────────────────────────────────────
# Statistics
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class ExperimentStats:
    n_runs:         int   = 0
    n_success:      int   = 0
    n_collision:    int   = 0
    n_timeout:      int   = 0
    n_error:        int   = 0

    success_rate:   float = 0.0
    collision_rate: float = 0.0
    timeout_rate:   float = 0.0
    error_rate:     float = 0.0

    mean_velocity:  float = 0.0   # mean of per-run mean velocities
    max_velocity:   float = 0.0   # global maximum across all runs
    std_velocity:   float = 0.0   # std-dev of per-run mean velocities

    mean_max_x:     float = 0.0   # mean of per-run maximum x-displacement
    std_max_x:      float = 0.0
    mean_duration:  float = 0.0   # mean run duration (s)
    std_duration:   float = 0.0

    mean_intervention: float = 0.0
    std_intervention:  float = 0.0

    # Safety filter diagnostics, over the runs whose filter published them
    # (None if no run did): mean/std of the per-run rates, and the certificate
    # age distribution pooled over all filter ticks (index 0 = newest observation)
    n_filter_runs:          int   = 0
    filter_ticks:           int   = 0
    mean_infeasible_rate:   Optional[float] = None
    std_infeasible_rate:    Optional[float] = None
    mean_solver_fail_rate:  Optional[float] = None
    mean_cbf_violated_rate: Optional[float] = None
    mean_fov_violated_rate: Optional[float] = None
    mean_unsafe_rate:       Optional[float] = None
    std_unsafe_rate:        Optional[float] = None
    cert_age_distribution:  Optional[List[float]] = None

    runs: List[dict] = field(default_factory=list)   # per-run records


def _stddev(values: List[float], mean: float) -> float:
    if len(values) < 2:
        return 0.0
    return math.sqrt(sum((v - mean) ** 2 for v in values) / (len(values) - 1))


def compute_stats(results: List[RunResult]) -> ExperimentStats:
    s = ExperimentStats()
    s.n_runs = len(results)
    if s.n_runs == 0:
        return s

    s.n_success   = sum(1 for r in results if r.exit_code == ExitCode.SUCCESS)
    s.n_collision = sum(1 for r in results if r.exit_code == ExitCode.COLLISION)
    s.n_timeout   = sum(1 for r in results if r.exit_code == ExitCode.TIMEOUT)
    s.n_error     = sum(1 for r in results if r.exit_code == ExitCode.ERROR)

    s.success_rate   = s.n_success   / s.n_runs
    s.collision_rate = s.n_collision / s.n_runs
    s.timeout_rate   = s.n_timeout   / s.n_runs
    s.error_rate     = s.n_error     / s.n_runs

    mean_vels  = [r.mean_velocity for r in results]
    max_xs     = [r.max_x         for r in results]
    durations  = [r.duration_s    for r in results]
    mean_ints  = [r.mean_intervention for r in results]

    s.mean_velocity = sum(mean_vels) / s.n_runs
    s.std_velocity  = _stddev(mean_vels, s.mean_velocity)
    s.max_velocity  = max(r.max_velocity for r in results)

    s.mean_max_x    = sum(max_xs)    / s.n_runs
    s.std_max_x     = _stddev(max_xs, s.mean_max_x)

    s.mean_duration = sum(durations) / s.n_runs
    s.std_duration  = _stddev(durations, s.mean_duration)

    s.mean_intervention = sum(mean_ints) / s.n_runs
    s.std_intervention  = _stddev(mean_ints, s.mean_intervention)

    compute_filter_stats(s, results)

    s.runs = [
        {
            "run":        i + 1,
            "outcome":    r.exit_code.name,
            "max_x_m":    round(r.max_x,          3),
            "mean_v_mps": round(r.mean_velocity,   3),
            "max_v_mps":  round(r.max_velocity,    3),
            "duration_s": round(r.duration_s,      2),
            "mean_int":   round(r.mean_intervention, 4),
            **r.filter_record(),
        }
        for i, r in enumerate(results)
    ]
    return s


def compute_filter_stats(s: ExperimentStats, results: List[RunResult]):
    '''Safety filter diagnostics; only runs whose filter published them count.'''
    fr = [r for r in results if r.filter_ticks > 0]
    s.n_filter_runs = len(fr)
    if not fr:
        return
    s.filter_ticks = sum(r.filter_ticks for r in fr)

    def mean_std(rates):
        m = sum(rates) / len(rates)
        return m, _stddev(rates, m)

    s.mean_infeasible_rate, s.std_infeasible_rate = mean_std([r.infeasible_rate for r in fr])
    s.mean_unsafe_rate,     s.std_unsafe_rate     = mean_std([r.unsafe_rate     for r in fr])
    s.mean_solver_fail_rate,  _ = mean_std([r.n_solver_fail  / r.filter_ticks for r in fr])
    s.mean_cbf_violated_rate, _ = mean_std([r.n_cbf_violated / r.filter_ticks for r in fr])
    s.mean_fov_violated_rate, _ = mean_std([r.n_fov_violated / r.filter_ticks for r in fr])

    counts = [0] * max(len(r.cert_age_counts) for r in fr)
    for r in fr:
        for age, n in enumerate(r.cert_age_counts):
            counts[age] += n
    total = sum(counts)
    s.cert_age_distribution = [n / total for n in counts] if total else None


# ─────────────────────────────────────────────────────────────────────────────
# Pretty printing
# ─────────────────────────────────────────────────────────────────────────────

BOLD  = "\033[1m"
GREEN = "\033[92m"
RED   = "\033[91m"
YEL   = "\033[93m"
CYAN  = "\033[96m"
RST   = "\033[0m"


def _bar(rate: float, width: int = 30, color: str = GREEN) -> str:
    filled = round(rate * width)
    return color + "█" * filled + RST + "░" * (width - filled)


def print_live_summary(run_idx: int, result: RunResult):
    outcome_color = {
        ExitCode.SUCCESS:   GREEN,
        ExitCode.COLLISION: RED,
        ExitCode.TIMEOUT:   YEL,
        ExitCode.ERROR:     RED,
    }.get(result.exit_code, RST)
    print(
        f"  Run {run_idx:>3d}  "
        f"{outcome_color}{result.exit_code.name:<9}{RST}  "
        f"max_x={result.max_x:6.2f} m  "
        f"mean_v={result.mean_velocity:5.2f} m/s  "
        f"max_v={result.max_velocity:5.2f} m/s  "
        f"t={result.duration_s:6.1f} s  "
        f"int={result.mean_intervention:5.2f}"
        + (f"  infeas={result.infeasible_rate*100:5.1f}%  unsafe={result.unsafe_rate*100:5.1f}%"
           if result.filter_ticks else "")
    )


def print_final_report(s: ExperimentStats, elapsed_total: float):
    sep = "─" * 60
    print(f"\n{BOLD}{'═'*60}{RST}")
    print(f"{BOLD}  EXPERIMENT COMPLETE  —  {s.n_runs} runs{RST}")
    print(f"{sep}")

    # Outcome rates
    print(f"\n  {BOLD}Outcome rates{RST}")
    print(f"  {'Success':12s}  {_bar(s.success_rate,   color=GREEN)}  {s.success_rate*100:5.1f}%  ({s.n_success}/{s.n_runs})")
    print(f"  {'Collision':12s}  {_bar(s.collision_rate, color=RED)}   {s.collision_rate*100:5.1f}%  ({s.n_collision}/{s.n_runs})")
    print(f"  {'Timeout':12s}  {_bar(s.timeout_rate,   color=YEL)}   {s.timeout_rate*100:5.1f}%  ({s.n_timeout}/{s.n_runs})")
    if s.n_error:
        print(f"  {'Error':12s}  {_bar(s.error_rate, color=RED)}   {s.error_rate*100:5.1f}%  ({s.n_error}/{s.n_runs})")

    # Velocity
    print(f"\n  {BOLD}Velocity  (m/s){RST}")
    print(f"  Mean (of run means) : {CYAN}{s.mean_velocity:7.3f}{RST}  ± {s.std_velocity:.3f}")
    print(f"  Global maximum      : {CYAN}{s.max_velocity:7.3f}{RST}")

    # Progress
    print(f"\n  {BOLD}x-displacement  (m){RST}")
    print(f"  Mean max-x          : {CYAN}{s.mean_max_x:7.3f}{RST}  ± {s.std_max_x:.3f}")

    # Duration & Intervention
    print(f"\n  {BOLD}Other Metrics{RST}")
    print(f"  Mean Duration (s)   : {CYAN}{s.mean_duration:7.2f}{RST}  ± {s.std_duration:.2f}")
    print(f"  Mean Intervention   : {CYAN}{s.mean_intervention:7.4f}{RST}  ± {s.std_intervention:.4f}")

    if s.n_filter_runs:
        print(f"\n  {BOLD}Safety filter  ({s.n_filter_runs} runs, {s.filter_ticks} ticks, mean of run rates){RST}")
        print(f"  QP infeasible       : {CYAN}{s.mean_infeasible_rate*100:6.2f}%{RST}  ± {s.std_infeasible_rate*100:.2f}%"
              f"   (solver {s.mean_solver_fail_rate*100:.2f}%, CBF {s.mean_cbf_violated_rate*100:.2f}%,"
              f" FOV {s.mean_fov_violated_rate*100:.2f}%)")
        print(f"  Certificate h < 0   : {CYAN}{s.mean_unsafe_rate*100:6.2f}%{RST}  ± {s.std_unsafe_rate*100:.2f}%")
        if s.cert_age_distribution:
            print("  Certificate age     : " + "  ".join(f"{age}:{p*100:.1f}%"
                                                         for age, p in enumerate(s.cert_age_distribution)))

    print(f"\n  Total wall time     : {elapsed_total/60:.1f} min")
    print(f"{'═'*60}\n")


# ─────────────────────────────────────────────────────────────────────────────
# Main experiment loop
# ─────────────────────────────────────────────────────────────────────────────

def run_experiment(
    controller_script: str,
    config_path:       str   = "ablation_config.yaml",
    n_runs:            int   = 20,
    world:             str   = "random3d",
    headless:          bool  = False,
    run_timeout_s:     float = 300.0,
    odom_topic:        str   = "/rmf/odom",
    imu_topic:         str   = "/rmf/imu",
    output_path:       str   = None,
    skip_errors:       bool  = True,
    radius:            float = 3.0,
    sphere_radius:     float = 0.5,
    world_gen_script:  str   = None,
) -> ExperimentStats:

    if not os.path.isfile(controller_script):
        print(f"ERROR: controller script not found: {controller_script}")
        sys.exit(1)

    print(f"\n{BOLD}{'='*60}{RST}")
    print(f"{BOLD}  Reactive Controller — Statistical Experiment (ROS2){RST}")
    print(f"  Controller : {controller_script}")
    print(f"  Config     : {config_path}")
    print(f"  World      : {world}")
    print(f"  Runs       : {n_runs}")
    print(f"  Headless   : {headless}")
    print(f"  Hard TO    : {run_timeout_s:.0f} s")
    print(f"  Radius     : {radius}")
    print(f"  Sphere Radius : {sphere_radius}")
    print(f"{'='*60}\n")

    results: List[RunResult] = []
    t_experiment_start = time.time()

    for i in range(1, n_runs + 1):
        print(f"\n{BOLD}── Run {i}/{n_runs} ──────────────────────────────────────{RST}")

        # ---> WORLD GENERATION: Call the external script before running sim_single
        if world_gen_script and os.path.isfile(world_gen_script):
            print(f"  [World Gen] Generating new world (radius: {radius}, sphere_radius: {sphere_radius})...")
            try:
                subprocess.run(
                    [sys.executable, world_gen_script, "-r", str(radius), "-sr", str(sphere_radius)],
                    check=True,
                    stdout=subprocess.DEVNULL  # Keeps the terminal clean. Remove if you want generator prints.
                )
            except subprocess.CalledProcessError as e:
                print(f"{RED}  ERROR: World generation failed with exit code {e.returncode}.{RST}")
                sys.exit(1)
        else:
            print(f"{YEL}  WARNING: World gen script '{world_gen_script}' not found. Skipping randomization.{RST}")

        # Update run_single call to pass the config path and use overrides
        result = run_single(
            controller_script=controller_script,
            config_path=config_path,
            world_override=world,
            headless_override=headless,
            timeout_override=run_timeout_s,
        )
        print_live_summary(i, result)
        results.append(result)

        # Running partial stats after every run
        partial = compute_stats(results)
        print(
            f"  [{i:>3}/{n_runs}]  "
            f"✓{partial.n_success}  ✗{partial.n_collision}  ⏱{partial.n_timeout}  "
            f"mean_v={partial.mean_velocity:.2f} m/s  max_v={partial.max_velocity:.2f} m/s"
        )

        if skip_errors and result.exit_code == ExitCode.ERROR:
            print("  (error run not counted towards statistics)")
            results.pop()

    elapsed = time.time() - t_experiment_start
    stats   = compute_stats(results)
    print_final_report(stats, elapsed)

    if output_path:
        _save_results(stats, output_path)

    return stats


def _save_results(stats: ExperimentStats, path: str):
    data = {
        "summary": {
            "n_runs":         stats.n_runs,
            "success_rate":   round(stats.success_rate,   4),
            "collision_rate": round(stats.collision_rate, 4),
            "timeout_rate":   round(stats.timeout_rate,   4),
            "error_rate":     round(stats.error_rate,     4),
            "mean_velocity":  round(stats.mean_velocity,  4),
            "std_velocity":   round(stats.std_velocity,   4),
            "max_velocity":   round(stats.max_velocity,   4),
            "mean_max_x":     round(stats.mean_max_x,     4),
            "std_max_x":      round(stats.std_max_x,      4),
            "mean_duration":  round(stats.mean_duration,  4),
            "std_duration":   round(stats.std_duration,   4),
            "mean_intervention": round(stats.mean_intervention, 4),
            "std_intervention":  round(stats.std_intervention,  4),
            "n_filter_runs":          stats.n_filter_runs,
            "filter_ticks":           stats.filter_ticks,
            "mean_infeasible_rate":   _round(stats.mean_infeasible_rate),
            "std_infeasible_rate":    _round(stats.std_infeasible_rate),
            "mean_solver_fail_rate":  _round(stats.mean_solver_fail_rate),
            "mean_cbf_violated_rate": _round(stats.mean_cbf_violated_rate),
            "mean_fov_violated_rate": _round(stats.mean_fov_violated_rate),
            "mean_unsafe_rate":       _round(stats.mean_unsafe_rate),
            "std_unsafe_rate":        _round(stats.std_unsafe_rate),
            "cert_age_distribution":  (None if stats.cert_age_distribution is None
                                       else [round(p, 4) for p in stats.cert_age_distribution]),
        },
        "runs": stats.runs,
    }
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"  Results saved → {path}")


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def _parse():
    p = argparse.ArgumentParser(
        description="Run N simulation trials and compute controller statistics.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("controller",
                   help="Path to the ROS2 controller Python script")
    p.add_argument("--config",         default="ablation_config.yaml",
                   help="Path to the YAML config file")
    p.add_argument("--runs",    "-n",  type=int,   default=20,
                   help="Number of trials to run")
    p.add_argument("--world",          default="random3d",
                   help="Gazebo world name (no .sdf extension)")
    p.add_argument("--headless",       action="store_true",
                   help="Run Gazebo without a GUI (faster)")
    p.add_argument("--timeout",  "-t", type=float, default=300.0,
                   help="Hard per-run timeout in seconds")
    p.add_argument("--odom-topic",     default="/rmf/odom",
                   help="ROS topic for Odometry messages")
    p.add_argument("--imu-topic",      default="/rmf/imu",
                   help="ROS topic for Imu messages (collision detection via accel spikes)")
    p.add_argument("--output",   "-o", default=None,
                   help="Optional JSON file to save results (e.g. results.json)")
    p.add_argument("--keep-errors",    action="store_true",
                   help="Include ERROR runs in statistics (excluded by default)")
    p.add_argument("--radius",   "-r", type=float, default=3.0,
                   help="Poisson disk radius for the world generation script")
    p.add_argument("--sphere_radius",   "-sr", type=float, default=0.5,
                   help="obstacle radius for the world generation script")
    p.add_argument("--world-gen",      default=None,
                   help="Path to the world generation script (default: scripts/rand_poisson2d.py "
                        "if the world name contains '2d', else rand_poisson3d.py)")
    return p.parse_args()


if __name__ == "__main__":
    args = _parse()
    if args.world_gen is None:
        gen_name = "rand_poisson2d.py" if "2d" in args.world else "rand_poisson3d.py"
        args.world_gen = str(Path(__file__).parent / "scripts" / gen_name)

    output_file = args.output
    if output_file:
        p = Path(output_file)
        # Ensure the directory generated by the meta-supervisor (or standalone run) exists
        p.parent.mkdir(parents=True, exist_ok=True)
        # Only append the Poisson density to the filename
        output_file = str(p.parent / f"{p.stem}_r{args.radius}{p.suffix}")

    run_experiment(
        controller_script=args.controller,
        config_path=args.config,
        n_runs=args.runs,
        world=args.world,
        headless=args.headless,
        run_timeout_s=args.timeout,
        odom_topic=args.odom_topic,
        imu_topic=args.imu_topic,
        output_path=output_file,
        skip_errors=not args.keep_errors,
        radius=args.radius,
        sphere_radius=args.sphere_radius,
        world_gen_script=args.world_gen,
    )