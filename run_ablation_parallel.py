#!/usr/bin/env python3
"""
Parallel ablation runner — N isolated Gazebo environments at once.

The sequential path (run_ablation_1d.py -> run_experiment.py -> sim_runner.py) is
untouched; this is an alternative driver that reuses sim_runner.run_single() and
run_experiment's result format, so the JSONs it writes are drop-in compatible with
the existing analysis.

Isolation, one slot at a time:
  * ROS_DOMAIN_ID   — separate DDS domain per slot. This is not a namespace trick:
                      the domain selects different discovery/data ports and a
                      separate shared-memory segment, so /clock and /rmf/odom in
                      slot 0 are invisible to slot 1. Linux-safe range is 0-101.
                      It is exported in the PARENT of the worker, because
                      sim_runner hosts its RunMonitor node in-process and reads
                      the domain at rclpy.init() time.
  * GZ_PARTITION    — Gazebo transport does not go through DDS at all; without a
                      partition two gz servers still see each other's topics.
  * SIM_RUNNER_NO_GLOBAL_SWEEP — disables sim_runner's name-based `pkill` fallback,
                      which would otherwise kill the other slots' processes. The
                      per-run process-group teardown (killpg) handles cleanup and
                      is scoped correctly.

World handling: rand_poisson3d.py writes to fixed paths (resources/worlds/random3d.sdf
and environment_assets/random_spheres.dae), so slots cannot generate worlds
concurrently. Instead a pool of distinct worlds is generated ONCE, up front, and
every setting replays the same pool — run k of setting A and run k of setting B see
the identical environment. That is a paired comparison, which removes
world-to-world variance from the setting-to-setting differences.

Note the worlds are written into the INSTALL share dir, because the launch file
sets GZ_SIM_RESOURCE_PATH to $(find-pkg-share rmf_gz)/resources/ — worlds written
only into src/ are never loaded.

    python3 run_ablation_parallel.py \
        --configs src/rmf_gz/cfg_p0.5_8.0.yaml src/rmf_gz/cfg_p2.0_2.0.yaml \
        --density 2.0 --runs 10 --slots 8 --outdir results_parallel
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
WS = HERE.parent.parent                      # /home/arl/lmf_ws
SRC_RES = HERE / "resources"
INSTALL_RES = WS / "install/rmf_gz/share/rmf_gz/resources"
WORLD_GEN = HERE / "scripts/rand_poisson3d.py"
DEFAULT_CONTROLLER = "/home/arl/nCBF3D/nCBF3D/G_ROS_interface/ROS2/sim_node_lidar_regression.py"


# ──────────────────────────────────────────────────────────────────────────────
# World pool
# ──────────────────────────────────────────────────────────────────────────────
def build_world_pool(n, density, sphere_radius, prefix="ablpar"):
    """
    Generate n distinct worlds sequentially and freeze each under its own name.
    Returns the list of world names to pass as `world:=` / world_override.
    """
    worlds_src = SRC_RES / "worlds" / "random3d.sdf"
    mesh_src = SRC_RES / "environment_assets" / "random_spheres.dae"
    worlds_dst_dir = INSTALL_RES / "worlds"
    mesh_dst_dir = SRC_RES / "environment_assets"
    worlds_dst_dir.mkdir(parents=True, exist_ok=True)

    names = []
    for k in range(n):
        name = f"{prefix}_{k:03d}"
        print(f"  [world {k+1}/{n}] generating {name} (density {density}, r {sphere_radius})", flush=True)
        subprocess.run([sys.executable, str(WORLD_GEN), "-r", str(density), "-sr", str(sphere_radius)],
                       check=True, stdout=subprocess.DEVNULL)
        # freeze the mesh under a unique name and repoint the SDF at it; the SDF
        # references the mesh by absolute path, so the copy can live anywhere
        mesh_dst = mesh_dst_dir / f"random_spheres_{name}.dae"
        shutil.copy2(mesh_src, mesh_dst)
        sdf = worlds_src.read_text()
        sdf = sdf.replace(str(mesh_src.resolve()), str(mesh_dst.resolve()))
        (worlds_dst_dir / f"{name}.sdf").write_text(sdf)
        names.append(name)
    return names


def clear_world_pool(prefix="ablpar"):
    for p in (INSTALL_RES / "worlds").glob(f"{prefix}_*.sdf"):
        p.unlink()
    for p in (SRC_RES / "environment_assets").glob(f"random_spheres_{prefix}_*.dae"):
        p.unlink()


# ──────────────────────────────────────────────────────────────────────────────
# Worker: runs its assigned items inside one isolated ROS domain
# ──────────────────────────────────────────────────────────────────────────────
def run_worker(work_path, out_path):
    sys.path.insert(0, str(HERE))
    from sim_runner import run_single           # imported AFTER the env is set

    work = json.loads(Path(work_path).read_text())
    out = []
    for item in work["items"]:
        t0 = time.time()
        try:
            r = run_single(
                controller_script=work["controller"],
                config_path=item["config"],
                world_override=item["world"],
                headless_override=True,
                timeout_override=work["timeout"],
            )
            out.append(dict(config=item["config"], world=item["world"],
                            outcome=r.exit_code.name, max_x_m=round(r.max_x, 3),
                            mean_v_mps=round(r.mean_velocity, 3),
                            max_v_mps=round(r.max_velocity, 3),
                            duration_s=round(r.duration_s, 2),
                            mean_int=round(r.mean_intervention, 4),
                            **r.filter_record(),
                            wall_s=round(time.time() - t0, 1)))
        except Exception as e:                   # one bad run must not kill the slot
            out.append(dict(config=item["config"], world=item["world"],
                            outcome="ERROR", error=repr(e),
                            max_x_m=0.0, mean_v_mps=0.0, max_v_mps=0.0,
                            duration_s=0.0, mean_int=0.0,
                            wall_s=round(time.time() - t0, 1)))
        Path(out_path).write_text(json.dumps(out, indent=1))   # flush as we go
    Path(out_path).write_text(json.dumps(out, indent=1))


def _pct(rate):
    return "-" if rate is None else f"{rate*100:.1f}%"


# ──────────────────────────────────────────────────────────────────────────────
# Driver
# ──────────────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description="Run ablation settings across N parallel Gazebo environments")
    ap.add_argument("--configs", nargs="+", help="per-setting YAML configs (required in driver mode)")
    ap.add_argument("--density", type=float, default=2.0)
    ap.add_argument("--sphere-radius", type=float, default=0.5)
    ap.add_argument("--runs", type=int, default=10, help="runs per setting")
    ap.add_argument("--slots", type=int, default=4, help="parallel environments")
    ap.add_argument("--outdir", default="results_parallel")
    ap.add_argument("--controller", default=DEFAULT_CONTROLLER)
    ap.add_argument("--timeout", type=float, default=100.0)
    ap.add_argument("--domain-base", type=int, default=20, help="first ROS_DOMAIN_ID (0-101 is the safe range)")
    ap.add_argument("--keep-worlds", action="store_true")
    # worker mode
    ap.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    ap.add_argument("--work", help=argparse.SUPPRESS)
    ap.add_argument("--out", help=argparse.SUPPRESS)
    args = ap.parse_args()

    if args.worker:
        run_worker(args.work, args.out)
        return

    if not args.configs:
        ap.error("--configs is required")

    if args.domain_base + args.slots > 101:
        sys.exit(f"ROS_DOMAIN_ID {args.domain_base}..{args.domain_base+args.slots-1} exceeds the Linux-safe range (0-101)")

    configs = [str(Path(c).resolve()) for c in args.configs]
    outdir = Path(args.outdir).resolve(); outdir.mkdir(parents=True, exist_ok=True)
    scratch = outdir / "_slots"; scratch.mkdir(exist_ok=True)

    print(f"\n{'='*70}\n  PARALLEL ABLATION — {len(configs)} settings x {args.runs} runs "
          f"across {args.slots} environments\n{'='*70}")
    print(f"  density {args.density}, sphere radius {args.sphere_radius}, "
          f"ROS_DOMAIN_ID {args.domain_base}..{args.domain_base+args.slots-1}\n")

    print(f"  Building a shared pool of {args.runs} worlds (all settings replay the same pool)")
    worlds = build_world_pool(args.runs, args.density, args.sphere_radius)

    # round-robin so every slot gets a mix of settings; if a slot dies, the loss
    # is spread over settings rather than wiping one out
    items = [dict(config=c, world=w) for c in configs for w in worlds]
    slots = [items[i::args.slots] for i in range(args.slots)]

    procs = []
    t0 = time.time()
    for i, chunk in enumerate(slots):
        if not chunk:
            continue
        wp, op = scratch / f"work{i}.json", scratch / f"out{i}.json"
        wp.write_text(json.dumps(dict(controller=args.controller, timeout=args.timeout, items=chunk)))
        env = dict(os.environ)
        env["ROS_DOMAIN_ID"] = str(args.domain_base + i)
        env["GZ_PARTITION"] = f"ablpar{i}"
        env["SIM_RUNNER_NO_GLOBAL_SWEEP"] = "1"
        log = open(scratch / f"slot{i}.log", "w")
        procs.append((i, subprocess.Popen(
            [sys.executable, "-u", str(Path(__file__).resolve()), "--worker",
             "--work", str(wp), "--out", str(op)],
            cwd=str(WS), env=env, stdout=log, stderr=subprocess.STDOUT), op, log))
        print(f"  slot {i}: {len(chunk):3d} runs, ROS_DOMAIN_ID={env['ROS_DOMAIN_ID']}, "
              f"GZ_PARTITION={env['GZ_PARTITION']}")

    print(f"\n  waiting for {len(procs)} slots...")
    try:
        for i, p, _, _ in procs:
            p.wait()
            print(f"  slot {i} finished ({(time.time()-t0)/60:.1f} min elapsed)")
    except KeyboardInterrupt:
        print("\n  interrupted — terminating slots")
        for _, p, _, _ in procs:
            p.terminate()                    # SIGTERM; sim_runner now cleans up on it
        for _, p, _, _ in procs:
            p.wait()
    finally:
        for _, _, _, log in procs:
            log.close()

    # ── merge, in run_experiment's own format so existing analysis just works
    sys.path.insert(0, str(HERE))
    from run_experiment import compute_stats, _save_results
    from sim_runner import RunResult, ExitCode

    allruns = []
    for _, _, op, _ in procs:
        if op.exists():
            allruns += json.loads(op.read_text())

    print(f"\n{'='*70}\n  RESULTS — {len(allruns)} runs in {(time.time()-t0)/60:.1f} min\n{'='*70}")
    print(f"  {'setting':>28} | {'n':>3} {'ok':>3} {'coll':>4} {'t/o':>4} {'err':>4} | "
          f"{'mean max-x':>10} {'mean v':>7} {'infeas':>7} {'h<0':>6}")
    print("  " + "-" * 94)
    for c in configs:
        rs = [r for r in allruns if r["config"] == c]
        res = []
        for r in rs:
            rr = RunResult()
            rr.exit_code = getattr(ExitCode, r["outcome"], ExitCode.ERROR)
            rr.max_x = r["max_x_m"]; rr.velocities = [r["mean_v_mps"], r["max_v_mps"]]
            rr.interventions = [r["mean_int"]]; rr.duration_s = r["duration_s"]
            rr.success = r["outcome"] == "SUCCESS"; rr.collision = r["outcome"] == "COLLISION"
            rr.timeout = r["outcome"] == "TIMEOUT"
            rr.load_filter_record(r)
            res.append(rr)
        stats = compute_stats(res)
        stats.runs = rs
        tag = Path(c).stem
        d = outdir / tag; d.mkdir(exist_ok=True)
        _save_results(stats, str(d / f"results_r{args.density}.json"))
        n = stats.n_runs
        print(f"  {tag:>28} | {n:>3} {round(stats.success_rate*n):>3} "
              f"{round(stats.collision_rate*n):>4} {round(stats.timeout_rate*n):>4} "
              f"{round(stats.error_rate*n):>4} | {stats.mean_max_x:>10.2f} {stats.mean_velocity:>7.2f} "
              f"{_pct(stats.mean_infeasible_rate):>7} {_pct(stats.mean_unsafe_rate):>6}")

    if not args.keep_worlds:
        clear_world_pool()
    print()


if __name__ == "__main__":
    main()
