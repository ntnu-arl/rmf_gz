#!/usr/bin/env python3
"""
Grid search of the safety-filter parameters in the 2D closed-loop simulation
(sim2d.py), run in parallel over CPU worker processes.

Every parameter combination of a method runs on the same world pool (paired
comparison). Runs are appended to <out>/<method>/runs.jsonl as they finish, so
an interrupted search resumes where it stopped; summary.csv / summary.json
rank the combinations by J = S - 3 C (S, C: success and collision rate; with
S + C + T = 1 a collision weighs as four timeouts), then collision rate, then
the mean time of the successful runs. make_tables.py turns the summaries into
LaTeX tables, plot_contours.py into contour plots.

    python run_grid_search.py                          # all methods in grid_config.yaml
    python run_grid_search.py --methods daf --workers 16
    python run_grid_search.py --summarize-only
"""
import argparse
import csv
import itertools
import json
import math
import multiprocessing as mp
import os
import time
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
RESULTS_DIR = HERE / "results"

# selection criterion J = S - COLLISION_WEIGHT * C
COLLISION_WEIGHT = 3.0


def criterion(success_rate, collision_rate):
    return success_rate - COLLISION_WEIGHT * collision_rate


def default_out(world_cfg):
    '''results/poisson_<lo>_<hi>: one folder per world configuration.'''
    lo, hi = world_cfg['poisson_radius']
    return RESULTS_DIR / f"poisson_{lo:g}_{hi:g}"

RESULT_FIELDS = ['outcome', 'duration_s', 'max_x', 'mean_speed', 'max_speed',
                 'min_clearance', 'mean_intervention', 'n_filter_warnings', 'wall_s']


# ======================================================================
# workers
# ======================================================================
_W = {}


def _init_worker(device, world_cfg):
    # one thread per process: the workers already fill the cores (set before
    # numpy/torch load their thread pools)
    for var in ('OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS'):
        os.environ[var] = '1'
    import torch
    torch.set_num_threads(1)
    torch.set_num_interop_threads(1)
    import warnings
    warnings.filterwarnings('ignore', message='The given NumPy array is not writable')
    import sim2d   # sibling module (spawned workers inherit sys.path)
    _W.update(sim2d=sim2d, device=device, world_cfg=world_cfg,
              base=sim2d.load_base_config(), classes={}, worlds={})


def _world(seed):
    sim2d, wc, base = _W['sim2d'], _W['world_cfg'], _W['base']
    if seed not in _W['worlds']:
        _W['worlds'][seed] = sim2d.generate_world(
            seed, wc['poisson_radius'], wc['pillar_radius'], base['DRONE_RADIUS'],
            base.get('termination', {}).get('success_x_dist_m', 30.0))
    return _W['worlds'][seed]


def _run_task(task):
    method, node_spec, params, seed = task
    sim2d = _W['sim2d']
    if node_spec not in _W['classes']:
        _W['classes'][node_spec] = sim2d.load_node_class(node_spec)
    world = _world(seed)
    row = dict(method=method, params=params, world_seed=seed,
               poisson_radius=round(world.poisson_radius, 4))
    try:
        result, _ = sim2d.run_episode(_W['classes'][node_spec], params, world, base_cfg=_W['base'],
                                      device=_W['device'], noise_seed=seed)
        row.update({k: getattr(result, k) for k in RESULT_FIELDS})
    except Exception as e:   # one failing episode must not stop the search
        row.update({k: None for k in RESULT_FIELDS}, outcome='ERROR', error=repr(e))
    return row


# ======================================================================
# bookkeeping
# ======================================================================
def param_key(params):
    return json.dumps(params, sort_keys=True)


def subgrids(method_cfg):
    '''`grid` is one grid {param: values} or a list of grids over the same parameters.'''
    grid = method_cfg.get('grid', {})
    return grid if isinstance(grid, list) else [grid]


def param_names(method_cfg):
    return list(subgrids(method_cfg)[0])


def combos(method_cfg):
    '''Points of the (union of the) grids, with the fixed parameters, that satisfy every constraint.'''
    fixed = method_cfg.get('fixed', {}) or {}
    constraints = method_cfg.get('constraints', []) or []
    names = param_names(method_cfg)
    seen = set()
    for grid in subgrids(method_cfg):
        for values in itertools.product(*(grid[n] for n in names)):
            params = dict(fixed) | dict(zip(names, values))
            if param_key(params) not in seen and all(eval(c, {'__builtins__': {}}, dict(params))
                                                     for c in constraints):
                seen.add(param_key(params))
                yield params


def load_runs(path, seeds=None):
    '''Runs logged so far (restricted to the world seeds given); a line cut off by a kill is skipped.'''
    runs = []
    if path.exists():
        with open(path) as f:
            for line in f:
                try:
                    runs.append(json.loads(line))
                except json.JSONDecodeError:
                    continue
    if seeds is not None:
        seeds = set(seeds)
        runs = [r for r in runs if r['world_seed'] in seeds]
    return runs


def wilson(k, n, z=1.96):
    if n == 0:
        return (0.0, 0.0)
    p = k / n
    c = (p + z * z / (2 * n)) / (1 + z * z / n)
    w = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return (max(0.0, c - w), min(1.0, c + w))


def summarize(method, method_cfg, out_dir, seeds, n_show=10):
    runs = load_runs(out_dir / 'runs.jsonl', seeds)
    if not runs:
        print(f"  {method}: no runs yet")
        return []
    names = param_names(method_cfg)
    baseline = param_key(dict(method_cfg.get('fixed', {}) or {}) | dict(method_cfg.get('baseline', {}) or {}))

    by_cfg = {}
    for r in runs:
        by_cfg.setdefault(param_key(r['params']), []).append(r)

    rows = []
    for key, rs in by_cfg.items():
        n = len(rs)
        count = lambda o: sum(r['outcome'] == o for r in rs)
        succ = [r for r in rs if r['outcome'] == 'SUCCESS']
        ok = [r for r in rs if r['outcome'] not in ('ERROR', 'NAN')]
        lo, hi = wilson(len(succ), n)
        rows.append(dict(
            json.loads(key),
            n=n, success_rate=round(len(succ) / n, 4),
            success_ci95=f"{lo:.2f}-{hi:.2f}",
            collision_rate=round(count('COLLISION') / n, 4),
            timeout_rate=round(count('TIMEOUT') / n, 4),
            crit=round(criterion(len(succ) / n, count('COLLISION') / n), 4),
            error_rate=round((count('ERROR') + count('NAN')) / n, 4),
            mean_success_time=round(sum(r['duration_s'] for r in succ) / len(succ), 3) if succ else None,
            mean_speed=round(sum(r['mean_speed'] for r in ok) / len(ok), 4) if ok else None,
            mean_min_clearance=round(sum(r['min_clearance'] for r in ok) / len(ok), 4) if ok else None,
            mean_intervention=round(sum(r['mean_intervention'] for r in ok) / len(ok), 4) if ok else None,
            is_baseline=(key == baseline)))

    rows.sort(key=lambda r: (-r['crit'], r['collision_rate'],
                             r['mean_success_time'] if r['mean_success_time'] is not None else 1e9))
    for i, r in enumerate(rows):
        r['rank'] = i + 1

    cols = ['rank'] + names + ['n', 'crit', 'success_rate', 'success_ci95', 'collision_rate', 'timeout_rate',
                               'error_rate', 'mean_success_time', 'mean_speed', 'mean_min_clearance',
                               'mean_intervention', 'is_baseline']
    with open(out_dir / 'summary.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction='ignore')
        w.writeheader()
        w.writerows(rows)
    (out_dir / 'summary.json').write_text(json.dumps(rows, indent=1))

    def fmt(r):
        ps = '  '.join(f"{n}={r[n]:<7g}" for n in names)
        t = f"{r['mean_success_time']:6.1f}s" if r['mean_success_time'] is not None else '     - '
        return (f"  {r['rank']:>3}  {ps}  n={r['n']:<3} J {r['crit']*100:+6.1f}  "
                f"succ {r['success_rate']*100:5.1f}% "
                f"[{r['success_ci95']}]  coll {r['collision_rate']*100:5.1f}%  "
                f"t/o {r['timeout_rate']*100:5.1f}%  t_succ {t}  v {r['mean_speed'] or 0:4.2f}"
                f"{'  <- baseline' if r['is_baseline'] else ''}")

    print(f"\n  {method.upper()}: {len(rows)} settings, {len(runs)} runs  ->  {out_dir}/summary.csv")
    for r in rows[:n_show]:
        print(fmt(r))
    base_rows = [r for r in rows[n_show:] if r['is_baseline']]
    if base_rows:
        print("  ...")
        print(fmt(base_rows[0]))
    return rows


# ======================================================================
# driver
# ======================================================================
def main():
    ap = argparse.ArgumentParser(description="2D closed-loop grid search of safety-filter parameters")
    ap.add_argument('--config', default=str(HERE / 'grid_config.yaml'))
    ap.add_argument('--methods', nargs='*', default=None, help="subset of the methods in the config")
    ap.add_argument('--worlds', type=int, default=None, help="override worlds.n")
    # the episodes are memory-bandwidth bound: more workers than physical cores does not help
    ap.add_argument('--workers', type=int, default=max(1, (os.cpu_count() or 2) // 2))
    ap.add_argument('--device', default='cpu')
    ap.add_argument('--out', default=None, help="default: results/poisson_<lo>_<hi>")
    ap.add_argument('--fresh', action='store_true', help="discard existing runs of the selected methods")
    ap.add_argument('--summarize-only', action='store_true')
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.config))
    world_cfg = cfg['worlds']
    if args.worlds is not None:
        world_cfg['n'] = args.worlds
    seeds = [world_cfg['seed'] + i for i in range(world_cfg['n'])]
    methods = args.methods or list(cfg['methods'])
    out_root = Path(args.out) if args.out else default_out(world_cfg)

    if not args.summarize_only:
        tasks, files = [], {}
        for m in methods:
            mcfg = cfg['methods'][m]
            d = out_root / m
            d.mkdir(parents=True, exist_ok=True)
            if args.fresh and (d / 'runs.jsonl').exists():
                (d / 'runs.jsonl').unlink()
            # runs are matched by (setting, world seed): resuming is only valid
            # if the seeds still generate the same worlds
            prev = d / 'grid_config.yaml'
            if (d / 'runs.jsonl').exists() and prev.exists():
                prev_worlds = {k: v for k, v in yaml.safe_load(prev.read_text())['worlds'].items() if k != 'n'}
                if prev_worlds != {k: v for k, v in world_cfg.items() if k != 'n'}:
                    raise SystemExit(f"{d} holds runs of a different world configuration "
                                     f"({prev_worlds}); use another --out or --fresh")
            prev.write_text(yaml.safe_dump(dict(worlds=world_cfg, method=mcfg), sort_keys=False))
            done = {(param_key(r['params']), r['world_seed']) for r in load_runs(d / 'runs.jsonl')}
            todo = [(m, mcfg['node'], p, s) for p in combos(mcfg) for s in seeds
                    if (param_key(p), s) not in done]
            n_cfg = sum(1 for _ in combos(mcfg))
            print(f"  {m}: {n_cfg} settings x {len(seeds)} worlds, {len(todo)} runs to do")
            tasks += todo
            files[m] = open(d / 'runs.jsonl', 'a')

        # interleave the methods so partial results cover all of them
        tasks.sort(key=lambda t: (t[3], t[0]))
        print(f"  {len(tasks)} runs on {args.workers} workers ({args.device})\n")

        t0, n_done = time.time(), 0
        counts = {}
        ctx = mp.get_context('spawn')
        try:
            with ctx.Pool(args.workers, initializer=_init_worker, initargs=(args.device, world_cfg)) as pool:
                for row in pool.imap_unordered(_run_task, tasks, chunksize=1):
                    files[row['method']].write(json.dumps(row) + '\n')
                    files[row['method']].flush()
                    n_done += 1
                    counts[row['outcome']] = counts.get(row['outcome'], 0) + 1
                    if n_done % 100 == 0 or n_done == len(tasks):
                        el = time.time() - t0
                        eta = el / n_done * (len(tasks) - n_done)
                        print(f"  {n_done:>6}/{len(tasks)}  {el/60:5.1f} min elapsed, ETA {eta/60:5.1f} min  "
                              f"{dict(sorted(counts.items()))}", flush=True)
        finally:
            for f in files.values():
                f.close()

    for m in methods:
        summarize(m, cfg['methods'][m], out_root / m, seeds)
    print()


if __name__ == '__main__':
    main()
