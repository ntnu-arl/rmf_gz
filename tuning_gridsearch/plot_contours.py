#!/usr/bin/env python3
"""
Contour plots of a grid search: the success and collision rate (--metric rates,
one row each) or the selection criterion J = S - 3 C of run_grid_search.py
(--metric crit). The last two grid parameters span each panel (x = last, y = second to
last); with a third parameter, it facets the panels (first grid parameter, one
column per value). With several sub-grids, the axes are their union and the
contours cover only grid cells with sampled corners (unsampled regions stay
blank). Grid points are dotted, settings excluded by the grid constraints
hatched, and the `official` and `selected` settings of grid_config.yaml marked.
Parameters spanning more than a factor of 8 are drawn on a log axis.

    python plot_contours.py                                  # DAF rates of results/poisson_1.5_4
    python plot_contours.py --method ccbf --metric crit      # -> figures/ccbf_crit.pdf/.png
"""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, BoundaryNorm, TwoSlopeNorm
from matplotlib.tri import Triangulation

from make_tables import SYMBOLS
from run_grid_search import subgrids, param_names, criterion, COLLISION_WEIGHT

HERE = Path(__file__).resolve().parent

# one-hue sequential ramps, light (near zero) -> dark: blue for success, the next
# categorical hue (orange) for collisions
BLUE   = ['#cde2fb', '#9ec5f4', '#6da7ec', '#3987e5', '#256abf', '#184f95', '#0d366b']
ORANGE = ['#fde4d8', '#f7b99c', '#f18e64', '#eb6834', '#c44e1f', '#963a14', '#68280c']
# diverging blue <-> red with a neutral gray midpoint, equal steps per arm
RED_TO_BLUE = ['#82201f', '#b8302f', '#e34948', '#e8706a', '#f2a29c',
               '#f0efec',
               '#9ec5f4', '#6da7ec', '#3987e5', '#256abf', '#0d366b']
INK, INK_2, MUTED, SURFACE = '#0b0b0b', '#52514e', '#898781', '#fcfcfb'

# J spans [-100 w, 100]: the diverging ramp keeps its neutral midpoint at J = 0
J_MIN = -100 * COLLISION_WEIGHT
METRICS = {
    'rates': [('success_rate', 'Success rate [%]', BLUE, np.linspace(0, 100, 11), None),
              ('collision_rate', 'Collision rate [%]', ORANGE, np.linspace(0, 100, 11), None)],
    'crit':  [('crit', f'$J = S - {COLLISION_WEIGHT:g}C$ [pp]', RED_TO_BLUE, np.arange(J_MIN, 101, 25),
               TwoSlopeNorm(vmin=J_MIN, vcenter=0, vmax=100))],
}
MARK_STYLE = {'official': dict(marker='o', s=40), 'selected': dict(marker='s', s=40)}
LENGTHS = ('DAF_EPS', 'DAF_EPS1', 'DAF_EPS2')   # parameters in metres


def thinned_labels(values, positions, min_gap=0.15):
    '''Tick labels at every grid value, blank where closer than min_gap of the axis to the last label.'''
    labels, labelled = [''] * len(values), []
    span = positions.max() - positions.min()
    order = np.argsort(positions)
    for k in order:
        if not labelled or positions[k] - positions[labelled[-1]] >= min_gap * span - 1e-9:
            labels[k] = f"{values[k]:g}"; labelled.append(k)
    # always label the far end, dropping the label before it if the two would collide
    end = order[-1]
    if labelled[-1] != end:
        if positions[end] - positions[labelled[-1]] < min_gap * span - 1e-9 and len(labelled) > 1:
            labels[labelled[-1]] = ''
        labels[end] = f"{values[end]:g}"
    return labels


def axis_transform(values):
    '''log2 for positive values spanning more than a factor of 8, else identity.'''
    if np.all(values > 0) and values.max() / values.min() > 8:
        return np.log2
    return lambda v: np.asarray(v, float)


def cell_triangles(px, py, xs, ys):
    '''
    Triangles over the cells of the xs x ys grid whose corners were sampled:
    two per cell with four sampled corners, one per cell with three (at a
    constraint or sub-grid boundary). Unsampled regions get none.
    '''
    index = {(x, y): k for k, (x, y) in enumerate(zip(px, py))}
    triangles = []
    for a in range(len(xs) - 1):
        for b in range(len(ys) - 1):
            c = [index.get((xs[a], ys[b])), index.get((xs[a + 1], ys[b])),
                 index.get((xs[a + 1], ys[b + 1])), index.get((xs[a], ys[b + 1]))]
            have = [k for k in c if k is not None]
            if len(have) == 4:
                triangles += [[c[0], c[1], c[2]], [c[0], c[2], c[3]]]
            elif len(have) == 3:
                triangles.append(have)
    return triangles


def allowed(constraints, namespace):
    return all(eval(c, {'__builtins__': {}}, namespace) for c in constraints)


def main():
    ap = argparse.ArgumentParser(description="Contour plots of a grid search")
    ap.add_argument('--results', default=str(HERE / 'results' / 'poisson_1.5_4'))
    ap.add_argument('--config', default=str(HERE / 'grid_config.yaml'),
                    help="for the official/selected settings")
    ap.add_argument('--method', default='daf')
    ap.add_argument('--metric', default='rates', choices=list(METRICS))
    ap.add_argument('--out', default=None, help="output path without extension "
                                                "(default: figures/<method>_<contours|crit>)")
    args = ap.parse_args()

    d = Path(args.results) / args.method
    rows = json.loads((d / 'summary.json').read_text())
    for r in rows:
        r['crit'] = criterion(r['success_rate'], r['collision_rate'])
    run_cfg = yaml.safe_load(open(d / 'grid_config.yaml'))
    # union of the sub-grids, values ascending
    grid = {n: sorted({float(v) for g in subgrids(run_cfg['method']) for v in g[n]})
            for n in param_names(run_cfg['method'])}
    fixed = run_cfg['method'].get('fixed', {}) or {}
    constraints = run_cfg['method'].get('constraints', []) or []
    cur_cfg = yaml.safe_load(open(args.config))['methods'].get(args.method, {})
    marks = [(key, cur_cfg[key]) for key in ('official', 'selected') if cur_cfg.get(key)]

    *facet, yname, xname = list(grid)
    facet = facet[0] if facet else None
    facet_vals = [float(v) for v in grid[facet]] if facet else [None]
    xs, ys = np.array(grid[xname], float), np.array(grid[yname], float)
    tx, ty = axis_transform(xs), axis_transform(ys)

    plt.rcParams.update({'font.size': 8, 'axes.labelsize': 8, 'axes.titlesize': 8,
                         'xtick.labelsize': 7, 'ytick.labelsize': 7,
                         'axes.edgecolor': MUTED, 'xtick.color': INK_2, 'ytick.color': INK_2,
                         'axes.labelcolor': INK_2, 'text.color': INK,
                         'hatch.color': '#c3c2b7', 'hatch.linewidth': 0.6})
    metrics = METRICS[args.metric]
    nrow, ncol = len(metrics), len(facet_vals)
    fig, axes = plt.subplots(nrow, ncol, figsize=(max(1.45 * ncol, 2.4) + 1.0, 1.8 * nrow + 0.4),
                             sharex=True, sharey=True, squeeze=False, constrained_layout=True)
    fig.set_facecolor(SURFACE)

    # settings the constraints exclude, on a fine grid in plot coordinates
    gx, gy = np.meshgrid(np.linspace(xs.min(), xs.max(), 200), np.linspace(ys.min(), ys.max(), 200))

    for i, (metric, label, ramp, levels, norm) in enumerate(metrics):
        cmap = LinearSegmentedColormap.from_list(metric, ramp)
        norm = norm if norm is not None else BoundaryNorm(levels, cmap.N)
        for j, fv in enumerate(facet_vals):
            ax = axes[i, j]
            ax.set_facecolor(SURFACE)
            base_ns = dict(fixed) | ({facet: fv} if facet else {})
            pts = [r for r in rows if facet is None or float(r[facet]) == fv]
            px = np.array([float(r[xname]) for r in pts])
            py = np.array([float(r[yname]) for r in pts])
            pz = np.array([100 * r[metric] for r in pts])

            triangles = cell_triangles(px, py, xs, ys)
            if triangles:
                tri = Triangulation(tx(px), ty(py), triangles=triangles)
                cs = ax.tricontourf(tri, pz, levels=levels, cmap=cmap, norm=norm)
                ax.tricontour(tri, pz, levels=levels, colors=SURFACE, linewidths=0.6, linestyles='solid')

            excluded = ~np.vectorize(lambda x, y: allowed(constraints, base_ns | {xname: x, yname: y}))(gx, gy)
            if excluded.any():
                ax.contourf(tx(gx), ty(gy), excluded.astype(float), levels=[0.5, 1.5], colors='none',
                            hatches=['////'])
            ax.scatter(tx(px), ty(py), s=5, c=INK_2, linewidths=0, zorder=3, clip_on=False)

            x_mid = (tx(xs).min() + tx(xs).max()) / 2
            for name, params in marks:
                if facet is None or float(params[facet]) == fv:
                    mx, my = tx(params[xname]), ty(params[yname])
                    ax.scatter(mx, my, facecolors='none', edgecolors=INK, linewidths=1.2, zorder=4,
                               clip_on=False, **MARK_STYLE[name])
                    # label towards the panel centre so it stays inside the panel
                    right = mx <= x_mid
                    ann = ax.annotate(name, (mx, my), xytext=(6 if right else -6, -10),
                                      textcoords='offset points', ha='left' if right else 'right',
                                      fontsize=7, color=INK)
                    ann.set_in_layout(False)

            if i == 0 and facet:
                ax.set_title(f"${SYMBOLS.get(facet, facet).strip('$')} = {fv:g}$")
            if i == nrow - 1:
                ax.set_xlabel(SYMBOLS.get(xname, xname) + (' [m]' if xname in LENGTHS else ''))
            if j == 0:
                ax.set_ylabel(SYMBOLS.get(yname, yname) + (' [m]' if yname in LENGTHS else ''))
            ax.set_xticks(tx(xs), thinned_labels(xs, tx(xs)))
            ax.set_yticks(ty(ys), thinned_labels(ys, ty(ys)))
            ax.tick_params(length=2)
            for side in ('top', 'right'):
                ax.spines[side].set_visible(False)
        ticks = levels[::2] if len(levels) <= 11 else levels[levels % 100 == 0]
        cb = fig.colorbar(cs, ax=list(axes[i]), shrink=0.95, pad=0.02, ticks=ticks)
        cb.set_label(label)
        cb.outline.set_visible(False)

    suffix = 'contours' if args.metric == 'rates' else args.metric
    out = Path(args.out) if args.out else HERE / 'figures' / f"{args.method}_{suffix}"
    out.parent.mkdir(parents=True, exist_ok=True)
    for ext in ('pdf', 'png'):
        fig.savefig(f"{out}.{ext}", dpi=200, facecolor=SURFACE)
    print(f"  {out}.pdf / .png")


if __name__ == '__main__':
    main()
