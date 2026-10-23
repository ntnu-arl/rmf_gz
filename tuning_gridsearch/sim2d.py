"""
2D double-integrator stand-in for the Gazebo benchmark, for tuning the safety
filters in closed loop at a fraction of the cost.

What is simulated, and what is the real code:
  * world      : the random2d Gazebo world (scripts/rand_poisson2d.py of rmf_gz):
                 three 8 x 8 m chunks of Poisson-disc pillars in an 8 m wide
                 corridor. The Poisson radius is drawn per world; worlds without
                 a collision-free path for a DRONE_RADIUS disc are redrawn.
  * vehicle    : planar double integrator at the takeoff altitude, unit attitude
                 (body = vehicle = world frame), integrated exactly over each
                 control step (zero-order hold).
  * LiDAR      : the Gazebo gpu_ray of the rmf model (512 x 128 rays, +-45 deg,
                 0.22-4 m, 3 cm Gaussian range noise). The world is the 2D
                 world extruded vertically (pillars and walls span floor to
                 ceiling; floor and ceiling are out of range at 4 m altitude),
                 so each ray hits where its azimuth's 2D ray hits and the full
                 point cloud is the 2D scan stacked over the 128 elevations.
  * real code  : the perception node's point-cloud -> range-image projection,
                 the heuristic reference controller node and the safety filter
                 node of each method (constructor, history pairing,
                 compute_safe_action, output clamp + low-pass, publish), driven
                 through their callbacks with the ROS transport stubbed out
                 (ros_stubs.py). The VAE latent is not computed (the range-image
                 filters do not use it).
  * timing     : control at FILTER_RATE, scans every FILTER_RATE / LIDAR_RATE
                 steps; the scan pose is the true pose at scan time and there
                 is no transport or compute latency.
  * outcome    : as sim_runner.RunMonitor: SUCCESS once x advanced
                 success_x_dist_m, TIMEOUT if a progress window gains less than
                 progress_min_dx_m (or at run_timeout_s), COLLISION when the
                 airframe's bounding circle touches an obstacle.

Single episode, with a plot:
    python sim2d.py --method daf --world-seed 3 --plot /tmp/ep.png --set DAF_K3=2.0
"""
import argparse
import copy
import importlib
import time
from dataclasses import dataclass, asdict
from pathlib import Path

import numpy as np
import scipy.stats
import yaml
from scipy import ndimage

import ros_stubs   # sibling module: this folder is not part of the nCBF3D package
ros_stubs.install()

import torch
import torch.nn.functional as F

from nCBF3D.G_ROS_interface.ROS2 import node_base
from nCBF3D.G_ROS_interface.ROS2.node_base import HeuristicControlNode, LidarPerceptionNode


# ======================================================================
# constants of the Gazebo setup (rmf_gz)
# ======================================================================
# resources/robots/rmf/model.sdf, sensor "lidar"
LIDAR_AZIMUTHS   = np.linspace(-3.1416, 3.1416, 512)
LIDAR_ELEVATIONS = np.linspace(-0.7854, 0.7854, 128)
LIDAR_RANGE_MIN  = 0.22
LIDAR_RANGE_MAX  = 4.0
LIDAR_NOISE_STD  = 0.03

# scripts/rand_poisson2d.py
CHUNK_SIZE       = 8.0                  # poisson_domain_size
N_CHUNKS         = 3
FIRST_CHUNK_X    = 2.0 + CHUNK_SIZE / 2 # x_offset of chunk 0
CHUNK_PITCH      = CHUNK_SIZE + 1.0
WALL_Y           = CHUNK_SIZE / 2 - 0.05  # inner face of the 0.1 m wall boxes at y = +-4

# the airframe's bounding circle: the collision mesh spans x +-0.1695, y +-0.19
# (node_base.SELF_BOX_*), so a contact registers at a centre distance of 0.255 m
COLLISION_RADIUS = float(np.hypot(0.1695, 0.19))

FLIGHT_ALTITUDE  = node_base.SafetyFilterNode.TAKEOFF_ALTITUDE

METHODS = {
    'ccbf': 'nCBF3D.G_ROS_interface.ROS2.sim_node_lidar_ccbf:CompositeCBFControlNode',
    'daf':  'nCBF3D.G_ROS_interface.ROS2.sim_node_lidar_daf:DAFControlNode',
}


def load_node_class(spec):
    ''''module:Class' or a key of METHODS -> class'''
    module, cls = METHODS.get(spec, spec).split(':')
    return getattr(importlib.import_module(module), cls)


# ======================================================================
# configuration (same merge as node_base.load_config)
# ======================================================================
def load_base_config(ablation_path=node_base.DEFAULT_ABLATION_PATH):
    with open(node_base.CBF_CONFIG_PATH) as f:
        cbf_data = yaml.safe_load(f) or {}
    with open(ablation_path) as f:
        ablation_data = yaml.safe_load(f) or {}
    return cbf_data | ablation_data


def make_config(base, overrides):
    '''A fresh Config object, as each node process gets its own copy in launch().'''
    return type('Config', (object,), copy.deepcopy(base) | dict(overrides))()


# ======================================================================
# world
# ======================================================================
@dataclass
class World:
    seed:           int
    poisson_radius: float
    pillar_radius:  float
    centers:        np.ndarray   # (M, 2)

    def clearance(self, p):
        '''Distance from p to the nearest obstacle surface.'''
        d_pillar = np.min(np.linalg.norm(self.centers - p, axis=1)) - self.pillar_radius
        return min(d_pillar, WALL_Y - abs(p[1]))

    def to_dict(self):
        return dict(seed=self.seed, poisson_radius=self.poisson_radius,
                    pillar_radius=self.pillar_radius, n_pillars=len(self.centers))


def poisson_pillars(poisson_radius, base_seed):
    '''Pillar centres exactly as rand_poisson2d.py places them.'''
    centers, x_offset = [], FIRST_CHUNK_X
    for j in range(N_CHUNKS):
        engine = scipy.stats.qmc.PoissonDisk(d=2, radius=poisson_radius / CHUNK_SIZE,
                                             ncandidates=100, seed=base_seed + j)
        pts = (engine.random(1000) - 0.5) * CHUNK_SIZE
        centers.append(pts + [x_offset, 0.0])
        x_offset += CHUNK_PITCH
    return np.vstack(centers)


def is_passable(centers, pillar_radius, robot_radius, goal_x, res=0.05):
    '''Whether a disc of robot_radius can get from the spawn point to x >= goal_x.'''
    xs = np.arange(-1.0, goal_x + 1.0, res)
    ys = np.arange(-WALL_Y, WALL_Y + res, res)
    X, Y = np.meshgrid(xs, ys, indexing='ij')
    free = np.abs(Y) < WALL_Y - robot_radius
    for c in centers:
        free &= (X - c[0]) ** 2 + (Y - c[1]) ** 2 > (pillar_radius + robot_radius) ** 2
    labels, _ = ndimage.label(free)
    start = labels[np.argmin(np.abs(xs)), np.argmin(np.abs(ys))]
    return start != 0 and np.any(labels[xs >= goal_x] == start)


def generate_world(seed, poisson_range, pillar_radius, robot_radius, goal_x, max_attempts=100):
    '''Deterministic in seed; redraws until the world is passable.'''
    rng = np.random.default_rng(seed)
    for _ in range(max_attempts):
        r = float(rng.uniform(*poisson_range))
        centers = poisson_pillars(r, int(rng.integers(0, 1_000_000)))
        if is_passable(centers, pillar_radius, robot_radius, goal_x):
            return World(seed=seed, poisson_radius=r, pillar_radius=pillar_radius, centers=centers)
    raise RuntimeError(f"no passable world after {max_attempts} draws (seed {seed})")


# ======================================================================
# LiDAR
# ======================================================================
class Lidar:
    def __init__(self, noise_std=LIDAR_NOISE_STD, rng=None):
        self.noise_std = noise_std
        self.rng = rng if rng is not None else np.random.default_rng()
        self.dir_xy = np.stack([np.cos(LIDAR_AZIMUTHS), np.sin(LIDAR_AZIMUTHS)], axis=1)   # (H, 2)
        el = LIDAR_ELEVATIONS[:, None]                                                      # (V, 1)
        self.dir_3d = np.stack([np.cos(el) * np.cos(LIDAR_AZIMUTHS),
                                np.cos(el) * np.sin(LIDAR_AZIMUTHS),
                                np.sin(el) * np.ones_like(LIDAR_AZIMUTHS)], axis=-1)        # (V, H, 3)
        self.cos_el = np.cos(el)

    def horizontal_ranges(self, world, p):
        '''Distance along each azimuth to the first pillar or wall (inf beyond reach).'''
        reach = LIDAR_RANGE_MAX + world.pillar_radius
        near  = world.centers[np.linalg.norm(world.centers - p, axis=1) < reach]
        d_h   = np.full(len(self.dir_xy), np.inf)

        if len(near):
            D  = near - p                                   # (M, 2)
            t  = D @ self.dir_xy.T                          # (M, H) distance to the closest approach
            m2 = np.sum(D ** 2, axis=1)[:, None] - t ** 2   # squared miss distance
            r2 = world.pillar_radius ** 2
            t1 = t - np.sqrt(np.maximum(r2 - m2, 0.0))
            d_h = np.min(np.where((m2 <= r2) & (t1 > 0.0), t1, np.inf), axis=0)

        uy = self.dir_xy[:, 1]
        with np.errstate(divide='ignore', invalid='ignore'):
            t_wall = np.where(uy > 1e-9, (WALL_Y - p[1]) / uy,
                              np.where(uy < -1e-9, (-WALL_Y - p[1]) / uy, np.inf))
        return np.minimum(d_h, t_wall)

    def point_cloud(self, world, p):
        '''(V*H, 3) float32 points in the sensor frame, inf where there is no return.'''
        rng_3d = self.horizontal_ranges(world, p)[None, :] / self.cos_el    # extruded world
        if self.noise_std > 0.0:
            rng_3d = rng_3d + self.rng.normal(0.0, self.noise_std, rng_3d.shape)
        rng_3d = np.where((rng_3d >= LIDAR_RANGE_MIN) & (rng_3d <= LIDAR_RANGE_MAX), rng_3d, np.inf)
        return (self.dir_3d * rng_3d[..., None]).reshape(-1, 3).astype(np.float32)


class Perception:
    '''
    The perception node's projection, on a node constructed without its VAE
    (only project_pc_to_range_image runs, on the attributes it reads).
    '''
    def __init__(self, cfg):
        node = object.__new__(LidarPerceptionNode)
        node.cfg       = cfg
        node.device    = cfg.DEVICE
        node.H_RES     = getattr(cfg, 'IMG_WIDTH', 512)
        node.V_RES     = getattr(cfg, 'IMG_HEIGHT', 128)
        node.V_FOV_RAD = np.radians(getattr(cfg, 'VFOV', 90.0))
        node._range_img_buffer = torch.full((node.V_RES, node.H_RES), cfg.MAX_RANGE,
                                            dtype=torch.float32, device=node.device)
        self.node = node

    def __call__(self, cloud):
        msg = ros_stubs.PointCloud2()
        msg.point_step = 12
        msg.data = cloud.tobytes()
        with torch.no_grad():
            range_img = self.node.project_pc_to_range_image(msg)
            # as in LidarPerceptionNode._encode_and_publish
            downsampled = F.avg_pool2d(range_img.unsqueeze(0).unsqueeze(0), kernel_size=8, stride=8).squeeze()
            return range_img.cpu().numpy().copy(), downsampled.cpu().numpy().copy()


# ======================================================================
# closed loop
# ======================================================================
def _image_msg(stamp, img):
    msg = node_base.LidarPerceptionNode._range_image_msg(ros_stubs.Header(stamp=stamp), img)
    return msg


def _odom_msg(stamp, p, v):
    msg = ros_stubs.Odometry()
    msg.header.stamp = stamp
    msg.pose.pose.position = ros_stubs.Vec3(float(p[0]), float(p[1]), FLIGHT_ALTITUDE)
    msg.pose.pose.orientation = ros_stubs.Quat(0.0, 0.0, 0.0, 1.0)
    msg.twist.twist.linear = ros_stubs.Vec3(float(v[0]), float(v[1]), 0.0)   # body = world
    return msg


def _pose_msg(stamp, odom):
    msg = ros_stubs.PoseStamped()
    msg.header.stamp = stamp
    msg.pose = odom.pose.pose
    return msg


def _xy(twist):
    return np.array([twist.linear.x, twist.linear.y])


@dataclass
class EpisodeResult:
    outcome:           str
    duration_s:        float
    max_x:             float
    mean_speed:        float
    max_speed:         float
    min_clearance:     float
    mean_intervention: float
    n_filter_warnings: int
    wall_s:            float


def run_episode(node_cls, params, world, base_cfg=None, device='cpu', noise_seed=0, record=False):
    '''
    One closed-loop run of node_cls with the method parameters `params` in `world`.
    Returns (EpisodeResult, trajectory dict or None).
    '''
    t_wall = time.perf_counter()
    base_cfg = base_cfg if base_cfg is not None else load_base_config()
    term = base_cfg.get('termination', {})
    success_x = term.get('success_x_dist_m', 30.0)
    window_s  = term.get('progress_window_s', 5.0)
    min_dx    = term.get('progress_min_dx_m', 0.5)
    timeout_s = base_cfg.get('simulation', {}).get('run_timeout_s', 100.0)

    sim_overrides = dict(DEVICE=device, LIDAR_RATE=20.0, USE_HEURISTIC=True,
                         USE_BLIND=False, USE_RANDOM=False, USE_EGO=False, ADD_LIDAR_NOISE=False)
    cfg_filter = make_config(base_cfg, sim_overrides | dict(params))
    cfg_ref    = make_config(base_cfg, sim_overrides | dict(params))
    cfg_perc   = make_config(base_cfg, sim_overrides | dict(params))

    filt = node_cls(cfg_filter)
    filt.watchdog_healthy = True   # no spool-up: the sim clock is steady
    filt.takeoff_complete = True   # the episode starts at the takeoff altitude
    ref  = HeuristicControlNode(cfg_ref, node_cls.needs_vertical_damping(cfg_filter))
    heur_pub = ref.publishers['/rmf/cmd/heuristic_acc']
    perceive = Perception(cfg_perc)
    lidar    = Lidar(rng=np.random.default_rng(noise_seed))

    dt         = 1.0 / cfg_filter.FILTER_RATE
    scan_every = max(1, int(round(cfg_filter.FILTER_RATE / cfg_filter.LIDAR_RATE)))
    n_steps    = int(round(timeout_s / dt))

    p, v = np.zeros(2), np.zeros(2)
    max_x = 0.0
    window_t0, window_x0 = 0.0, 0.0
    speeds, interventions = [], []
    min_clearance, outcome, t = np.inf, 'TIMEOUT', 0.0
    traj = dict(p=[], u_ref=[], u=[], scan_p=[]) if record else None

    for k in range(n_steps):
        t = k * dt
        ros_stubs.SIM_CLOCK.set(t)
        stamp = ros_stubs.Stamp.from_sec(t)
        odom  = _odom_msg(stamp, p, v)

        if k % scan_every == 0:
            full, down = perceive(lidar.point_cloud(world, p))
            filt.scan_pose_callback(_pose_msg(stamp, odom))
            filt.image_callback(_image_msg(stamp, full))
            ref.image_callback(_image_msg(stamp, down))
            if record:
                traj['scan_p'].append(p.copy())

        ref.odometry_callback(odom)
        filt.odometry_callback(odom)

        ref.timer_callback()
        filt.heuristic_sp_callback(heur_pub.last)
        filt.generate_safe_action()
        cmd = filt.acc_pub.last

        u_ref = _xy(heur_pub.last)
        u     = _xy(cmd) if cmd is not None else np.zeros(2)
        if not np.all(np.isfinite(u)):
            outcome = 'NAN'   # the filter produced a non-finite command
            break
        interventions.append(np.linalg.norm(_xy(cmd) - u_ref) if cmd is not None else 0.0)
        if record:
            traj['p'].append(p.copy()); traj['u_ref'].append(u_ref); traj['u'].append(u)

        # exact zero-order-hold integration of the double integrator
        p = p + v * dt + 0.5 * u * dt ** 2
        v = v + u * dt
        t = (k + 1) * dt
        speeds.append(np.linalg.norm(v))
        max_x = max(max_x, p[0])

        clearance = world.clearance(p)
        min_clearance = min(min_clearance, clearance)
        if clearance < COLLISION_RADIUS:
            outcome = 'COLLISION'
            break
        if p[0] >= success_x:
            outcome = 'SUCCESS'
            break
        if t - window_t0 >= window_s:
            if p[0] - window_x0 < min_dx:
                outcome = 'TIMEOUT'
                break
            window_t0, window_x0 = t, p[0]

    if record:
        traj = {key: np.array(val) for key, val in traj.items()}
        traj['p'] = np.vstack([traj['p'], p[None]])

    result = EpisodeResult(
        outcome=outcome, duration_s=round(t, 3), max_x=round(float(max_x), 3),
        mean_speed=round(float(np.mean(speeds)), 4) if speeds else 0.0,
        max_speed=round(float(np.max(speeds)), 4) if speeds else 0.0,
        min_clearance=round(float(min_clearance), 4),
        mean_intervention=round(float(np.nanmean(interventions)), 4) if interventions else 0.0,
        n_filter_warnings=int(sum(filt.get_logger().warn_counts.values())),
        wall_s=round(time.perf_counter() - t_wall, 2))
    return result, traj


# ======================================================================
# plotting / CLI
# ======================================================================
def plot_episode(world, traj, result, path, title=''):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    fig, ax = plt.subplots(figsize=(16, 4.8))
    for c in world.centers:
        ax.add_patch(Circle(c, world.pillar_radius, color='dimgray', zorder=2))
    ax.axhline(WALL_Y, color='k', lw=2); ax.axhline(-WALL_Y, color='k', lw=2)
    ax.plot(traj['p'][:, 0], traj['p'][:, 1], 'C0', lw=1.5, zorder=3)
    end = traj['p'][-1]
    ax.add_patch(Circle(end, COLLISION_RADIUS, fill=False, color='C3', zorder=4))
    ax.set_xlim(-1.5, 31.5); ax.set_ylim(-WALL_Y - 0.5, WALL_Y + 0.5); ax.set_aspect('equal')
    ax.set_title(f"{title}  |  {result.outcome}  t={result.duration_s:.1f}s  "
                 f"v={result.mean_speed:.2f}m/s  min clearance={result.min_clearance:.2f}m  "
                 f"(Poisson r={world.poisson_radius:.2f})")
    fig.tight_layout(); fig.savefig(path, dpi=130); plt.close(fig)


def parse_overrides(items):
    out = {}
    for item in items or []:
        key, _, val = item.partition('=')
        out[key] = yaml.safe_load(val)
    return out


def main():
    ap = argparse.ArgumentParser(description="Run one 2D closed-loop episode of a safety filter node")
    ap.add_argument('--method', default='daf', help=f"one of {list(METHODS)} or module:Class")
    ap.add_argument('--world-seed', type=int, default=0)
    ap.add_argument('--poisson-range', type=float, nargs=2, default=[1.5, 3.0])
    ap.add_argument('--pillar-radius', type=float, default=0.5)
    ap.add_argument('--set', nargs='*', default=[], help="method parameters, e.g. DAF_K3=2.0")
    ap.add_argument('--device', default='cpu')
    ap.add_argument('--plot', default=None, help="save a trajectory plot here")
    args = ap.parse_args()

    torch.set_num_threads(1)
    base = load_base_config()
    world = generate_world(args.world_seed, args.poisson_range, args.pillar_radius,
                           base['DRONE_RADIUS'], base.get('termination', {}).get('success_x_dist_m', 30.0))
    result, traj = run_episode(load_node_class(args.method), parse_overrides(args.set), world,
                               base_cfg=base, device=args.device, noise_seed=args.world_seed,
                               record=args.plot is not None)
    print(asdict(result))
    if args.plot:
        plot_episode(world, traj, result, args.plot, title=f"{args.method} {args.set}")
        print(f"plot -> {args.plot}")


if __name__ == '__main__':
    main()
