#!/usr/bin/env python3
"""
sim_runner.py  —  Single simulation run with termination monitoring.

Termination conditions (Configured via ablation_config.yaml):
  - SUCCESS  : drone travels > success_x_dist_m in the x-direction
  - COLLISION: IMU sensor fires massive spike (collision with environment)
  - TIMEOUT  : less than progress_min_dx_m progress in progress_window_s

Returns exit codes:
  0  = SUCCESS
  1  = COLLISION
  2  = TIMEOUT
  3  = ERROR (launch failure)
"""

import atexit
import subprocess
import time
import sys
import os
import signal
import threading
import argparse
import yaml
from typing import List, Tuple, Optional
from dataclasses import dataclass, field
from enum import IntEnum

# ── ROS 2 / message imports ───────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float32MultiArray
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    raise ImportError("ROS 2 (rclpy) is not available. Please source your ROS workspace and try again.")


class ExitCode(IntEnum):
    SUCCESS   = 0
    COLLISION = 1
    TIMEOUT   = 2
    ERROR     = 3


# ── Process management ────────────────────────────────────────────────────────

processes: List[Tuple[subprocess.Popen, str]] = []
_shutting_down = False


def cleanup():
    global _shutting_down
    if _shutting_down:
        return
    _shutting_down = True

    # 1. Ask processes to shut down gracefully.
    #    NOTE: signal the process GROUP even when the direct child has already
    #    exited. Launched processes are `bash -c "... ros2 launch ..."`, so the
    #    bridges are grandchildren; if the bash wrapper dies first, skipping it
    #    here leaves the whole group running and they get reparented to init,
    #    which is how ros_gz_bridge/parameter_bridge instances survive a run and
    #    keep publishing /clock against the next simulation.
    for proc, name in reversed(processes):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except (ProcessLookupError, OSError):
            pass

    # 2. Wait for them to exit, then force kill if they hang
    for proc, name in reversed(processes):
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                pgid = os.getpgid(proc.pid)
                os.killpg(pgid, signal.SIGKILL)
            except (ProcessLookupError, OSError):
                pass
            proc.wait()

    # 3. AGGRESSIVE CLEANUP: nuke anything orphaned by a group kill that missed.
    #    Skipped when running parallel environments (run_ablation_parallel.py):
    #    these patterns match by process NAME, so one slot finishing a run would
    #    kill every other slot's Gazebo and bridges. The process-group teardown
    #    above is correctly scoped and is sufficient on its own. Unset by
    #    default, so the sequential path behaves exactly as before.
    if os.environ.get("SIM_RUNNER_NO_GLOBAL_SWEEP") == "1":
        time.sleep(1.0)
        return

    #    Same list as scorched_earth_cleanup() in the ablation drivers -- in
    #    particular the ros_gz bridges, which the previous `killall ros2 ruby gz
    #    ign` did not match at all. `killall -9 ros2` is deliberately NOT used:
    #    it also kills unrelated `ros2` CLI invocations.
    subprocess.run(
        "pkill -9 -x gz; pkill -9 -x ruby; pkill -9 -f 'gz sim'; "
        "pkill -9 -x ros_gz_bridge; pkill -9 -x parameter_bridge; pkill -9 -x image_bridge; "
        "pkill -9 -f static_transform_publisher; pkill -9 -f robot_state_publisher; "
        "pkill -9 -f 'ros2 launch'",
        shell=True,
        stderr=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
    )

    # Brief pause to let the OS release the ports and memory
    time.sleep(1.0)


def _terminate_handler(signum, _frame):
    """
    SIGINT becomes KeyboardInterrupt and unwinds through run_single's `finally`,
    but a plain SIGTERM (`kill`, `pkill` without -9) terminates the interpreter
    without unwinding, so cleanup() would never run and the whole ROS graph would
    be orphaned. Catch it, tear down, then exit.
    """
    cleanup()
    sys.exit(128 + signum)


for _sig in (signal.SIGTERM, signal.SIGHUP):
    try:
        signal.signal(_sig, _terminate_handler)
    except (ValueError, OSError):
        pass            # not the main thread / unsupported platform

# last resort for any other interpreter exit; cleanup() is idempotent via
# _shutting_down, which run_single resets per run
atexit.register(cleanup)


def start_process(cmd: List[str], name: str, env=None) -> subprocess.Popen:
    proc = subprocess.Popen(cmd, env=env, start_new_session=True)
    processes.append((proc, name))
    print(f"  [+] '{name}' started (PID {proc.pid})")
    return proc


def build_env(extra_source: Optional[str] = None) -> dict:
    source_cmds = "source /opt/ros/humble/setup.bash 2>/dev/null"
    if extra_source:
        source_cmds += f" && source {extra_source} 2>/dev/null"
    source_cmds += " && env"
    result = subprocess.run(["bash", "-c", source_cmds], capture_output=True, text=True)
    env = {}
    for line in result.stdout.splitlines():
        if "=" in line:
            k, _, v = line.partition("=")
            env[k] = v
    return env


def wait_for_controller_subscription(node: Node, odom_topic: str, timeout: float = 15.0) -> bool:
    """
    Polls the ROS2 graph until an external node subscribes to the odometry topic.
    Since the monitor itself subscribes, we wait for the count to be > 1.
    """
    start_time = time.time()
    while time.time() - start_time < timeout:
        if node.count_subscribers(odom_topic) > 1:
            print(f"  [+] Controller active! (Additional subscriber detected on {odom_topic})")
            return True
        time.sleep(0.5)

    return False


# ── Per-run telemetry ─────────────────────────────────────────────────────────

@dataclass
class RunResult:
    exit_code:  ExitCode = ExitCode.ERROR
    max_x:      float    = 0.0
    velocities: list     = field(default_factory=list)  # |v| samples (m/s)
    interventions: list  = field(default_factory=list)
    duration_s: float    = 0.0
    collision:  bool     = False
    timeout:    bool     = False
    success:    bool     = False

    # Safety filter diagnostics (filters publishing on FILTER_DIAG_TOPIC only),
    # counted per filter tick during the mission
    filter_ticks:       int  = 0
    n_solver_fail:      int  = 0      # solver did not return a solution
    n_cbf_violated:     int  = 0      # a soft CBF row needed a slack
    n_fov_violated:     int  = 0      # a soft FOV row needed a slack
    n_infeasible:       int  = 0      # any of the three above
    n_unsafe:           int  = 0      # min CBF value of the selected certificate < 0
    cert_age_counts:    list = field(default_factory=list)  # ticks per certificate age (0 = newest)

    def _rate(self, n: int) -> Optional[float]:
        return n / self.filter_ticks if self.filter_ticks else None

    @property
    def infeasible_rate(self) -> Optional[float]:
        return self._rate(self.n_infeasible)

    @property
    def unsafe_rate(self) -> Optional[float]:
        return self._rate(self.n_unsafe)

    def filter_record(self) -> dict:
        '''Filter diagnostics of this run as a flat, JSON-ready dict (counts and rates).'''
        return {
            "filter_ticks":        self.filter_ticks,
            "n_solver_fail":       self.n_solver_fail,
            "n_cbf_violated":      self.n_cbf_violated,
            "n_fov_violated":      self.n_fov_violated,
            "n_infeasible":        self.n_infeasible,
            "n_unsafe":            self.n_unsafe,
            "infeasible_rate":     _round(self.infeasible_rate),
            "solver_fail_rate":    _round(self._rate(self.n_solver_fail)),
            "cbf_violated_rate":   _round(self._rate(self.n_cbf_violated)),
            "fov_violated_rate":   _round(self._rate(self.n_fov_violated)),
            "unsafe_rate":         _round(self.unsafe_rate),
            "cert_age_counts":     list(self.cert_age_counts),
        }

    def load_filter_record(self, rec: dict):
        '''Inverse of filter_record; missing keys (other controllers, errored runs) leave the defaults.'''
        for key in ("filter_ticks", "n_solver_fail", "n_cbf_violated", "n_fov_violated",
                    "n_infeasible", "n_unsafe"):
            setattr(self, key, int(rec.get(key, 0)))
        self.cert_age_counts = list(rec.get("cert_age_counts", []))

    @property
    def mean_velocity(self) -> float:
        return sum(self.velocities) / len(self.velocities) if self.velocities else 0.0

    @property
    def max_velocity(self) -> float:
        return max(self.velocities) if self.velocities else 0.0

    @property
    def mean_intervention(self) -> float:
        return sum(self.interventions) / len(self.interventions) if self.interventions else 0.0


def _round(x: Optional[float], nd: int = 4) -> Optional[float]:
    return None if x is None else round(x, nd)


# [solver_failed, cbf_violated, fov_violated, h_min, cert_age, n_buffered], see sim_node_lidar2.py
FILTER_DIAG_TOPIC = '/rmf/filter/diag'


class RunMonitor(Node):
    """
    Subscribes to odometry and IMU topics; evaluates termination conditions.
    Thread-safe: the main loop polls `self.done` and `self.result`.
    """

    def __init__(self, term_cfg: dict = None):
        super().__init__('sim_monitor')
        if term_cfg is None:
            term_cfg = {}

        self.result  = RunResult()
        self.done    = threading.Event()
        self._lock   = threading.Lock()

        # Dynamic parameter injection from YAML (with safe fallbacks)
        self.SUCCESS_X_DIST      = term_cfg.get('success_x_dist_m', 30.0)
        self.TIMEOUT_WINDOW      = term_cfg.get('progress_window_s', 5.0)
        self.TIMEOUT_MIN_DX      = term_cfg.get('progress_min_dx_m', 1.0)
        self.IMU_ACCEL_THRESHOLD = term_cfg.get('imu_accel_threshold', 40.0)
        self.IMU_STABILIZATION_S = term_cfg.get('imu_stabilization_s', 3.0)

        self._start_x:    Optional[float] = None
        self._start_time: Optional[float] = None
        self.latest_time: Optional[float] = None

        self._window_start_x: float = 0.0
        self._window_start_t: float = 0.0

        # Mission tracker based on geometric takeoff altitude
        self._mission_armed = False

        self._odom_sub = None
        self._imu_sub  = None
        self._latest_heuristic_acc = None 
        self._heur_sub = None
        self._acc_sub = None
        self._diag_sub = None

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        if self.done.is_set():
            return

        # ROS2 Time extraction from header
        now = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
        if now == 0.0:
            return  # Wait until Gazebo actually starts publishing valid time

        self.latest_time = now

        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        speed = (vel.x**2 + vel.y**2 + vel.z**2) ** 0.5

        with self._lock:
            # ─────────────────────────────────────────────────────────────
            # TAKEOFF GUARD: Do not start timers until drone reaches [0,0,4]
            # ─────────────────────────────────────────────────────────────
            if not self._mission_armed:
                dist_to_takeoff = ((pos.x - 0.0)**2 + (pos.y - 0.0)**2 + (pos.z - 4.0)**2)**0.5
                if dist_to_takeoff <= 0.5:
                    self._mission_armed = True
                    print("  [+] Takeoff altitude reached! Mission timers and collision detection ARMED.")
                    
                    # Reset all trackers as if the run starts RIGHT NOW
                    self._start_x         = pos.x
                    self._start_time      = now
                    self._window_start_x  = pos.x
                    self._window_start_t  = now
                return  # Silent return, do not track max_x or timeouts during takeoff!

            # ── GAZEBO GRACE PERIOD (Post-Takeoff) ────────────────────────────
            if now - self._start_time < 0.5:
                self._start_x = pos.x
                self._window_start_x = pos.x
                return
            # ──────────────────────────────────────────────────────────────────

            dx_total = pos.x - self._start_x
            self.result.max_x = max(self.result.max_x, dx_total)
            self.result.velocities.append(speed)

            # ── SUCCESS ───────────────────────────────────────────────────────
            if dx_total >= self.SUCCESS_X_DIST:
                self.result.success   = True
                self.result.exit_code = ExitCode.SUCCESS
                self.result.duration_s = now - self._start_time
                self.done.set()
                return

            # ── TIMEOUT check ─────────────────────────────────────────────────
            elapsed_window = now - self._window_start_t
            if elapsed_window >= self.TIMEOUT_WINDOW:
                dx_window = pos.x - self._window_start_x
                if dx_window < self.TIMEOUT_MIN_DX:
                    self.result.timeout   = True
                    self.result.exit_code = ExitCode.TIMEOUT
                    self.result.duration_s = now - self._start_time
                    self.done.set()
                    return
                # slide the window forward
                self._window_start_x = pos.x
                self._window_start_t = now

    def _imu_cb(self, msg: Imu):
        if self.done.is_set():
            return

        with self._lock:
            # Absolute immunity to collisions during spool-up and takeoff
            if not self._mission_armed:
                return

            if self._start_time is None or self.latest_time is None:
                return 
            elapsed = self.latest_time - self._start_time
            if elapsed < self.IMU_STABILIZATION_S:
                return

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        accel_mag = (ax**2 + ay**2 + az**2) ** 0.5

        if accel_mag > self.IMU_ACCEL_THRESHOLD:
            with self._lock:
                self.result.collision  = True
                self.result.exit_code  = ExitCode.COLLISION
                if self._start_time is not None and self.latest_time is not None:
                    self.result.duration_s = self.latest_time - self._start_time
                self.done.set()

    def _heur_cb(self, msg: Twist):
        if self.done.is_set():
            return
        with self._lock:
            self._latest_heuristic_acc = msg

    def _acc_cb(self, msg: Twist):
        if self.done.is_set():
            return
        with self._lock:
            # Only track interventions once in flight and if we have a baseline
            if not self._mission_armed or self._latest_heuristic_acc is None:
                return
            
            ref = self._latest_heuristic_acc
            dx = msg.linear.x - ref.linear.x
            dy = msg.linear.y - ref.linear.y
            dz = msg.linear.z - ref.linear.z
            dax = msg.angular.x - ref.angular.x
            day = msg.angular.y - ref.angular.y
            daz = msg.angular.z - ref.angular.z
            
            l2_err = (dx**2 + dy**2 + dz**2 + dax**2 + day**2 + daz**2) ** 0.5
            self.result.interventions.append(l2_err)

    def _diag_cb(self, msg: Float32MultiArray):
        if self.done.is_set():
            return
        failed, cbf_v, fov_v, h_min, age = msg.data[:5]
        age = int(round(age))
        with self._lock:
            # count the same mission window as the interventions
            if not self._mission_armed:
                return
            r = self.result
            r.filter_ticks   += 1
            r.n_solver_fail  += failed > 0.5
            r.n_cbf_violated += cbf_v > 0.5
            r.n_fov_violated += fov_v > 0.5
            r.n_infeasible   += (failed > 0.5) or (cbf_v > 0.5) or (fov_v > 0.5)
            r.n_unsafe       += h_min < 0.0
            if age >= len(r.cert_age_counts):
                r.cert_age_counts.extend([0] * (age + 1 - len(r.cert_age_counts)))
            r.cert_age_counts[age] += 1

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def subscribe(self,
                  odom_topic: str = "/rmf/odom",
                  imu_topic:  str = "/rmf/imu"):
        
        # Applied QoS profiles to match best-effort sensor data
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_cb, qos_profile_sensor_data)
        self._imu_sub  = self.create_subscription(Imu, imu_topic, self._imu_cb, qos_profile_sensor_data)
        self._heur_sub = self.create_subscription(Twist, '/rmf/cmd/heuristic_acc', self._heur_cb, 10)
        self._acc_sub  = self.create_subscription(Twist, '/rmf/cmd/acc', self._acc_cb, 10)
        self._diag_sub = self.create_subscription(Float32MultiArray, FILTER_DIAG_TOPIC, self._diag_cb, 50)

    def unsubscribe(self):
        if self._odom_sub:
            self.destroy_subscription(self._odom_sub)
        if self._imu_sub:
            self.destroy_subscription(self._imu_sub)
        if self._heur_sub:
            self.destroy_subscription(self._heur_sub)
        if self._acc_sub:
            self.destroy_subscription(self._acc_sub)
        if self._diag_sub:
            self.destroy_subscription(self._diag_sub)


# ── Per-run report ────────────────────────────────────────────────────────────

def print_run_statistics(r: RunResult):
    '''Print the statistics of one finished run, including the safety filter diagnostics if any.'''
    print(f"\n  ── Run statistics ──────────────────────────────────────")
    print(f"  Outcome             : {r.exit_code.name}")
    print(f"  max x / duration    : {r.max_x:.2f} m / {r.duration_s:.1f} s")
    print(f"  velocity mean / max : {r.mean_velocity:.2f} / {r.max_velocity:.2f} m/s")
    print(f"  mean intervention   : {r.mean_intervention:.4f}")
    if r.filter_ticks:
        rate = lambda n: f"{n / r.filter_ticks * 100:6.2f}%  ({n}/{r.filter_ticks})"
        print(f"  filter ticks        : {r.filter_ticks}")
        print(f"  QP infeasible       : {rate(r.n_infeasible)}")
        print(f"    solver failed     : {rate(r.n_solver_fail)}")
        print(f"    CBF slack         : {rate(r.n_cbf_violated)}")
        print(f"    FOV slack         : {rate(r.n_fov_violated)}")
        print(f"  certificate h < 0   : {rate(r.n_unsafe)}")
        print("  certificate age     : " + "  ".join(f"{age}:{n / r.filter_ticks * 100:.1f}%"
                                                     for age, n in enumerate(r.cert_age_counts)))
    print(f"  ────────────────────────────────────────────────────────", flush=True)


# ── Top-level single-run function ─────────────────────────────────────────────

def run_single(
    controller_script: str,
    config_path: str         = "/home/arl/lmf_ws/src/rmf_gz/ablation_config.yaml",
    world_override: str      = None,
    headless_override: bool  = None,
    timeout_override: float  = None,
) -> RunResult:
    """
    Starts the monitor node, launches the controller, launches Gazebo sim,
    monitors until a termination condition fires, then cleans up safely.
    """
    global processes, _shutting_down
    processes      = []
    _shutting_down = False

    ros_env    = build_env()
    ros_ws_env = build_env("install/setup.bash")

    # Load the YAML configuration
    try:
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"  [!] Config file {config_path} not found. Defaulting to safe hardcoded values.")
        config = {}

    sim_cfg  = config.get('simulation', {})
    term_cfg = config.get('termination', {})
    top_cfg  = config.get('topics', {})

    # Priority mapping: 1. CLI Override -> 2. YAML File -> 3. Hardcoded Default
    world         = world_override if world_override is not None else sim_cfg.get('world_name', 'random3d')
    headless      = headless_override if headless_override is not None else sim_cfg.get('headless', True)
    run_timeout_s = timeout_override if timeout_override is not None else sim_cfg.get('run_timeout_s', 300.0)

    robot_name = sim_cfg.get('robot_name', 'rmf')
    spawn_x    = sim_cfg.get('spawn_pose', {}).get('x', 0.0)
    spawn_y    = sim_cfg.get('spawn_pose', {}).get('y', 0.0)
    spawn_z    = sim_cfg.get('spawn_pose', {}).get('z', 1.0)

    odom_topic = top_cfg.get('odom', '/rmf/odom')
    imu_topic  = top_cfg.get('imu', '/rmf/imu')

    result = RunResult()
    
    # Pre-declare variables for safe teardown in the finally block
    monitor = None
    executor = None
    spin_thread = None

    try:
        # 1. Start ROS2 Monitor Node ──────────────────────────────────────────
        rclpy.init()
        monitor = RunMonitor(term_cfg)
        monitor.subscribe(odom_topic=odom_topic, imu_topic=imu_topic)
        
        executor = MultiThreadedExecutor()
        executor.add_node(monitor)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        # 2. Controller ───────────────────────────────────────────────────────
        print("  Starting controller ahead of simulation...")
        start_process(
            ["bash", "-c",
             f"source /opt/ros/humble/setup.bash && "
             f"source install/setup.bash && "
             f"python3 {controller_script} --config {config_path}"],  # <--- Added config flag!
            name="controller",
            env=ros_ws_env,
        )

        print("  Waiting for controller to register with ROS graph...")
        if not wait_for_controller_subscription(monitor, odom_topic=odom_topic, timeout=15.0):
            print("  [!] WARNING: Controller did not subscribe to odometry within 15s. Launching sim anyway...")
        else:
            time.sleep(0.5)

        # 3. Simulation launch ────────────────────────────────────────────────
        print("  Launching Gazebo Sim (auto-running)...")
        headless_str = "true" if headless else "false"
        launch_cmd = (
            "source /opt/ros/humble/setup.bash && "
            "source install/setup.bash && "
            f"ros2 launch rmf_gz start_sim.launch.xml "
            f"world:={world} headless:={headless_str} "
            f"robot_name:={robot_name} "
            f"x:={spawn_x} y:={spawn_y} z:={spawn_z}"
        )
        start_process(["bash", "-c", launch_cmd], name="lmf_sim", env=ros_ws_env)

        # 4. Wait for termination ─────────────────────────────────────────────
        print(f"  Monitoring... (hard timeout {run_timeout_s:.0f} s sim time)")

        wall_start   = time.time()
        wall_timeout = (run_timeout_s * 5.0) + 30.0

        while not monitor.done.is_set():
            if time.time() - wall_start > wall_timeout:
                print("  [!] Wall-clock safety timeout triggered (Gazebo might have crashed).")
                break

            if monitor._start_time is not None and monitor.latest_time is not None:
                sim_elapsed = monitor.latest_time - monitor._start_time
                if sim_elapsed >= run_timeout_s:
                    monitor.result.exit_code  = ExitCode.TIMEOUT
                    monitor.result.timeout    = True
                    monitor.result.duration_s = sim_elapsed
                    monitor.done.set()
                    break

            monitor.done.wait(timeout=0.2)

        if not monitor.done.is_set():
            result.exit_code = ExitCode.ERROR
        else:
            result = monitor.result

    finally:
        cleanup()
        if ROS_AVAILABLE:
            try:
                # 1. Stop processing incoming messages
                if monitor is not None:
                    monitor.unsubscribe()
                
                # 2. Command the executor to break its spin loop
                if executor is not None:
                    executor.shutdown()
                
                # 3. Wait for the background thread to safely exit
                if spin_thread is not None:
                    spin_thread.join(timeout=2.0)
                
                # 4. Deallocate node structures
                if monitor is not None:
                    monitor.destroy_node()
                
                # 5. Shut down global ROS context
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception as e:
                print(f"  [!] Clean teardown warning: {e}")

    print_run_statistics(result)
    return result


# ── CLI entry-point (single run) ──────────────────────────────────────────────

def _parse_args():
    p = argparse.ArgumentParser(description="Run a single simulation trial.")
    p.add_argument("controller", help="Path to the controller Python script")
    p.add_argument("--config",   default="ablation_config.yaml", help="Path to YAML configuration file")
    p.add_argument("--world",    default=None, help="Override world name from YAML")
    p.add_argument("--headless", action="store_true", help="Force headless mode if passed")
    p.add_argument("--timeout",  type=float, default=None, help="Override timeout from YAML")
    return p.parse_args()


if __name__ == "__main__":
    if not ROS_AVAILABLE:
        print("ERROR: ROS2 (rclpy) is not available. Source your ROS workspace first.")
        sys.exit(3)

    args = _parse_args()
    
    # Process CLI headless flag carefully so it doesn't overwrite a True YAML setting if omitted
    cli_headless = True if args.headless else None
    
    r = run_single(
        controller_script=args.controller,
        config_path=args.config,
        world_override=args.world,
        headless_override=cli_headless,
        timeout_override=args.timeout,
    )
    sys.exit(int(r.exit_code))