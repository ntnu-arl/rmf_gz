#!/usr/bin/env python3
"""
sim_runner.py  —  Single simulation run with termination monitoring.

Termination conditions:
  - SUCCESS  : drone travels > 60 m in the x-direction from its spawn pose
  - COLLISION: IMU sensor fires massive spike (collision with environment)
  - TIMEOUT  : less than 3 m x-progress in the last 30 s (measured in sim time)

Returns exit codes:
  0  = SUCCESS
  1  = COLLISION
  2  = TIMEOUT
  3  = ERROR (roscore/launch failure)
"""

import subprocess
import time
import sys
import os
import signal
import socket
import threading
import argparse
from typing import List, Tuple, Optional
from dataclasses import dataclass, field
from enum import IntEnum

# ── ROS / message imports (available after sourcing ROS) ──────────────────────
try:
    import rospy
    import rosgraph
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from geometry_msgs.msg import Point
    from sensor_msgs.msg import Imu
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


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

    # 1. Ask processes to shut down gracefully
    for proc, name in reversed(processes):
        if proc.poll() is not None:
            continue
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGINT)
        except ProcessLookupError:
            pass
        except OSError:
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

    # 3. AGGRESSIVE CLEANUP: Nuke orphaned Gazebo processes (both Classic and New)
    subprocess.run(
        "killall -9 gzserver gzclient roslaunch 2>/dev/null; pkill -9 -f 'gz sim' 2>/dev/null",
        shell=True,
        stderr=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
    )

    # Brief pause to let the OS release the ports and memory
    time.sleep(1)


def start_process(cmd: List[str], name: str, env=None) -> subprocess.Popen:
    proc = subprocess.Popen(cmd, env=env, start_new_session=True)
    processes.append((proc, name))
    print(f"  [+] '{name}' started (PID {proc.pid})")
    return proc


def build_env(extra_source: Optional[str] = None) -> dict:
    source_cmds = "source /opt/ros/noetic/setup.bash 2>/dev/null"
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


def roscore_ready() -> bool:
    try:
        sock = socket.create_connection(("localhost", 11311), timeout=1)
        sock.close()
        return True
    except (socket.timeout, ConnectionRefusedError, OSError):
        return False


def wait_for_controller_subscription(odom_topic: str, timeout: float = 15.0) -> bool:
    """
    Polls the ROS Master until an external node subscribes to the odometry topic,
    proving the controller has booted up and initialized its ROS interfaces.
    """
    master = rosgraph.Master('/sim_runner')
    start_time = time.time()

    while time.time() - start_time < timeout:
        try:
            # getSystemState() returns [publishers, subscribers, services]
            _, subscribers, _ = master.getSystemState()
            for topic, nodes in subscribers:
                if topic == odom_topic:
                    # Ignore our own monitor node
                    external_nodes = [n for n in nodes if not n.startswith('/sim_monitor')]
                    if external_nodes:
                        print(f"  [+] Controller active! (Node '{external_nodes[0]}' subscribed to {odom_topic})")
                        return True
        except socket.error:
            pass
        time.sleep(0.1)

    return False


# ── Per-run telemetry ─────────────────────────────────────────────────────────

@dataclass
class RunResult:
    exit_code:  ExitCode = ExitCode.ERROR
    max_x:      float    = 0.0
    velocities: list     = field(default_factory=list)  # |v| samples (m/s)
    duration_s: float    = 0.0
    collision:  bool     = False
    timeout:    bool     = False
    success:    bool     = False

    @property
    def mean_velocity(self) -> float:
        return sum(self.velocities) / len(self.velocities) if self.velocities else 0.0

    @property
    def max_velocity(self) -> float:
        return max(self.velocities) if self.velocities else 0.0


class RunMonitor:
    """
    Subscribes to odometry and IMU topics; evaluates termination conditions.
    Thread-safe: the main loop polls `self.done` and `self.result`.

    All time comparisons use simulation time sourced from message headers —
    never rospy.get_time() / wall-clock time — to avoid apples-to-oranges
    comparisons when the sim runs slower or faster than real-time.
    """

    SUCCESS_X_DIST       = 60.0   # m
    TIMEOUT_WINDOW       = 30.0   # s  (sim time)
    TIMEOUT_MIN_DX       = 3.0    # m  — must make at least this much x-progress per window
    IMU_ACCEL_THRESHOLD  = 40.0   # m/s² — spike magnitude that signals a collision
    IMU_STABILIZATION_S  = 4.0    # s  (sim time) — ignore IMU spikes during startup transient

    def __init__(self):
        self.result  = RunResult()
        self.done    = threading.Event()
        self._lock   = threading.Lock()

        self._start_x:    Optional[float] = None
        self._start_time: Optional[float] = None

        # latest_time tracks the most recent sim timestamp seen on any topic;
        # used by _imu_cb and the main loop so they never need rospy.get_time().
        self.latest_time: Optional[float] = None

        # sliding-window timeout tracking
        self._window_start_x: float = 0.0
        self._window_start_t: float = 0.0

        # ROS subscribers (set up after rospy.init_node)
        self._odom_sub: Optional[rospy.Subscriber] = None
        self._imu_sub:  Optional[rospy.Subscriber] = None

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _odom_cb(self, msg: 'Odometry'):
        if self.done.is_set():
            return

        # Use exact simulation time from the message header
        now = msg.header.stamp.to_sec()
        if now == 0.0:
            return  # Wait until Gazebo actually starts publishing valid time

        self.latest_time = now

        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        speed = (vel.x**2 + vel.y**2 + vel.z**2) ** 0.5

        with self._lock:
            if self._start_x is None:
                self._start_x         = pos.x
                self._start_time      = now
                self._window_start_x  = pos.x
                self._window_start_t  = now
                return

            # ── GAZEBO GRACE PERIOD ───────────────────────────────────────────
            # Ignore the first 0.5 seconds of sim time to let physics settle.
            if now - self._start_time < 0.5:
                # Constantly update the start position during the grace period 
                # so we measure the 60m goal from where it actually stabilizes.
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
                
                
    def _imu_cb(self, msg: 'Imu'):
        if self.done.is_set():
            return

        # ── Startup stabilization guard ───────────────────────────────────────
        # Gazebo emits large transient spikes in the first few seconds as the
        # drone settles onto its spawn pose (especially with spawn_z > 0).
        # Ignore IMU data until the sim has been running for IMU_STABILIZATION_S.
        with self._lock:
            if self._start_time is None:
                return  # odom hasn't arrived yet — sim not truly started
            if self.latest_time is None:
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

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def subscribe(self,
                  odom_topic: str = "/rmf_owl/odom",
                  imu_topic:  str = "/rmf_owl/imu"):
        self._odom_sub = rospy.Subscriber(odom_topic, Odometry, self._odom_cb, queue_size=10)
        self._imu_sub  = rospy.Subscriber(imu_topic,  Imu,      self._imu_cb,  queue_size=10)

    def unsubscribe(self):
        if self._odom_sub:
            self._odom_sub.unregister()
        if self._imu_sub:
            self._imu_sub.unregister()


# ── Top-level single-run function ─────────────────────────────────────────────

def run_single(
    controller_script: str,
    world: str           = "random3d",
    headless: bool       = True,
    robot_name: str      = "rmf_owl",
    spawn_x: float       = 0.0,
    spawn_y: float       = 0.0,
    spawn_z: float       = 1.0,
    run_timeout_s: float = 300.0,
    odom_topic: str      = "/rmf_owl/odom",
    imu_topic:  str      = "/rmf_owl/imu",
) -> RunResult:
    """
    Checks for roscore, starts the controller, launches simulation,
    monitors until a termination condition fires, then cleans up.
    """
    global processes, _shutting_down
    processes      = []
    _shutting_down = False

    ros_env    = build_env()
    ros_ws_env = build_env("devel/setup.bash")

    result = RunResult()

    try:
        # 1. Check for external roscore ───────────────────────────────────────
        print("  Checking for an active roscore instance...")
        deadline = time.time() + 10.0
        while not roscore_ready():
            if time.time() > deadline:
                print("  ERROR: No active roscore detected on localhost:11311.")
                print("         Please start 'roscore' in a separate terminal before running this script.")
                cleanup()
                result.exit_code = ExitCode.ERROR
                return result
            time.sleep(0.5)
        print("  External roscore detected.")

        # 2. ROS node for monitoring ──────────────────────────────────────────
        rospy.init_node("sim_monitor", anonymous=True, disable_signals=True)
        monitor = RunMonitor()
        monitor.subscribe(odom_topic=odom_topic, imu_topic=imu_topic)

        # 3. Controller ───────────────────────────────────────────────────────
        print("  Starting controller ahead of simulation...")
        start_process(
            ["bash", "-c",
             f"source /opt/ros/noetic/setup.bash && "
             f"source devel/setup.bash && "
             f"python3 {controller_script}"],
            name="controller",
            env=ros_ws_env,
        )

        # Wait dynamically for the controller to register its ROS subscribers
        # rather than using a fixed sleep — more reliable on loaded machines.
        print("  Waiting for controller to register with ROS master...")
        if not wait_for_controller_subscription(odom_topic=odom_topic, timeout=15.0):
            print("  [!] WARNING: Controller did not subscribe to odometry within 15s. Launching sim anyway...")
        else:
            # Short grace period for the controller to finish setting up any
            # remaining publishers (e.g. cmd_vel) after its odom subscriber fires.
            time.sleep(0.5)

        # 4. Simulation launch ────────────────────────────────────────────────
        print("  Launching Gazebo (auto-running)...")
        headless_str = "true" if headless else "false"
        launch_cmd = (
            "source /opt/ros/noetic/setup.bash && "
            "source devel/setup.bash && "
            f"roslaunch lmf_sim start_sim.launch "
            f"world:={world} headless:={headless_str} "
            f"robot_name:={robot_name} "
            f"x:={spawn_x} y:={spawn_y} z:={spawn_z}"
        )
        start_process(["bash", "-c", launch_cmd], name="lmf_sim", env=ros_ws_env)

        # 5. Wait for termination ─────────────────────────────────────────────
        print(f"  Monitoring... (hard timeout {run_timeout_s:.0f} s sim time)")

        # Real-world safety fallback in case Gazebo crashes or runs extremely
        # slowly — prevents the script from hanging forever.
        wall_start   = time.time()
        wall_timeout = (run_timeout_s * 5.0) + 30.0

        while not monitor.done.is_set():
            # 1. Check for real-world hang
            if time.time() - wall_start > wall_timeout:
                print("  [!] Wall-clock safety timeout triggered (Gazebo might have crashed).")
                break

            # 2. Hard sim-time timeout — compare sim timestamps only, never
            #    rospy.get_time(), to avoid wall-clock vs sim-clock mismatch.
            if monitor._start_time is not None and monitor.latest_time is not None:
                sim_elapsed = monitor.latest_time - monitor._start_time
                if sim_elapsed >= run_timeout_s:
                    monitor.result.exit_code  = ExitCode.TIMEOUT
                    monitor.result.timeout    = True
                    monitor.result.duration_s = sim_elapsed
                    monitor.done.set()
                    break

            monitor.done.wait(timeout=0.2)

        monitor.unsubscribe()

        if not monitor.done.is_set():
            result.exit_code = ExitCode.ERROR
        else:
            result = monitor.result

    finally:
        cleanup()

    return result


# ── CLI entry-point (single run) ──────────────────────────────────────────────

def _parse_args():
    p = argparse.ArgumentParser(description="Run a single simulation trial.")
    p.add_argument("controller", help="Path to the controller Python script")
    p.add_argument("--world",    default="random3d")
    p.add_argument("--headless", action="store_true")
    p.add_argument("--timeout",  type=float, default=300.0)
    return p.parse_args()


if __name__ == "__main__":
    if not ROS_AVAILABLE:
        print("ERROR: ROS (rospy) is not available. Source your ROS workspace first.")
        sys.exit(3)

    args = _parse_args()
    r = run_single(
        controller_script=args.controller,
        world=args.world,
        headless=args.headless,
        run_timeout_s=args.timeout,
    )
    print(f"\nResult: {r.exit_code.name}  |  max_x={r.max_x:.2f} m  "
          f"|  mean_v={r.mean_velocity:.2f} m/s  |  max_v={r.max_velocity:.2f} m/s  "
          f"|  duration={r.duration_s:.1f} s")
    sys.exit(int(r.exit_code))