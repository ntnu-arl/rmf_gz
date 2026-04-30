#!/usr/bin/env python3
"""
sim_runner.py  —  Single simulation run with termination monitoring.

Termination conditions:
  - SUCCESS  : drone travels > 60 m in the x-direction from its spawn pose
  - COLLISION: IMU sensor fires massive spike (collision with environment)
  - TIMEOUT  : less than 3 m x-progress in the last 15 s

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
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from geometry_msgs.msg import Point
    # Replaced ContactsState with Imu
    from sensor_msgs.msg import Imu
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


class ExitCode(IntEnum):
    SUCCESS   = 0
    COLLISION = 1
    TIMEOUT   = 2
    ERROR     = 3


# ── Process management (same pattern as your existing launcher) ───────────────

processes: List[Tuple[subprocess.Popen, str]] = []
_shutting_down = False


def cleanup():
    global _shutting_down
    if _shutting_down:
        return
    _shutting_down = True
    for proc, name in reversed(processes):
        if proc.poll() is not None:
            continue
        pgid = None
        try:
            pgid = os.getpgid(proc.pid)
        except ProcessLookupError:
            pass
        try:
            if pgid is not None:
                os.killpg(pgid, signal.SIGTERM)
            else:
                proc.terminate()
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            try:
                if pgid is not None:
                    os.killpg(pgid, signal.SIGKILL)
                else:
                    proc.kill()
            except ProcessLookupError:
                pass
            proc.wait()


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


# ── Per-run telemetry ─────────────────────────────────────────────────────────

@dataclass
class RunResult:
    exit_code:      ExitCode = ExitCode.ERROR
    max_x:          float    = 0.0          # maximum x reached (m)
    velocities:     list     = field(default_factory=list)  # |v| samples (m/s)
    duration_s:     float    = 0.0          # wall-clock seconds
    collision:      bool     = False
    timeout:        bool     = False
    success:        bool     = False

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
    """

    SUCCESS_X_DIST      = 60.0   # m
    TIMEOUT_WINDOW      = 15.0   # s
    TIMEOUT_MIN_DX      = 3.0    # m  — must make at least this much progress
    IMU_ACCEL_THRESHOLD = 40.0   # m/s^2 — threshold for detecting a collision spike

    def __init__(self):
        self.result   = RunResult()
        self.done     = threading.Event()
        self._lock    = threading.Lock()

        self._start_x:   Optional[float] = None
        self._start_time: float           = time.time()

        # sliding-window timeout tracking
        self._window_start_x:  float = 0.0
        self._window_start_t:  float = time.time()

        # ROS subscribers (set up after rospy.init_node)
        self._odom_sub:    Optional[rospy.Subscriber] = None
        self._imu_sub:     Optional[rospy.Subscriber] = None

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _odom_cb(self, msg: 'Odometry'):
        if self.done.is_set():
            return

        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        speed = (vel.x**2 + vel.y**2 + vel.z**2) ** 0.5
        now   = time.time()

        with self._lock:
            if self._start_x is None:
                self._start_x         = pos.x
                self._window_start_x  = pos.x
                self._window_start_t  = now

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
        
        # Calculate magnitude of linear acceleration
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        accel_mag = (ax**2 + ay**2 + az**2) ** 0.5

        if accel_mag > self.IMU_ACCEL_THRESHOLD:
            with self._lock:
                self.result.collision  = True
                self.result.exit_code  = ExitCode.COLLISION
                self.result.duration_s = time.time() - self._start_time
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
    odom_topic:    str   = "/rmf_owl/odom",
    imu_topic:     str   = "/rmf_owl/imu",
) -> RunResult:
    """
    Launch roscore + simulation + controller, monitor until a termination
    condition fires or the hard timeout expires, then clean up.

    Returns a RunResult.
    """
    global processes, _shutting_down
    processes      = []
    _shutting_down = False

    ros_env    = build_env()
    ros_ws_env = build_env("devel/setup.bash")

    result = RunResult()

    try:
        # 1. roscore ──────────────────────────────────────────────────────────
        print("  Starting roscore...")
        start_process(
            ["bash", "-c", "source /opt/ros/noetic/setup.bash && roscore"],
            name="roscore", env=ros_env,
        )
        deadline = time.time() + 30
        while not roscore_ready():
            if time.time() > deadline:
                print("  ERROR: roscore did not become ready in 30 s")
                cleanup()
                result.exit_code = ExitCode.ERROR
                return result
            time.sleep(0.3)
        print("  roscore ready.")

        # 2. Simulation launch ────────────────────────────────────────────────
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
        time.sleep(8)   # give Gazebo time to fully initialise

        # 3. ROS node for monitoring ──────────────────────────────────────────
        rospy.init_node("sim_monitor", anonymous=True, disable_signals=True)
        monitor = RunMonitor()
        monitor.subscribe(odom_topic=odom_topic, imu_topic=imu_topic)

        # 4. Controller ───────────────────────────────────────────────────────
        start_process(
            ["bash", "-c",
             f"source /opt/ros/noetic/setup.bash && "
             f"source devel/setup.bash && "
             f"python3 {controller_script}"],
            name="controller", env=ros_ws_env,
        )

        # 5. Wait for termination ─────────────────────────────────────────────
        print(f"  Monitoring... (hard timeout {run_timeout_s:.0f} s)")
        monitor.done.wait(timeout=run_timeout_s)
        monitor.unsubscribe()

        if not monitor.done.is_set():
            # Hard timeout expired (shouldn't happen under normal conditions)
            result.exit_code  = ExitCode.TIMEOUT
            result.timeout    = True
            result.duration_s = run_timeout_s
            result.max_x      = monitor.result.max_x
            result.velocities = monitor.result.velocities
        else:
            result = monitor.result

    finally:
        cleanup()
        # Give ROS master a moment to release port 11311
        time.sleep(3)

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