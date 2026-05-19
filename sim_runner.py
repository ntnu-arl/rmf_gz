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
  3  = ERROR (launch failure)
"""

import subprocess
import time
import sys
import os
import signal
import threading
import argparse
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

    # 1. Ask processes to shut down gracefully
    for proc, name in reversed(processes):
        if proc.poll() is not None:
            continue
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGINT)
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

    # 3. AGGRESSIVE CLEANUP: Nuke orphaned ROS2/Gazebo processes
    subprocess.run(
        "killall -9 ros2 ruby gz ign 2>/dev/null; pkill -9 -f 'gz sim' 2>/dev/null",
        shell=True,
        stderr=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
    )

    # Brief pause to let the OS release the ports and memory
    time.sleep(3)


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


class RunMonitor(Node):
    """
    Subscribes to odometry and IMU topics; evaluates termination conditions.
    Thread-safe: the main loop polls `self.done` and `self.result`.
    """

    SUCCESS_X_DIST       = 30.0   # m
    TIMEOUT_WINDOW       = 5.0    # s  (sim time)
    TIMEOUT_MIN_DX       = 1.0    # m  — must make at least this much x-progress per window
    IMU_ACCEL_THRESHOLD  = 40.0   # m/s² — spike magnitude that signals a collision
    IMU_STABILIZATION_S  = 3.0    # s  (sim time) — ignore IMU spikes during startup transient

    def __init__(self):
        super().__init__('sim_monitor')
        self.result  = RunResult()
        self.done    = threading.Event()
        self._lock   = threading.Lock()

        self._start_x:    Optional[float] = None
        self._start_time: Optional[float] = None
        self.latest_time: Optional[float] = None

        self._window_start_x: float = 0.0
        self._window_start_t: float = 0.0

        # Command tracking to prevent premature inactivity timeouts
        self._control_active = False

        self._odom_sub = None
        self._imu_sub  = None
        self._cmd_sub  = None

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _cmd_cb(self, msg: Twist):
        # Triggers the first time the control node publishes an action
        if not self._control_active:
            self._control_active = True
            print("  [+] First command received! Progress timeout tracking started.")

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
            if self._start_x is None:
                self._start_x         = pos.x
                self._start_time      = now
                self._window_start_x  = pos.x
                self._window_start_t  = now
                return

            # If the controller is still warming up/stabilizing, slide the progress window 
            # forward so we don't trigger the 5-second inactivity timeout.
            # (Note: We do NOT reset self._start_time, so the global 300s timeout still protects us)
            if not self._control_active:
                self._window_start_x  = pos.x
                self._window_start_t  = now
                return

            # ── GAZEBO GRACE PERIOD ───────────────────────────────────────────
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
            # ─────────────────────────────────────────────────────────────
            # CRITICAL FIX: Ignore all physics/spawn glitches until the 
            # high-level controller has actually started the mission.
            # ─────────────────────────────────────────────────────────────
            if not self._control_active:
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

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def subscribe(self,
                  odom_topic: str = "/rmf/odom",
                  imu_topic:  str = "/rmf/imu",
                  cmd_topic:  str = "/rmf/cmd/acc"):
        
        # Applied QoS profiles to match best-effort sensor data
        self._odom_sub = self.create_subscription(Odometry, odom_topic, self._odom_cb, qos_profile_sensor_data)
        self._imu_sub  = self.create_subscription(Imu, imu_topic, self._imu_cb, qos_profile_sensor_data)
        
        # Command subscriber to unlock the progress timer
        self._cmd_sub  = self.create_subscription(Twist, cmd_topic, self._cmd_cb, 10)

    def unsubscribe(self):
        if self._odom_sub:
            self.destroy_subscription(self._odom_sub)
        if self._imu_sub:
            self.destroy_subscription(self._imu_sub)
        if self._cmd_sub:
            self.destroy_subscription(self._cmd_sub)


# ── Top-level single-run function ─────────────────────────────────────────────

def run_single(
    controller_script: str,
    world: str           = "pillar",
    headless: bool       = True,
    robot_name: str      = "rmf",
    spawn_x: float       = 0.0,
    spawn_y: float       = 0.0,
    spawn_z: float       = 1.0,
    run_timeout_s: float = 300.0,
    odom_topic: str      = "/rmf/odom",
    imu_topic:  str      = "/rmf/imu",
    cmd_topic:  str      = "/rmf/cmd/acc",
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

    result = RunResult()
    
    # Pre-declare variables for safe teardown in the finally block
    monitor = None
    executor = None
    spin_thread = None

    try:
        # 1. Start ROS2 Monitor Node ──────────────────────────────────────────
        rclpy.init()
        monitor = RunMonitor()
        monitor.subscribe(odom_topic=odom_topic, imu_topic=imu_topic, cmd_topic=cmd_topic)
        
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
             f"python3 {controller_script}"],
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
        print("ERROR: ROS2 (rclpy) is not available. Source your ROS workspace first.")
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