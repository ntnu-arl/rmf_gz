#!/usr/bin/env python3
"""
Automatically launch roscore and roslaunch lmf_sim start_sim.launch.
The script first starts roscore, waits for it to be ready,
then starts the simulation launch.
Press Ctrl+C to cleanly shut down all child processes and exit.
"""
import subprocess
import time
import sys
import os
import signal
import socket
from typing import List, Tuple

# ----------------------------------------------------------------------
# Global list of (process, name) tuples
# ----------------------------------------------------------------------
processes: List[Tuple[subprocess.Popen, str]] = []
_shutting_down = False


def cleanup():
    """
    Kill every tracked process and its entire process group.
    Safe to call multiple times.
    """
    global _shutting_down
    if _shutting_down:
        return
    _shutting_down = True

    print("\nShutting down all child processes...")
    for proc, name in reversed(processes):          # reverse: kill children before parents
        if proc.poll() is not None:
            continue                                 # already dead
        pgid = None
        try:
            pgid = os.getpgid(proc.pid)
        except ProcessLookupError:
            pass

        print(f"  Terminating '{name}' (PID {proc.pid}, PGID {pgid})...")
        try:
            if pgid is not None:
                os.killpg(pgid, signal.SIGTERM)     # ask the whole group nicely
            else:
                proc.terminate()
        except ProcessLookupError:
            pass

        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            print(f"  '{name}' did not exit — sending SIGKILL...")
            try:
                if pgid is not None:
                    os.killpg(pgid, signal.SIGKILL)
                else:
                    proc.kill()
            except ProcessLookupError:
                pass
            proc.wait()

    print("All processes terminated.")


def signal_handler(sig, frame):
    cleanup()
    sys.exit(0)


# Register signal handlers (SIGINT = Ctrl+C, SIGTERM = kill <pid>)
signal.signal(signal.SIGINT,  signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# atexit is a safety net in case neither signal fires
import atexit
atexit.register(cleanup)

# ----------------------------------------------------------------------

def start_process(cmd: List[str], name: str, env=None) -> subprocess.Popen:
    """
    Launch a command in its own process group so we can kill the whole
    group (process + any children it spawns) cleanly.
    """
    proc = subprocess.Popen(
        cmd,
        env=env,
        # Each child gets its own process group → we can os.killpg() it
        start_new_session=True,
    )
    processes.append((proc, name))
    print(f"  Started '{name}' (PID {proc.pid})")
    return proc


def build_env(extra_source: str = None) -> dict:
    """
    Return an environment dict that has ROS sourced in.
    Optionally source an additional setup file (e.g. workspace devel/setup.bash).
    """
    # We need bash to run 'source', so we ask bash to print the resulting env.
    source_cmds = "source /opt/ros/noetic/setup.bash 2>/dev/null"
    if extra_source:
        source_cmds += f" && source {extra_source} 2>/dev/null"
    source_cmds += " && env"

    result = subprocess.run(
        ["bash", "-c", source_cmds],
        capture_output=True, text=True
    )
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


def main():
    ros_env        = build_env()
    ros_ws_env     = build_env("devel/setup.bash")   # workspace overlay

    # 1. Start roscore
    print("Launching roscore...")
    roscore_proc = start_process(
        ["bash", "-c", "source /opt/ros/noetic/setup.bash && roscore"],
        name="roscore",
        env=ros_env,
    )

    # 2. Wait until roscore is ready
    print("Waiting for roscore to become ready (up to 30 s)...")
    deadline = time.time() + 30
    while not roscore_ready():
        if roscore_proc.poll() is not None:
            print("roscore exited unexpectedly. Aborting.")
            cleanup()
            sys.exit(1)
        if time.time() > deadline:
            print("Timeout waiting for roscore. Aborting.")
            cleanup()
            sys.exit(1)
        time.sleep(0.5)
    print("roscore is up!")

    # 3. Start the simulation
    print("Launching lmf_sim...")
    start_process(
        ["bash", "-c",
         "source /opt/ros/noetic/setup.bash && "
         "source devel/setup.bash && "
         "roslaunch lmf_sim start_sim.launch"],
        name="lmf_sim",
        env=ros_ws_env,
    )

    print("\nBoth processes are running. Press Ctrl+C to shut everything down.")
    try:
        while True:
            # If either child dies on its own, report it and exit cleanly
            for proc, name in processes:
                if proc.poll() is not None:
                    print(f"\nProcess '{name}' exited with code {proc.returncode}.")
                    cleanup()
                    sys.exit(proc.returncode)
            time.sleep(1)
    except KeyboardInterrupt:
        pass   # signal_handler already called cleanup()


if __name__ == "__main__":
    main()