#!/usr/bin/env python3
"""
Apply GUI-only view settings once the Gazebo GUI scene is populated:
camera follows the robot, wireframe obstacles and a transparent ceiling.

Follow is set via /gui/track: in Harmonic, <follow_target> in the GUI config
stores the target but never enables follow mode.

The /gui/view/* services only affect the GUI's render scene (not the sensor
scene), are silently dropped if the target isn't loaded yet, and toggle on
repeated calls, so we wait for the scene and call each exactly once.
"""
import argparse
import subprocess
import sys
import time


def gz_service_listed(name: str) -> bool:
    out = subprocess.run(["gz", "service", "-l"], capture_output=True, text=True).stdout
    return name in out.split()


def wait_for_message(topic: str, timeout: float) -> bool:
    try:
        subprocess.run(["gz", "topic", "-e", "-n", "1", "-t", topic],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=timeout)
        return True
    except subprocess.TimeoutExpired:
        return False


def call_view(service: str, entity: str) -> None:
    subprocess.run(["gz", "service", "-s", service,
                    "--reqtype", "gz.msgs.StringMsg", "--reptype", "gz.msgs.Boolean",
                    "--timeout", "2000", "--req", f'data: "{entity}"'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def follow(entity: str, offset, pgain: float) -> None:
    # track_mode 2 = FOLLOW (gz.msgs.CameraTrack)
    req = (f'track_mode: 2, follow_target: {{name: "{entity}"}}, '
           f'follow_offset: {{x: {offset[0]}, y: {offset[1]}, z: {offset[2]}}}, '
           f'follow_pgain: {pgain}')
    # /gui/track is a topic (not a service) in Harmonic
    subprocess.run(["gz", "topic", "-t", "/gui/track", "-m", "gz.msgs.CameraTrack", "-p", req],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--robot", default="rmf")
    p.add_argument("--wireframe", nargs="*", default=["random_spheres"])
    p.add_argument("--transparent", nargs="*", default=["ceil"])
    p.add_argument("--follow_offset", type=float, nargs=3, default=[-4.0, 0.0, 2.0])
    p.add_argument("--follow_pgain", type=float, default=0.05)
    p.add_argument("--timeout", type=float, default=120.0)
    p.add_argument("--settle", type=float, default=3.0, help="extra wait for the GUI scene to load")
    args, _ = p.parse_known_args()  # ignore ROS launch args

    deadline = time.time() + args.timeout
    while not gz_service_listed("/gui/view/wireframes"):
        if time.time() > deadline:
            print("[gui_view_setup] GUI not found, skipping view settings")
            sys.exit(0)
        time.sleep(0.5)

    # robot publishing odometry => spawned; the GUI scene follows shortly after
    if not wait_for_message(f"/model/{args.robot}/odometry", max(1.0, deadline - time.time())):
        print("[gui_view_setup] robot never spawned, skipping view settings")
        sys.exit(0)
    time.sleep(args.settle)

    for entity in args.wireframe:
        call_view("/gui/view/wireframes", entity)
    for entity in args.transparent:
        call_view("/gui/view/transparent", entity)

    # unlike the view toggles, follow is idempotent: repeat in case the robot
    # wasn't in the GUI scene yet (a missing target is silently dropped)
    for _ in range(3):
        follow(args.robot, args.follow_offset, args.follow_pgain)
        time.sleep(2.0)
    print("[gui_view_setup] view settings applied")
