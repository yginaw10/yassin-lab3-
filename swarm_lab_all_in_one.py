#!/usr/bin/env python3
"""
swarm_lab_all_in_one.py
========================
One file. One command. Runs everything.

Usage:
    python3 swarm_lab_all_in_one.py

To kill everything:
    pkill -9 -f "gazebo|px4|mavros|swarm|yolo|gzserver|gzclient|gz_bridge"
"""

import os
import subprocess
import time

# ── Configuration ─────────────────────────────────────────────────────────────
ROS_SETUP  = "/opt/ros/humble/setup.bash"
SWARM_LAB  = os.path.expanduser("~/swarm_lab")
YOLO_MODEL = os.path.join(SWARM_LAB, "WSL_Assets/yolo11n-obb.pt")
PLUGIN_DIR = os.path.join(SWARM_LAB, "WSL_Assets/plugins")

# ── Source ROS + set env vars ──────────────────────────────────────────────────
def source_env():
    cmd = f"bash -c 'source {ROS_SETUP} && env'"
    proc = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    env = {}
    for line in proc.stdout.splitlines():
        if "=" in line:
            k, _, v = line.partition("=")
            env[k] = v
    if not env:
        print("[WARN] Could not source ROS — using current environment.")
        env = os.environ.copy()
    env["YOLO_MODEL"]          = YOLO_MODEL
    env["YOLO_AUTOINSTALL"]    = "False"
    env["ULTRALYTICS_OFFLINE"] = "True"
    env["GAZEBO_PLUGIN_PATH"]  = PLUGIN_DIR + ":" + env.get("GAZEBO_PLUGIN_PATH", "")
    # Add ROS humble lib to LD_LIBRARY_PATH for libgazebo_ros_camera.so
    env["LD_LIBRARY_PATH"]     = "/opt/ros/humble/lib:" + env.get("LD_LIBRARY_PATH", "")
    return env


# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    print("\n╔══════════════════════════════════════════╗")
    print("║   Swarm Lab — All-In-One Auto Launch     ║")
    print("║                                          ║")
    print("║  Step 1 — Gazebo + PX4 + MAVROS          ║")
    print("║  Step 2 — Camera Bridge                  ║")
    print("║  Step 3 — Swarm Mission                  ║")
    print("║  Step 4 — YOLO Camera Viewer             ║")
    print("╚══════════════════════════════════════════╝\n")

    env = source_env()

    # ── Step 1: Launch Gazebo + PX4 + MAVROS ──────────────────────────────────
    print("[1/4] Launching Gazebo / PX4 / MAVROS...")
    launch = subprocess.Popen(
        ["bash", os.path.join(SWARM_LAB, "swarm_lab_launch.sh")],
        env=env, cwd=SWARM_LAB)
    print(f"      PID: {launch.pid}")
    print("      Waiting 90s for full initialisation...")
    for i in range(90):
        time.sleep(1)
        print(f"      {i+1}/90s", end="\r")
    print("\n      [OK] Gazebo ready.")

    # ── Step 2: Camera bridge ──────────────────────────────────────────────────
    print("\n[2/4] Starting Gazebo->ROS2 camera bridge...")
    bridge_cmd = (
        f"source {ROS_SETUP} && "
        "export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH && "
        "ros2 run ros_gz_bridge parameter_bridge "
        "'/gazebo/baylands_swarm/nadir_cam_uav0/cam_link/camera/image"
        "@sensor_msgs/msg/Image@gz.msgs.Image' "
        "'/gazebo/baylands_swarm/nadir_cam_uav1/cam_link/camera/image"
        "@sensor_msgs/msg/Image@gz.msgs.Image' "
        "--ros-args "
        "-r /gazebo/baylands_swarm/nadir_cam_uav0/cam_link/camera/image:=/uav0/drone_cam/image_raw "
        "-r /gazebo/baylands_swarm/nadir_cam_uav1/cam_link/camera/image:=/uav1/drone_cam/image_raw"
    )
    bridge = subprocess.Popen(
        ["bash", "-c", bridge_cmd],
        env=env, cwd=SWARM_LAB
    )
    print(f"      PID: {bridge.pid}")
    time.sleep(5)
    print("      [OK] Camera bridge running.")

    # ── Step 3: Swarm mission ──────────────────────────────────────────────────
    print("\n[3/4] Starting swarm mission...")
    mission = subprocess.Popen(
        ["python3", os.path.join(SWARM_LAB, "swarm_mission.py")],
        env=env, cwd=SWARM_LAB)
    print(f"      PID: {mission.pid}")
    time.sleep(5)

    # ── Step 4: YOLO camera viewer ─────────────────────────────────────────────
    print("\n[4/4] Launching YOLO camera viewer...")
    viewer = subprocess.Popen(
        ["python3", os.path.join(SWARM_LAB, "yolo_camera_viewer.py")],
        env=env, cwd=SWARM_LAB)
    print(f"      PID: {viewer.pid}")

    print("\n╔══════════════════════════════════════════╗")
    print("║  All processes running!                  ║")
    print("║                                          ║")
    print("║  Camera feeds:                           ║")
    print("║  /uav0/drone_cam/image_raw  (narrow)     ║")
    print("║  /uav1/drone_cam/image_raw  (wide)       ║")
    print("║                                          ║")
    print("║  Press Ctrl+C to stop everything.        ║")
    print("╚══════════════════════════════════════════╝\n")

    try:
        while True:
            time.sleep(5)
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down all processes...")
        for p in [viewer, mission, bridge, launch]:
            if p and p.poll() is None:
                p.terminate()
                try:
                    p.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    p.kill()
        print("[INFO] Done. Goodbye!")

if __name__ == "__main__":
    main()
