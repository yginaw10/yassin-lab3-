#!/usr/bin/env python3
"""
run_lab3.py
===========
One script to set up and launch the entire Lab 3 pipeline.

Usage (with ROS 2 Humble already installed):
    python3 run_lab3.py

What it does:
  1. Writes all assurance_harness package files
  2. Builds the ROS 2 workspace with colcon
  3. Opens Terminal 1 → PX4 + Gazebo
  4. Waits 15s, then opens Terminal 2 → MAVROS
  5. Waits 10s, then opens Terminal 3 → Assurance Harness
  6. Waits 10s, then opens Terminal 4 → Verification commands
  7. Waits 10s, then opens Terminal 5 → gnuplot

Requirements:
  - ROS 2 Humble installed
  - PX4-Autopilot cloned at ~/PX4-Autopilot
  - MAVROS installed (ros-humble-mavros)
  - gnome-terminal (default on Ubuntu)
"""

import os
import sys
import time
import subprocess
import textwrap
from pathlib import Path

# ─────────────────────────────────────────────────────────────
#  ALL PACKAGE FILE CONTENTS
# ─────────────────────────────────────────────────────────────

SETUP_PY = '''\
from setuptools import setup
import os
from glob import glob

package_name = 'assurance_harness'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS 2 assurance harness: risk proxy + evidence logging + RViz marker',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telemetry_gate_node = assurance_harness.telemetry_gate_node:main',
            'risk_model_node = assurance_harness.risk_model_node:main',
            'evidence_logger_node = assurance_harness.evidence_logger_node:main',
            'viz_node = assurance_harness.viz_node:main',
            'fault_injector_node = assurance_harness.fault_injector_node:main',
        ],
    },
)
'''

PACKAGE_XML = '''\
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>assurance_harness</name>
  <version>0.0.1</version>
  <description>ROS 2 assurance harness</description>
  <maintainer email="your_email@example.com">your_name</maintainer>
  <license>Apache-2.0</license>
  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>std_msgs</depend>
  <depend>visualization_msgs</depend>
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
'''

DSM_PY = '''\
import numpy as np

def ground_height(x: float, y: float) -> float:
    return 2.0 + 1.5 * np.exp(-0.01 * (x ** 2 + y ** 2))

def line_of_sight_clear(drone_pos, gcs_pos, steps: int = 50) -> bool:
    x0, y0, z0 = gcs_pos
    x1, y1, z1 = drone_pos
    for i in range(1, steps + 1):
        t = i / steps
        x = x0 + t * (x1 - x0)
        y = y0 + t * (y1 - y0)
        z = z0 + t * (z1 - z0)
        if z <= ground_height(x, y):
            return False
    return True
'''

TELEMETRY_GATE_NODE_PY = '''\
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class TelemetryGate(Node):
    def __init__(self):
        super().__init__("telemetry_gate")
        self.declare_parameter("input_pose_topic", "/mavros/local_position/pose")
        input_topic = (
            self.get_parameter("input_pose_topic").get_parameter_value().string_value
        )
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self.sub = self.create_subscription(PoseStamped, input_topic, self.cb, qos)
        self.pub = self.create_publisher(PoseStamped, "/assurance/pose", 10)
        self.get_logger().info(f"TelemetryGate subscribing to: {input_topic}")

    def cb(self, msg: PoseStamped):
        gated = PoseStamped()
        gated.header = msg.header
        gated.pose = msg.pose
        self.pub.publish(gated)

def main():
    rclpy.init()
    node = TelemetryGate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
'''

RISK_MODEL_NODE_PY = '''\
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from assurance_harness.dsm import ground_height, line_of_sight_clear

class RiskModel(Node):
    def __init__(self):
        super().__init__("risk_model")
        self.sub = self.create_subscription(PoseStamped, "/assurance/pose", self.cb, 10)
        self.pub = self.create_publisher(Float32, "/assurance/risk", 10)
        self.declare_parameter("gcs_x", 0.0)
        self.declare_parameter("gcs_y", 0.0)
        self.declare_parameter("gcs_z", 2.0)
        self.get_logger().info("RiskModel publishing /assurance/risk (Float32: 0.0, 0.5, 1.0)")

    def cb(self, msg: PoseStamped):
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        z = float(msg.pose.position.z)
        gcs = (
            self.get_parameter("gcs_x").value,
            self.get_parameter("gcs_y").value,
            self.get_parameter("gcs_z").value,
        )
        agl = z - ground_height(x, y)
        los = line_of_sight_clear((x, y, z), gcs)
        if agl < 15.0 and (not los):
            risk = 1.0
        elif agl < 15.0:
            risk = 0.5
        else:
            risk = 0.0
        self.pub.publish(Float32(data=risk))

def main():
    rclpy.init()
    node = RiskModel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
'''

EVIDENCE_LOGGER_NODE_PY = '''\
import rclpy
import csv
import time
from pathlib import Path
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32

class EvidenceLogger(Node):
    def __init__(self):
        super().__init__("evidence_logger")
        self.pose = None
        self.risk = None
        out_path = Path("evidence.csv")
        self.file = out_path.open("w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["t_unix", "x", "y", "z", "risk"])
        self.create_subscription(PoseStamped, "/assurance/pose", self.pose_cb, 10)
        self.create_subscription(Float32, "/assurance/risk", self.risk_cb, 10)
        self.timer = self.create_timer(0.5, self.tick)
        self.get_logger().info(f"EvidenceLogger writing: {out_path.resolve()}")

    def pose_cb(self, msg: PoseStamped):
        self.pose = msg

    def risk_cb(self, msg: Float32):
        self.risk = float(msg.data)

    def tick(self):
        if (self.pose is None) or (self.risk is None):
            return
        p = self.pose.pose.position
        self.writer.writerow([time.time(), float(p.x), float(p.y), float(p.z), self.risk])
        self.file.flush()

    def destroy_node(self):
        try:
            self.file.flush()
            self.file.close()
        except Exception:
            pass
        super().destroy_node()

def main():
    rclpy.init()
    node = EvidenceLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
'''

VIZ_NODE_PY = '''\
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Float32

class RiskViz(Node):
    def __init__(self):
        super().__init__("risk_viz")
        self.risk = 0.0
        self.points = []
        self.create_subscription(Float32, "/assurance/risk", self.risk_cb, 10)
        self.create_subscription(PoseStamped, "/assurance/pose", self.pose_cb, 10)
        self.pub = self.create_publisher(Marker, "/assurance/route", 10)
        self.declare_parameter("frame_id", "map")
        self.get_logger().info("RiskViz publishing /assurance/route (visualization_msgs/Marker)")

    def risk_cb(self, msg: Float32):
        self.risk = float(msg.data)

    def pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self.points.append((float(p.x), float(p.y), float(p.z), self.risk))
        m = Marker()
        m.header = msg.header
        m.header.frame_id = self.get_parameter("frame_id").value
        m.ns = "assurance_route"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.2
        for x, y, z, _r in self.points:
            m.points.append(Point(x=x, y=y, z=z))
        m.color.a = 1.0
        if self.risk > 0.8:
            m.color.r = 1.0
        elif self.risk > 0.3:
            m.color.g = 1.0
        else:
            m.color.b = 1.0
        self.pub.publish(m)

def main():
    rclpy.init()
    node = RiskViz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
'''

FAULT_INJECTOR_NODE_PY = '''\
import rclpy
import random
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class FaultInjector(Node):
    def __init__(self):
        super().__init__("fault_injector")
        self.declare_parameter("drop_rate", 0.3)
        self.declare_parameter("input_pose_topic", "/mavros/local_position/pose")
        self.declare_parameter("output_pose_topic", "/assurance/pose_injected")
        self.drop_rate = float(self.get_parameter("drop_rate").value)
        inp = self.get_parameter("input_pose_topic").value
        out = self.get_parameter("output_pose_topic").value
        self.sub = self.create_subscription(PoseStamped, inp, self.cb, 10)
        self.pub = self.create_publisher(PoseStamped, out, 10)
        self.get_logger().info(f"FaultInjector {inp} -> {out}, drop_rate={self.drop_rate}")

    def cb(self, msg: PoseStamped):
        if random.random() > self.drop_rate:
            self.pub.publish(msg)

def main():
    rclpy.init()
    node = FaultInjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
'''

ASSURANCE_LAUNCH_PY = '''\
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="assurance_harness",
            executable="telemetry_gate_node",
            name="telemetry_gate",
            parameters=[{"input_pose_topic": "/mavros/local_position/pose"}],
        ),
        Node(package="assurance_harness", executable="risk_model_node", name="risk_model"),
        Node(package="assurance_harness", executable="evidence_logger_node", name="evidence_logger"),
        Node(package="assurance_harness", executable="viz_node", name="viz"),
    ])
'''

ASSURANCE_FAULTED_LAUNCH_PY = '''\
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="assurance_harness",
            executable="fault_injector_node",
            name="fault_injector",
            parameters=[
                {"drop_rate": 0.3},
                {"input_pose_topic": "/mavros/local_position/pose"},
                {"output_pose_topic": "/assurance/pose_injected"},
            ],
        ),
        Node(
            package="assurance_harness",
            executable="telemetry_gate_node",
            name="telemetry_gate",
            parameters=[{"input_pose_topic": "/assurance/pose_injected"}],
        ),
        Node(package="assurance_harness", executable="risk_model_node", name="risk_model"),
        Node(package="assurance_harness", executable="evidence_logger_node", name="evidence_logger"),
        Node(package="assurance_harness", executable="viz_node", name="viz"),
    ])
'''

# ─────────────────────────────────────────────────────────────
#  TERMINAL COMMANDS
# ─────────────────────────────────────────────────────────────

CMD_T1_PX4 = """\
bash -c '
echo "=== TERMINAL 1: PX4 + Gazebo ===";
cd ~/PX4-Autopilot && make px4_sitl gazebo;
exec bash'
"""

CMD_T2_MAVROS = """\
bash -c '
echo "=== TERMINAL 2: MAVROS ===";
source /opt/ros/humble/setup.bash;
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557";
exec bash'
"""

CMD_T3_HARNESS = """\
bash -c '
echo "=== TERMINAL 3: Assurance Harness ===";
source /opt/ros/humble/setup.bash;
source ~/ros2_ws/install/setup.bash;
cd ~/ros2_ws;
ros2 launch assurance_harness assurance.launch.py;
exec bash'
"""

CMD_T4_VERIFY = """\
bash -c '
echo "=== TERMINAL 4: Verification ===";
source /opt/ros/humble/setup.bash;
source ~/ros2_ws/install/setup.bash;
echo "--- Waiting 5s for harness to start ---";
sleep 5;
echo "";
echo "--- /assurance/pose ---";
ros2 topic echo /assurance/pose --once;
echo "";
echo "--- /assurance/risk ---";
ros2 topic echo /assurance/risk --once;
echo "";
echo "--- Topic rate (hz) - press Ctrl+C to stop ---";
ros2 topic hz /assurance/risk;
exec bash'
"""

CMD_T5_GNUPLOT = """\
bash -c '
echo "=== TERMINAL 5: gnuplot ===";
echo "--- Installing gnuplot if needed ---";
sudo apt install gnuplot -y;
cd ~/ros2_ws;
echo "--- Waiting for evidence.csv to have data ---";
sleep 5;
echo "--- Last 5 lines of evidence.csv ---";
tail -n 5 evidence.csv;
echo "";
echo "--- Opening gnuplot ---";
echo "--- Run inside gnuplot: ---";
echo "    set datafile separator \",\"";
echo "    plot \\"evidence.csv\\" using 4:5 with points title \\"Risk vs Height\\"";
echo "";
gnuplot;
exec bash'
"""

# ─────────────────────────────────────────────────────────────
#  HELPERS
# ─────────────────────────────────────────────────────────────

def banner(msg: str):
    print("\n" + "─" * 60)
    print(f"  {msg}")
    print("─" * 60)


def write_file(path: Path, content: str):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)
    print(f"  [wrote]  {path}")


def open_terminal(title: str, command: str, wait: int = 0):
    """Open a new gnome-terminal tab with the given command."""
    if wait > 0:
        print(f"  Waiting {wait}s before opening: {title} ...")
        time.sleep(wait)
    print(f"  Opening terminal: {title}")
    subprocess.Popen([
        "gnome-terminal",
        f"--title={title}",
        "--",
        "bash", "-c", command
    ])


def build_workspace(ws: Path):
    banner("Step 2 – Building ROS 2 workspace with colcon")
    build_cmd = (
        "bash -c '"
        "source /opt/ros/humble/setup.bash && "
        f"cd {ws} && "
        "colcon build --symlink-install"
        "'"
    )
    result = subprocess.run(build_cmd, shell=True)
    if result.returncode != 0:
        print("\n[ERROR] colcon build failed. Check output above.")
        sys.exit(result.returncode)
    print("  Build successful!")


# ─────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────

def main():
    home = Path.home()
    ws   = home / "ros2_ws"
    src  = ws / "src"
    pkg  = src / "assurance_harness"

    # ── Step 1: Write all files ──────────────────────────────
    banner("Step 1 – Writing all package files")

    files = {
        pkg / "setup.py":                                    SETUP_PY,
        pkg / "package.xml":                                 PACKAGE_XML,
        pkg / "resource" / "assurance_harness":              "",
        pkg / "assurance_harness" / "__init__.py":           "",
        pkg / "assurance_harness" / "dsm.py":                DSM_PY,
        pkg / "assurance_harness" / "telemetry_gate_node.py":TELEMETRY_GATE_NODE_PY,
        pkg / "assurance_harness" / "risk_model_node.py":    RISK_MODEL_NODE_PY,
        pkg / "assurance_harness" / "evidence_logger_node.py":EVIDENCE_LOGGER_NODE_PY,
        pkg / "assurance_harness" / "viz_node.py":           VIZ_NODE_PY,
        pkg / "assurance_harness" / "fault_injector_node.py":FAULT_INJECTOR_NODE_PY,
        pkg / "launch" / "assurance.launch.py":              ASSURANCE_LAUNCH_PY,
        pkg / "launch" / "assurance_faulted.launch.py":      ASSURANCE_FAULTED_LAUNCH_PY,
    }

    for path, content in files.items():
        write_file(path, content)

    # ── Step 2: Build ────────────────────────────────────────
    build_workspace(ws)

    # ── Step 3-7: Open all terminals ─────────────────────────
    banner("Step 3 – Launching all terminals")

    print("\n  Terminal layout:")
    print("  T1 → PX4 + Gazebo        (starts immediately)")
    print("  T2 → MAVROS              (starts after 15s)")
    print("  T3 → Assurance Harness   (starts after 30s)")
    print("  T4 → Verification        (starts after 45s)")
    print("  T5 → gnuplot             (starts after 60s)")
    print("\n  NOTE: In the PX4 terminal, once Gazebo is open,")
    print("  type:  commander arm && commander takeoff -a 30")
    print()

    open_terminal("T1 - PX4 + Gazebo",      CMD_T1_PX4,     wait=0)
    open_terminal("T2 - MAVROS",            CMD_T2_MAVROS,  wait=15)
    open_terminal("T3 - Assurance Harness", CMD_T3_HARNESS, wait=15)
    open_terminal("T4 - Verification",      CMD_T4_VERIFY,  wait=10)
    open_terminal("T5 - gnuplot",           CMD_T5_GNUPLOT, wait=10)

    banner("All terminals launched!")
    print(textwrap.dedent("""
  What to do next:
  ─────────────────
  1. Wait for Gazebo to open in T1 (may take ~30s)
  2. In T1 (PX4 terminal), arm and take off:
       commander arm
       commander takeoff -a 30
  3. Watch risk change in T4:
       data: 1.0  → on ground (HIGH risk)
       data: 0.5  → low altitude, LOS clear (MEDIUM)
       data: 0.0  → above 15m (LOW risk)
  4. In T5, after data collects, plot with gnuplot:
       set datafile separator ","
       plot "evidence.csv" using 4:5 with points title "Risk vs Height"
  5. Take screenshots of each terminal for your lab report!
    """))


if __name__ == "__main__":
    main()
