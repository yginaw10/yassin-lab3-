#!/usr/bin/env python3
"""
run_lab4.py
===========
Writes all Lab 4 node files and launches all terminals.

Usage:
    python3 run_lab4.py          -> writes files + opens all terminals
    python3 run_lab4.py --stop   -> kills everything

After launch:
  1. Wait for Gazebo to open in T1
  2. In T1 type:
       param set MIS_TAKEOFF_ALT 10.0
       commander mode offboard
       commander arm
  3. New terminal - start mission:
       source /opt/ros/humble/setup.bash
       ros2 service call /start_mission std_srvs/srv/Trigger '{}'
  4. Watch states:
       source /opt/ros/humble/setup.bash
       ros2 topic echo /mission/state
"""

import sys
import time
import subprocess
from pathlib import Path

# ─────────────────────────────────────────────────────────────
#  MISSION PLANNER
# ─────────────────────────────────────────────────────────────

MISSION_PLANNER_PY = '''\
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


def generate_lawnmower_waypoints(origin_x, origin_y, altitude,
                                  line_spacing, waypoint_spacing,
                                  num_lines, line_length):
    waypoints = []
    for i in range(num_lines):
        x = origin_x + i * line_spacing
        direction = 1 if i % 2 == 0 else -1
        num_wps = int(line_length / waypoint_spacing) + 1
        for j in range(num_wps):
            y = origin_y + direction * j * waypoint_spacing
            waypoints.append((x, y, altitude))
    return waypoints


class MissionPlannerNode(Node):
    def __init__(self):
        super().__init__("mission_planner")
        self.declare_parameter("altitude",            10.0)
        self.declare_parameter("cruise_speed",         3.0)
        self.declare_parameter("line_spacing",         5.0)
        self.declare_parameter("waypoint_spacing",     3.0)
        self.declare_parameter("stationkeep_duration", 3.0)
        self.declare_parameter("num_lines",            4)
        self.declare_parameter("line_length",         20.0)

        self.altitude         = self.get_parameter("altitude").value
        self.cruise_speed     = self.get_parameter("cruise_speed").value
        self.line_spacing     = self.get_parameter("line_spacing").value
        self.waypoint_spacing = self.get_parameter("waypoint_spacing").value
        self.stationkeep_dur  = self.get_parameter("stationkeep_duration").value
        self.num_lines        = self.get_parameter("num_lines").value
        self.line_length      = self.get_parameter("line_length").value

        self.get_logger().info(
            f"MissionPlanner — alt={self.altitude}m  speed={self.cruise_speed}m/s  "
            f"line_spacing={self.line_spacing}m  wp_spacing={self.waypoint_spacing}m  "
            f"stationkeep={self.stationkeep_dur}s  lines={self.num_lines}  "
            f"line_length={self.line_length}m"
        )

        self.wp_pub = self.create_publisher(Path, "/mission/waypoints", 10)

        waypoints = generate_lawnmower_waypoints(
            origin_x=0.0, origin_y=0.0,
            altitude=self.altitude,
            line_spacing=self.line_spacing,
            waypoint_spacing=self.waypoint_spacing,
            num_lines=self.num_lines,
            line_length=self.line_length,
        )

        self.get_logger().info(f"Generated {len(waypoints)} waypoints.")
        for idx, wp in enumerate(waypoints):
            self.get_logger().info(
                f"  WP {idx:02d}: x={wp[0]:.1f}  y={wp[1]:.1f}  z={wp[2]:.1f}")

        path_msg = Path()
        path_msg.header.stamp    = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for (x, y, z) in waypoints:
            pose = PoseStamped()
            pose.header.frame_id    = "map"
            pose.pose.position.x    = float(x)
            pose.pose.position.y    = float(y)
            pose.pose.position.z    = float(z)
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_msg = path_msg
        self.create_timer(1.0, self.publish_waypoints)

    def publish_waypoints(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.wp_pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
'''

# ─────────────────────────────────────────────────────────────
#  MISSION EXECUTOR
# ─────────────────────────────────────────────────────────────

MISSION_EXECUTOR_PY = '''\
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from enum import Enum, auto
import math


class MissionState(Enum):
    IDLE           = auto()
    ARMED          = auto()
    TAKEOFF        = auto()
    NAVIGATING     = auto()
    STATIONKEEPING = auto()
    RTL            = auto()
    COMPLETE       = auto()


class MissionExecutorNode(Node):
    def __init__(self):
        super().__init__("mission_executor")
        self.declare_parameter("stationkeep_duration", 3.0)
        self.declare_parameter("acceptance_radius",    2.0)
        self.declare_parameter("altitude",            10.0)

        self.stationkeep_duration = self.get_parameter("stationkeep_duration").value
        self.acceptance_radius    = self.get_parameter("acceptance_radius").value
        self.takeoff_altitude     = self.get_parameter("altitude").value

        self.get_logger().info(
            f"MissionExecutor — stationkeep={self.stationkeep_duration}s  "
            f"acceptance_radius={self.acceptance_radius}m  "
            f"takeoff_alt={self.takeoff_altitude}m"
        )

        self.state             = MissionState.IDLE
        self.waypoints         = []
        self.wp_index          = 0
        self.current_pose      = None
        self.stationkeep_timer = None

        self.setpoint_pub = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)
        self.state_pub = self.create_publisher(
            String, "/mission/state", 10)

        mavros_qos = QoSProfile(depth=10)
        mavros_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        mavros_qos.durability  = DurabilityPolicy.VOLATILE

        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose",
            self.pose_callback, mavros_qos)
        self.create_subscription(
            Path, "/mission/waypoints",
            self.waypoints_callback, 10)

        self.start_srv = self.create_service(
            Trigger, "/start_mission", self.handle_start_mission)

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("MissionExecutor ready. Call /start_mission to begin.")

    def pose_callback(self, msg):
        self.current_pose = msg.pose.position

    def waypoints_callback(self, msg):
        if self.state == MissionState.IDLE:
            self.waypoints = msg.poses
            self.get_logger().info(f"Received {len(self.waypoints)} waypoints.")

    def handle_start_mission(self, request, response):
        if self.state != MissionState.IDLE:
            response.success = False
            response.message = f"Cannot start — currently in state {self.state.name}"
            return response
        if not self.waypoints:
            response.success = False
            response.message = "No waypoints received yet."
            return response
        self.transition(MissionState.ARMED)
        response.success = True
        response.message = "Mission accepted. Arming and taking off."
        return response

    def transition(self, new_state):
        self.get_logger().info(f"State: {self.state.name} -> {new_state.name}")
        self.state = new_state
        self.publish_state()

    def publish_state(self):
        msg = String()
        msg.data = self.state.name
        self.state_pub.publish(msg)

    def distance_to(self, target_pose):
        if self.current_pose is None:
            return float("inf")
        dx = self.current_pose.x - target_pose.pose.position.x
        dy = self.current_pose.y - target_pose.pose.position.y
        dz = self.current_pose.z - target_pose.pose.position.z
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def publish_setpoint(self, pose_stamped):
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose = pose_stamped.pose
        self.setpoint_pub.publish(msg)

    def takeoff_setpoint(self):
        msg = PoseStamped()
        msg.header.stamp       = self.get_clock().now().to_msg()
        msg.header.frame_id    = "map"
        msg.pose.position.z    = self.takeoff_altitude
        msg.pose.orientation.w = 1.0
        self.setpoint_pub.publish(msg)

    def on_waypoint_reached(self):
        self.get_logger().info(
            f"Waypoint {self.wp_index}/{len(self.waypoints)-1} reached — "
            f"stationkeeping for {self.stationkeep_duration}s"
        )
        self.transition(MissionState.STATIONKEEPING)
        self.stationkeep_timer = self.create_timer(
            self.stationkeep_duration, self.on_stationkeep_complete)

    def on_stationkeep_complete(self):
        if self.stationkeep_timer is not None:
            self.stationkeep_timer.cancel()
            self.stationkeep_timer = None
        self.wp_index += 1
        if self.wp_index >= len(self.waypoints):
            self.get_logger().info("All waypoints complete. Returning to launch.")
            self.transition(MissionState.RTL)
        else:
            self.get_logger().info(f"Advancing to waypoint {self.wp_index}.")
            self.transition(MissionState.NAVIGATING)

    def control_loop(self):
        self.publish_state()
        if self.state == MissionState.IDLE:
            return
        elif self.state == MissionState.ARMED:
            self.takeoff_setpoint()
            if self.current_pose is not None:
                self.transition(MissionState.TAKEOFF)
        elif self.state == MissionState.TAKEOFF:
            self.takeoff_setpoint()
            if self.current_pose is not None:
                if abs(self.current_pose.z - self.takeoff_altitude) < self.acceptance_radius:
                    self.get_logger().info("Takeoff altitude reached.")
                    self.wp_index = 0
                    self.transition(MissionState.NAVIGATING)
        elif self.state == MissionState.NAVIGATING:
            target = self.waypoints[self.wp_index]
            self.publish_setpoint(target)
            if self.distance_to(target) < self.acceptance_radius:
                self.on_waypoint_reached()
        elif self.state == MissionState.STATIONKEEPING:
            self.publish_setpoint(self.waypoints[self.wp_index])
        elif self.state == MissionState.RTL:
            rtl = PoseStamped()
            rtl.pose.position.z    = self.takeoff_altitude
            rtl.pose.orientation.w = 1.0
            self.publish_setpoint(rtl)
            if self.current_pose is not None:
                if math.sqrt(self.current_pose.x**2 + self.current_pose.y**2) < self.acceptance_radius:
                    self.transition(MissionState.COMPLETE)
        elif self.state == MissionState.COMPLETE:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
'''

# ─────────────────────────────────────────────────────────────
#  TELEMETRY LOGGER
# ─────────────────────────────────────────────────────────────

TELEMETRY_LOGGER_PY = '''\
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import csv, os
from datetime import datetime


class TelemetryLoggerNode(Node):
    def __init__(self):
        super().__init__("telemetry_logger")
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_path  = os.path.expanduser(f"~/mission_log_{timestamp}.csv")
        self.csv_file = open(log_path, "w", newline="")
        self.writer   = csv.writer(self.csv_file)
        self.writer.writerow(["ros_time", "state", "x", "y", "z"])
        self.current_state = "UNKNOWN"
        self.current_pose  = None

        mavros_qos = QoSProfile(depth=10)
        mavros_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        mavros_qos.durability  = DurabilityPolicy.VOLATILE

        self.create_subscription(String, "/mission/state", self.state_cb, 10)
        self.create_subscription(
            PoseStamped, "/mavros/local_position/pose", self.pose_cb, mavros_qos)
        self.create_timer(0.5, self.log_row)
        self.get_logger().info(f"TelemetryLogger writing to {log_path}")

    def state_cb(self, msg): self.current_state = msg.data
    def pose_cb(self, msg):  self.current_pose  = msg.pose.position

    def log_row(self):
        if self.current_pose is None:
            return
        t = self.get_clock().now().nanoseconds / 1e9
        self.writer.writerow([f"{t:.3f}", self.current_state,
            f"{self.current_pose.x:.3f}",
            f"{self.current_pose.y:.3f}",
            f"{self.current_pose.z:.3f}"])
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryLoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
'''

# ─────────────────────────────────────────────────────────────
#  TERMINAL COMMANDS
# ─────────────────────────────────────────────────────────────

T1_PX4 = """\
bash -c '
echo "=== T1: PX4 + Gazebo ===";
cd ~/PX4-Autopilot;
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default;
Tools/simulation/gazebo-classic/sitl_run.sh \
  $(pwd)/build/px4_sitl_default/bin/px4 \
  none iris empty \
  $(pwd) $(pwd)/build/px4_sitl_default;
exec bash'
"""

T2_MAVROS = """\
bash -c '
echo "=== T2: MAVROS ===";
source /opt/ros/humble/setup.bash;
ros2 launch mavros px4.launch fcu_url:=udp://:14540@127.0.0.1:14557;
exec bash'
"""

T3_PLANNER = """\
bash -c '
echo "=== T3: Mission Planner ===";
source /opt/ros/humble/setup.bash;
cd ~/drone_mission;
python3 mission_planner.py;
exec bash'
"""

T4_EXECUTOR = """\
bash -c '
echo "=== T4: Mission Executor ===";
source /opt/ros/humble/setup.bash;
cd ~/drone_mission;
python3 mission_executor.py;
exec bash'
"""

T5_LOGGER = """\
bash -c '
echo "=== T5: Telemetry Logger ===";
source /opt/ros/humble/setup.bash;
cd ~/drone_mission;
python3 telemetry_logger.py;
exec bash'
"""

T6_MONITOR = """\
bash -c '
echo "=== T6: State Monitor ===";
source /opt/ros/humble/setup.bash;
sleep 10;
ros2 topic echo /mission/state;
exec bash'
"""

# ─────────────────────────────────────────────────────────────
#  KILL TARGETS
# ─────────────────────────────────────────────────────────────

KILL_TARGETS = [
    "px4", "gzserver", "gzclient", "gazebo",
    "mavros", "ros2", "ruby",
    "mission_planner", "mission_executor", "telemetry_logger",
]

# ─────────────────────────────────────────────────────────────
#  HELPERS
# ─────────────────────────────────────────────────────────────

def banner(msg):
    print("\n" + "─" * 55)
    print(f"  {msg}")
    print("─" * 55)

def write_file(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)
    path.chmod(path.stat().st_mode | 0o755)
    print(f"  [wrote]  {path}")

def open_terminal(title, command, wait=0):
    if wait > 0:
        print(f"  Waiting {wait}s -> {title}")
        time.sleep(wait)
    print(f"  Opening: {title}")
    subprocess.Popen([
        "gnome-terminal", f"--title={title}",
        "--", "bash", "-c", command
    ])

def stop_all():
    banner("STOPPING all Lab 4 processes")
    for name in KILL_TARGETS:
        result = subprocess.run(["pkill", "-f", name], capture_output=True)
        if result.returncode == 0:
            print(f"  [killed]       {name}")
        else:
            print(f"  [not running]  {name}")
    subprocess.run("pkill -f gnome-terminal", shell=True, capture_output=True)
    subprocess.run(
        "bash -c 'source /opt/ros/humble/setup.bash && ros2 daemon stop'",
        shell=True, capture_output=True)
    banner("Done. Everything stopped.")

# ─────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────

def main():
    if "--stop" in sys.argv:
        stop_all()
        return

    drone_dir = Path.home() / "drone_mission"

    banner("Step 1 – Writing all node files to ~/drone_mission/")
    write_file(drone_dir / "mission_planner.py",  MISSION_PLANNER_PY)
    write_file(drone_dir / "mission_executor.py", MISSION_EXECUTOR_PY)
    write_file(drone_dir / "telemetry_logger.py", TELEMETRY_LOGGER_PY)

    banner("Step 2 – Launching terminals")
    print("""
  T1 -> PX4 + Gazebo       (now)
  T2 -> MAVROS             (+20s)
  T3 -> Mission Planner    (+40s)
  T4 -> Mission Executor   (+45s)
  T5 -> Telemetry Logger   (+50s)
  T6 -> State Monitor      (+55s)
    """)

    open_terminal("T1 - PX4 + Gazebo",      T1_PX4,      wait=0)
    open_terminal("T2 - MAVROS",            T2_MAVROS,   wait=20)
    open_terminal("T3 - Mission Planner",   T3_PLANNER,  wait=20)
    open_terminal("T4 - Mission Executor",  T4_EXECUTOR, wait=5)
    open_terminal("T5 - Telemetry Logger",  T5_LOGGER,   wait=5)
    open_terminal("T6 - State Monitor",     T6_MONITOR,  wait=5)

    banner("All terminals launched!")
    print("""
  Next steps:
  ─────────────────────────────────────────────
  1. Wait for Gazebo + PX4 to fully open in T1

  2. In T1 (PX4 console) type:
       param set MIS_TAKEOFF_ALT 10.0
       commander mode offboard
       commander arm

  3. New terminal — start the mission:
       source /opt/ros/humble/setup.bash
       ros2 service call /start_mission std_srvs/srv/Trigger '{}'

  4. Watch T6 for state changes:
       ARMED -> TAKEOFF -> NAVIGATING -> STATIONKEEPING -> RTL -> COMPLETE

  5. Check CSV after mission:
       cat ~/mission_log_*.csv | head -20

  To STOP everything:
       python3 run_lab4.py --stop
  ─────────────────────────────────────────────
    """)

if __name__ == "__main__":
    main()
