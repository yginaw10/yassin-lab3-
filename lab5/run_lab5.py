#!/usr/bin/env python3
"""
run_lab5.py
===========
Writes all Lab 5 test and analysis files to ~/drone_mission/
and updates mission_executor.py with the watchdog.

Usage:
    python3 run_lab5.py
"""

from pathlib import Path

# ─────────────────────────────────────────────────────────────
#  TEST FILE 1: test_mission_planner.py
# ─────────────────────────────────────────────────────────────

TEST_MISSION_PLANNER = '''\
#!/usr/bin/env python3
"""
Unit tests for generate_lawnmower_waypoints().
Run with: pytest test_mission_planner.py -v
No ROS2 or Gazebo required.
"""
import sys
import os
import pytest

sys.path.insert(0, os.path.dirname(__file__))
from mission_planner import generate_lawnmower_waypoints


# TC-WP-01: Correct waypoint count
def test_waypoint_count_default_params():
    """
    TC-WP-01
    Precondition : default parameters (4 lines, 20m length, 3m spacing)
    Stimulus     : call generate_lawnmower_waypoints with default values
    Expected     : (20/3)+1 = 8 waypoints per line * 4 lines = 28 waypoints
    Pass criterion: len(waypoints) == 28
    """
    wps = generate_lawnmower_waypoints(
        origin_x=0.0, origin_y=0.0, altitude=10.0,
        line_spacing=5.0, waypoint_spacing=3.0,
        num_lines=4, line_length=20.0
    )
    expected = (int(20.0 / 3.0) + 1) * 4
    assert len(wps) == expected, f"Expected {expected} waypoints, got {len(wps)}"


# TC-WP-02: All waypoints at correct altitude
def test_all_waypoints_at_correct_altitude():
    """
    TC-WP-02
    Precondition : altitude=7.5 (non-default to catch hardcoding)
    Stimulus     : generate waypoints
    Expected     : every waypoint has z == 7.5
    Pass criterion: all(wp[2] == 7.5 for wp in wps)
    """
    altitude = 7.5
    wps = generate_lawnmower_waypoints(0, 0, altitude, 5.0, 3.0, 4, 20.0)
    assert all(wp[2] == altitude for wp in wps), (
        f"Not all waypoints at z={altitude}. Found: {set(wp[2] for wp in wps)}"
    )


# TC-WP-03: Lawnmower direction alternates
def test_lawnmower_direction_alternates():
    """
    TC-WP-03
    Precondition : 2 lines, 2 waypoints per line
    Stimulus     : generate waypoints
    Expected     : line 0 goes +y, line 1 goes -y
    Pass criterion: wps[1][1] > wps[0][1] and wps[3][1] < wps[2][1]
    """
    wps = generate_lawnmower_waypoints(
        origin_x=0.0, origin_y=0.0, altitude=10.0,
        line_spacing=5.0, waypoint_spacing=10.0,
        num_lines=2, line_length=10.0
    )
    assert wps[1][1] > wps[0][1], "Line 0 should travel in +y direction"
    assert wps[3][1] < wps[2][1], "Line 1 should travel in -y direction"


# TC-WP-04: Correct line spacing in x
def test_line_spacing_in_x():
    """
    TC-WP-04
    Precondition : line_spacing=5.0, num_lines=4
    Stimulus     : generate waypoints
    Expected     : x values of first waypoint on each line are 0, 5, 10, 15
    Pass criterion: line start x values match expected sequence
    """
    spacing = 5.0
    num_lines = 4
    wps = generate_lawnmower_waypoints(0, 0, 10.0, spacing, 3.0, num_lines, 9.0)
    wps_per_line = int(9.0 / 3.0) + 1
    for i in range(num_lines):
        first_wp_of_line = wps[i * wps_per_line]
        expected_x = i * spacing
        assert abs(first_wp_of_line[0] - expected_x) < 1e-9, (
            f"Line {i}: expected x={expected_x}, got x={first_wp_of_line[0]}"
        )


# TC-WP-05: Custom origin is respected
def test_custom_origin():
    """
    TC-WP-05
    Precondition : origin_x=10.0, origin_y=5.0
    Stimulus     : generate waypoints
    Expected     : first waypoint is (10.0, 5.0, altitude)
    Pass criterion: wps[0] == (10.0, 5.0, 10.0)
    """
    wps = generate_lawnmower_waypoints(10.0, 5.0, 10.0, 5.0, 3.0, 2, 10.0)
    assert wps[0][0] == 10.0, f"Expected origin_x=10.0, got {wps[0][0]}"
    assert wps[0][1] == 5.0,  f"Expected origin_y=5.0, got {wps[0][1]}"


# TC-WP-06: Single line, single waypoint
def test_single_line_single_waypoint():
    """
    TC-WP-06 Edge case: num_lines=1, waypoint_spacing >= line_length
    Expected    : exactly 1 waypoint at origin altitude
    """
    wps = generate_lawnmower_waypoints(0, 0, 10.0, 5.0, 20.0, 1, 10.0)
    assert len(wps) == 1, f"Expected 1 waypoint, got {len(wps)}"
    assert wps[0] == (0.0, 0.0, 10.0)


# TC-WP-07: Zero lines returns empty list
def test_zero_lines_returns_empty():
    """
    TC-WP-07 Edge case: num_lines=0
    Expected    : empty list, no exception
    """
    wps = generate_lawnmower_waypoints(0, 0, 10.0, 5.0, 3.0, 0, 20.0)
    assert wps == [], f"Expected empty list, got {wps}"


# TC-WP-08: All waypoints are 3-tuples of floats
def test_waypoints_are_float_tuples():
    """
    TC-WP-08 Type correctness test
    Expected    : every element is a tuple of 3 numeric values
    """
    wps = generate_lawnmower_waypoints(0, 0, 10.0, 5.0, 3.0, 3, 15.0)
    for i, wp in enumerate(wps):
        assert len(wp) == 3, f"WP {i} has {len(wp)} elements, expected 3"
        for j, val in enumerate(wp):
            assert isinstance(val, (int, float)), (
                f"WP {i}[{j}] is {type(val)}, expected numeric"
            )
'''

# ─────────────────────────────────────────────────────────────
#  TEST FILE 2: test_mission_executor.py
# ─────────────────────────────────────────────────────────────

TEST_MISSION_EXECUTOR = '''\
#!/usr/bin/env python3
"""
Unit tests for MissionState enum and mission logic.
Run with: pytest test_mission_executor.py -v
No ROS2 running required.
"""
import sys, os
import pytest

sys.path.insert(0, os.path.dirname(__file__))
from mission_executor import MissionState


# TC-SM-01: All required states exist
def test_all_states_exist():
    """
    TC-SM-01
    Precondition : MissionState enum imported
    Expected     : all 8 states present (including FAILSAFE)
    Pass criterion: no AttributeError
    """
    required = ["IDLE", "ARMED", "TAKEOFF", "NAVIGATING",
                "STATIONKEEPING", "RTL", "COMPLETE", "FAILSAFE"]
    for name in required:
        assert hasattr(MissionState, name), f"Missing state: {name}"


# TC-SM-02: States are distinct
def test_states_are_distinct():
    """
    TC-SM-02
    Expected : all state values are unique
    """
    values = [s.value for s in MissionState]
    assert len(values) == len(set(values)), (
        f"Duplicate state values detected: {values}"
    )


# TC-SM-03: State comparison works
def test_state_comparison():
    """
    TC-SM-03
    Expected : states compare equal to themselves, unequal to others
    """
    assert MissionState.IDLE == MissionState.IDLE
    assert MissionState.IDLE != MissionState.NAVIGATING
    assert MissionState.COMPLETE != MissionState.IDLE


# TC-SM-04: State name attribute is correct
def test_state_name():
    """
    TC-SM-04
    Expected : .name returns the string used in logs and /mission/state topic
    """
    assert MissionState.NAVIGATING.name    == "NAVIGATING"
    assert MissionState.STATIONKEEPING.name == "STATIONKEEPING"
    assert MissionState.COMPLETE.name      == "COMPLETE"
    assert MissionState.FAILSAFE.name      == "FAILSAFE"


# ── Mock classes for service logic tests ─────────────────────────────────

class MockResponse:
    def __init__(self):
        self.success = None
        self.message = ""

class MockExecutor:
    def __init__(self, state=MissionState.IDLE, waypoints=None):
        self.state     = state
        self.waypoints = waypoints or []

    def handle_start_mission(self, request, response):
        if self.state != MissionState.IDLE:
            response.success = False
            response.message = f"Cannot start — currently in state {self.state.name}"
            return response
        if not self.waypoints:
            response.success = False
            response.message = "No waypoints received yet."
            return response
        self.state = MissionState.ARMED
        response.success = True
        response.message = "Mission accepted."
        return response


# TC-SM-05: Service rejected when no waypoints
def test_service_rejected_when_no_waypoints():
    """
    TC-SM-05
    Precondition : executor in IDLE, no waypoints loaded
    Stimulus     : call handle_start_mission
    Expected     : response.success == False, message mentions waypoints
    """
    executor = MockExecutor(state=MissionState.IDLE, waypoints=[])
    response = MockResponse()
    executor.handle_start_mission(None, response)
    assert response.success == False
    assert "waypoints" in response.message.lower()


# TC-SM-06: Service rejected when not in IDLE
def test_service_rejected_when_not_idle():
    """
    TC-SM-06
    Precondition : executor in NAVIGATING state
    Stimulus     : call handle_start_mission
    Expected     : response.success == False, message mentions NAVIGATING
    """
    executor = MockExecutor(state=MissionState.NAVIGATING, waypoints=["wp1"])
    response = MockResponse()
    executor.handle_start_mission(None, response)
    assert response.success == False
    assert "NAVIGATING" in response.message


# TC-SM-07: Service accepted when IDLE with waypoints
def test_service_accepted_when_ready():
    """
    TC-SM-07
    Precondition : executor in IDLE, waypoints loaded
    Stimulus     : call handle_start_mission
    Expected     : response.success == True, state transitions to ARMED
    """
    executor = MockExecutor(state=MissionState.IDLE, waypoints=["wp1", "wp2"])
    response = MockResponse()
    executor.handle_start_mission(None, response)
    assert response.success == True
    assert executor.state == MissionState.ARMED


# TC-SM-08: Calling service twice rejected on second call
def test_service_idempotent_rejection():
    """
    TC-SM-08
    Precondition : call service once (transitions to ARMED)
    Stimulus     : call service again
    Expected     : second call rejected, state unchanged
    """
    executor = MockExecutor(state=MissionState.IDLE, waypoints=["wp1"])
    r1, r2 = MockResponse(), MockResponse()
    executor.handle_start_mission(None, r1)
    assert r1.success == True
    executor.handle_start_mission(None, r2)
    assert r2.success == False
    assert executor.state == MissionState.ARMED
'''

# ─────────────────────────────────────────────────────────────
#  KPI ANALYSIS SCRIPT: analyse_kpis.py
# ─────────────────────────────────────────────────────────────

ANALYSE_KPIS = '''\
#!/usr/bin/env python3
"""
KPI analysis script for COMP11132 mission telemetry.
Usage: python3 analyse_kpis.py ~/mission_log_YYYYMMDD_HHMMSS.csv
"""
import csv
import sys
import math

# Declared KPI targets (defined BEFORE looking at data)
KPI_TARGETS = {
    "stationkeep_duration_tolerance_s": 0.5,
    "configured_stationkeep_s":         3.0,
    "configured_altitude_m":           10.0,
    "max_altitude_deviation_m":         0.5,
    "max_takeoff_time_s":              15.0,
    "max_position_error_m":             0.5,
}


def load_csv(path):
    rows = []
    with open(path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({
                "time":  float(row["ros_time"]),
                "state": row["state"],
                "x":     float(row["x"]),
                "y":     float(row["y"]),
                "z":     float(row["z"]),
            })
    return rows


def find_first_transition(rows, from_state, to_state):
    prev = None
    for row in rows:
        if prev == from_state and row["state"] == to_state:
            return row["time"]
        prev = row["state"]
    return None


def find_all_transitions(rows, to_state):
    times = []
    prev = None
    for row in rows:
        if prev != to_state and row["state"] == to_state:
            times.append(row["time"])
        prev = row["state"]
    return times


def compute_kpis(rows):
    results = {}

    # KPI-1: Takeoff time
    armed_t = find_first_transition(rows, "IDLE", "ARMED")
    nav_t   = find_first_transition(rows, "TAKEOFF", "NAVIGATING")
    if armed_t and nav_t:
        results["takeoff_time_s"] = nav_t - armed_t
    else:
        results["takeoff_time_s"] = None

    # KPI-2: Altitude accuracy
    target_z     = KPI_TARGETS["configured_altitude_m"]
    flying_rows  = [r for r in rows if r["state"] in
                    ("NAVIGATING", "STATIONKEEPING", "RTL")]
    if flying_rows:
        deviations = [abs(r["z"] - target_z) for r in flying_rows]
        results["max_altitude_deviation_m"]  = max(deviations)
        results["mean_altitude_deviation_m"] = sum(deviations) / len(deviations)
    else:
        results["max_altitude_deviation_m"] = None

    # KPI-3: Stationkeep duration per waypoint
    sk_starts    = find_all_transitions(rows, "STATIONKEEPING")
    sk_ends_all  = find_all_transitions(rows, "NAVIGATING") + \
                   find_all_transitions(rows, "RTL")
    sk_ends_all.sort()
    durations = []
    for start in sk_starts:
        later = [t for t in sk_ends_all if t > start]
        if later:
            durations.append(later[0] - start)
    results["stationkeep_durations_s"] = durations
    if durations:
        results["stationkeep_mean_s"]  = sum(durations) / len(durations)
        results["stationkeep_max_err"] = max(
            abs(d - KPI_TARGETS["configured_stationkeep_s"]) for d in durations
        )

    # KPI-4: Total mission time
    armed_t2   = find_first_transition(rows, "IDLE", "ARMED")
    complete_t = find_first_transition(rows, "RTL", "COMPLETE")
    if armed_t2 and complete_t:
        results["total_mission_time_s"] = complete_t - armed_t2

    # KPI-5: Waypoints visited
    results["waypoints_visited"] = len(sk_starts)

    return results


def print_report(results):
    t = KPI_TARGETS
    print("\n" + "=" * 60)
    print("  KPI EVALUATION REPORT")
    print("=" * 60)

    def row(name, value, target, unit=""):
        if value is None:
            status  = "NO DATA"
            val_str = "N/A"
        else:
            val_str = f"{value:.2f}{unit}"
            status  = "PASS" if value <= target else "FAIL"
        print(f"  {name:<35} {val_str:<12} target:<{target}{unit}  [{status}]")

    row("Takeoff time",          results.get("takeoff_time_s"),         t["max_takeoff_time_s"],        "s")
    row("Max altitude deviation", results.get("max_altitude_deviation_m"), t["max_altitude_deviation_m"], "m")
    row("Max stationkeep error",  results.get("stationkeep_max_err"),    t["stationkeep_duration_tolerance_s"], "s")

    wps   = results.get("waypoints_visited", 0)
    total = results.get("total_mission_time_s")
    print(f"  {'Waypoints visited':<35} {wps}")
    if total:
        print(f"  {'Total mission time':<35} {total:.1f}s")

    sks = results.get("stationkeep_durations_s", [])
    if sks:
        print(f"\n  Stationkeep durations (s): {[round(d, 2) for d in sks]}")
        print(f"  Mean: {sum(sks)/len(sks):.2f}s  Configured: {t['configured_stationkeep_s']}s")
    print("=" * 60 + "\n")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 analyse_kpis.py <path_to_csv>")
        sys.exit(1)
    rows = load_csv(sys.argv[1])
    print(f"Loaded {len(rows)} rows from {sys.argv[1]}")
    results = compute_kpis(rows)
    print_report(results)
'''

# ─────────────────────────────────────────────────────────────
#  UPDATED MISSION EXECUTOR WITH WATCHDOG
# ─────────────────────────────────────────────────────────────

MISSION_EXECUTOR_WITH_WATCHDOG = '''\
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
    FAILSAFE       = auto()   # added for watchdog


class MissionExecutorNode(Node):
    def __init__(self):
        super().__init__("mission_executor")

        self.declare_parameter("stationkeep_duration", 3.0)
        self.declare_parameter("acceptance_radius",    2.0)
        self.declare_parameter("altitude",            10.0)
        self.declare_parameter("pose_timeout_s",       2.0)  # watchdog timeout

        self.stationkeep_duration = self.get_parameter("stationkeep_duration").value
        self.acceptance_radius    = self.get_parameter("acceptance_radius").value
        self.takeoff_altitude     = self.get_parameter("altitude").value
        self.pose_timeout         = self.get_parameter("pose_timeout_s").value

        self.get_logger().info(
            f"MissionExecutor — stationkeep={self.stationkeep_duration}s  "
            f"acceptance_radius={self.acceptance_radius}m  "
            f"takeoff_alt={self.takeoff_altitude}m  "
            f"pose_timeout={self.pose_timeout}s"
        )

        self.state             = MissionState.IDLE
        self.waypoints         = []
        self.wp_index          = 0
        self.current_pose      = None
        self.stationkeep_timer = None
        self.last_pose_time    = None   # watchdog timestamp

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
        self.current_pose   = msg.pose.position
        self.last_pose_time = self.get_clock().now()   # update watchdog

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
            response.message = "No waypoints received yet. Is MissionPlannerNode running?"
            return response
        self.transition(MissionState.ARMED)
        response.success = True
        response.message = "Mission accepted. Arming and taking off."
        return response

    def transition(self, new_state):
        self.get_logger().info(f"State: {self.state.name} -> {new_state.name}")
        self.state = new_state
        if new_state == MissionState.COMPLETE:
            self.get_logger().info("Mission complete. All waypoints visited.")
        if new_state == MissionState.FAILSAFE:
            self.get_logger().error("FAILSAFE active. Stopping setpoints. PX4 will auto-land.")
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

    def check_pose_watchdog(self):
        """Detect pose topic dropout and transition to FAILSAFE."""
        active_states = {
            MissionState.TAKEOFF,
            MissionState.NAVIGATING,
            MissionState.STATIONKEEPING,
            MissionState.RTL,
        }
        if self.state not in active_states:
            return
        if self.last_pose_time is None:
            return
        elapsed = (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9
        if elapsed > self.pose_timeout:
            self.get_logger().error(
                f"WATCHDOG TRIGGERED: no pose for {elapsed:.2f}s "
                f"(timeout={self.pose_timeout}s). Entering FAILSAFE."
            )
            self.transition(MissionState.FAILSAFE)

    def control_loop(self):
        self.check_pose_watchdog()   # FIRST — always check before acting
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
        elif self.state == MissionState.FAILSAFE:
            pass   # stop sending setpoints — PX4 will auto-land


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
#  WRITE ALL FILES
# ─────────────────────────────────────────────────────────────

def write_file(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content)
    path.chmod(path.stat().st_mode | 0o755)
    print(f"  [wrote]  {path}")


def main():
    drone_dir = Path.home() / "drone_mission"

    print("\n" + "─" * 55)
    print("  Lab 5 — Writing all files to ~/drone_mission/")
    print("─" * 55)

    write_file(drone_dir / "test_mission_planner.py",  TEST_MISSION_PLANNER)
    write_file(drone_dir / "test_mission_executor.py", TEST_MISSION_EXECUTOR)
    write_file(drone_dir / "analyse_kpis.py",          ANALYSE_KPIS)
    write_file(drone_dir / "mission_executor.py",      MISSION_EXECUTOR_WITH_WATCHDOG)

    print("\n" + "─" * 55)
    print("  All files written!")
    print("─" * 55)
    print("""
  Next steps:
  ─────────────────────────────────────────
  STEP 1 — Install pytest:
    pip install pytest --break-system-packages

  STEP 2 — Run waypoint unit tests:
    cd ~/drone_mission
    pytest test_mission_planner.py -v

  STEP 3 — Run state machine unit tests:
    pytest test_mission_executor.py -v

  STEP 4 — Run ALL tests together:
    pytest test_mission_planner.py test_mission_executor.py -v

  STEP 5 — Run KPI analysis on your CSV:
    python3 analyse_kpis.py ~/mission_log_*.csv

  STEP 6 — Start full system for integration tests:
    python3 ~/Downloads/run_lab4.py

  STEP 7 — Run integration checks (new terminal):
    source /opt/ros/humble/setup.bash
    ros2 topic list | grep mission
    ros2 topic hz /mission/waypoints --window 5
    ros2 topic hz /mission/state --window 10
  ─────────────────────────────────────────
    """)


if __name__ == "__main__":
    main()
