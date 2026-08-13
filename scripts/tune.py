#!/usr/bin/env python3
"""Live parameter tuner for brain_node.

Run over SSH on the robot while brain is playing. Most brain parameters are
re-read via get_parameter() on every tick, so setting them takes effect
immediately -- no rebuild, no restart, no walking the robot back to position.

Launch through ./scripts/tune.sh, which sets the FastDDS profile that
start_brain.sh uses. Without that profile this tuner cannot discover brain_node.

Three kinds of knob, and the difference matters:

  live      Read every tick. Changes apply instantly. This is most of them.
  [restart] Snapshotted into config-> by Brain::loadConfig() at startup. The
            set SUCCEEDS but changes nothing until brain is relaunched. Shown
            dimmed and tagged so a no-op is never mistaken for a working knob.
  [unused]  Declared but never read anywhere in the brain source. Hidden from
            the group view; reachable via '/' search only.

Behaviour-tree port values -- the Chase node's vx_limit/vy_limit/dist/safe_dist,
Adjust's gains, Kick's speed_limit -- are NOT ROS parameters and cannot appear
here at all. Those still mean: edit behavior_trees/**.xml and restart. Press '?'
for the in-app version of this note.
"""

import curses
import os
import sys

try:
    import yaml
except ImportError:  # pragma: no cover - yaml ships with the ROS 2 desktop set
    yaml = None

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import (
    DescribeParameters,
    GetParameters,
    ListParameters,
    SetParameters,
)

NODE = "/brain_node"
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(REPO, "src", "brain", "config", "config.yaml")
CONFIG_LOCAL = os.path.join(REPO, "src", "brain", "config", "config_local.yaml")

# Snapshotted into config-> by Brain::loadConfig(); a live set is a silent no-op.
# Regenerate by diffing get_parameter() calls inside loadConfig() against those
# in tick paths if brain.cpp's parameter handling ever changes.
RESTART_ONLY = {
    "RLVisionKick.visual_kick_version",
    "enable_com",
    "game.field_type",
    "game.number_of_players",
    "game.player_id",
    "game.player_role",
    "game.player_start_pos",
    "game.team_id",
    "game.treat_person_as_robot",
    "locator.max_residual",
    "locator.min_marker_count",
    "obstacle_avoidance.avoid_secs",
    "obstacle_avoidance.collision_threshold",
    "obstacle_avoidance.safe_distance",
    "odomLog.enable",
    "odomLog.flush_interval_ms",
    "odomLog.hz",
    "odomLog.log_dir",
    "rerunLog.debug_hz",
    "rerunLog.enable",
    "rerunLog.enable_file",
    "rerunLog.enable_tcp",
    "rerunLog.img_interval",
    "rerunLog.log_dir",
    "rerunLog.max_log_file_mins",
    "rerunLog.server_ip",
    "rerunLog.timeseries_hz",
    "rerunLog.visual_hz",
    "robot.min_vtheta",
    "robot.min_vx",
    "robot.min_vy",
    "robot.odom_factor",
    "robot.odom_theta_alignment_distance",
    "robot.odom_theta_alignment_min_concentration",
    "robot.odom_theta_auto_align",
    "robot.odom_theta_offset",
    "robot.robot_height",
    "robot.vtheta_limit",
    "robot.vx_factor",
    "robot.vx_limit",
    "robot.vy_limit",
    "robot.yaw_offset",
    "sound.enable",
    "sound.sound_pack",
    "strategy.ball_confidence_threshold",
    "strategy.limit_near_ball_speed",
    "strategy.near_ball_range",
    "strategy.near_ball_speed_limit",
    "strategy.tm_ball_dist_threshold",
    "tree_file_path",
    "vision.cam_fov_x",
    "vision.cam_fov_y",
    "vision.depth_aligned_to_color",
    "vision.depth_is_z",
    "vision.depth_project_to_ground",
    "vision.depth_scale_16u",
    "vision.depth_scale_32f",
    "vision_config_local_path",
    "vision_config_path",
}

# Declared in brain.cpp but never read. Tuning these does nothing, ever.
UNUSED = {
    "recovery.post_getup_odom_settle_max_gyro_rad_per_sec",
    "recovery.post_getup_odom_settle_max_theta_range_deg",
    "recovery.post_getup_odom_settle_max_tilt_rad",
    "recovery.post_getup_odom_settle_max_xy_range_m",
    "recovery.post_getup_odom_settle_maximum_ms",
    "recovery.post_getup_odom_settle_minimum_ms",
    "recovery.post_getup_odom_settle_window_ms",
    "strategy.cooperation.assist_anchor_deadband",
    "strategy.cooperation.assist_target_deadband",
    "strategy.power_shoot.xmax",
    "strategy.power_shoot.xmin",
    "strategy.power_shoot.ymax",
    "strategy.power_shoot.ymin",
    "strategy.shoot.xmax",
    "strategy.shoot.xmin",
    "strategy.shoot.ymax",
    "strategy.shoot.ymin",
}

PI = 3.14159

# name -> (min, max, step). Bools carry no clamp.
GROUPS = [
    ("GOALIE", [
        ("strategy.cooperation.goalie_claim_max_ball_range", 0.0, 10.0, 0.25),
        ("strategy.cooperation.goalie_claim_extra_depth", 0.0, 5.0, 0.1),
        ("strategy.cooperation.goalie_claim_lateral_margin", 0.0, 3.0, 0.1),
        ("strategy.cooperation.goalie_box_priority_bonus", 0.0, 20.0, 0.5),
        ("strategy.cooperation.goalie_restore_stable_ms", 0.0, 5000.0, 50.0),
        ("strategy.cooperation.ball_control_cost_threshold", 0.0, 30.0, 0.5),
        ("strategy.cooperation.enable_role_switch", None, None, None),
    ]),
    ("VISUAL KICK", [
        ("strategy.enable_auto_visual_kick", None, None, None),
        ("strategy.auto_visual_kick_enable_dist_min", 0.0, 5.0, 0.1),
        ("strategy.auto_visual_kick_enable_dist_max", 0.0, 10.0, 0.1),
        ("strategy.auto_visual_kick_enable_angle", 0.0, PI, 0.05),
        ("obstacle_avoidance.enable_fallen_robot_visual_kick_exit", None, None, None),
        ("obstacle_avoidance.fallen_robot_distance", 0.0, 5.0, 0.1),
        ("obstacle_avoidance.fallen_robot_angle", 0.0, PI, 0.05),
    ]),
    ("KICK & SHOOT", [
        ("strategy.kick_range", 0.0, 5.0, 0.05),
        ("strategy.kick_theta_range", 0.0, 1.57, 0.02),
        ("strategy.enable_shoot", None, None, None),
        ("strategy.enable_bypass", None, None, None),
        ("strategy.enable_directional_kick", None, None, None),
        ("strategy.power_shoot.enable", None, None, None),
        ("strategy.power_shoot.use_for_kickoff", None, None, None),
        ("strategy.shoot.threat_threshold", -10.0, 10.0, 0.25),
        ("strategy.soft_kickoff", None, None, None),
        ("strategy.soft_kickoff_speed", 0.0, 2.0, 0.05),
        ("strategy.abort_kick_when_ball_moved", None, None, None),
        ("strategy.abort_kick_ball_move_threshold", 0.0, 2.0, 0.05),
        ("strategy.use_move_block", None, None, None),
        ("strategy.move_block_msecs", 0.0, 10000.0, 100.0),
    ]),
    ("SEARCH", [
        ("strategy.search.vx_limit", 0.0, 2.0, 0.05),
        ("strategy.search.vy_limit", 0.0, 2.0, 0.05),
        ("strategy.search.vtheta_limit", 0.0, 3.0, 0.05),
        ("strategy.search.initial_spin_secs", 0.0, 10.0, 0.25),
        ("strategy.search.waypoint_scan_secs", 0.0, 15.0, 0.25),
        ("strategy.search.waypoint_timeout_secs", 0.0, 60.0, 1.0),
        ("strategy.search.arrival_tolerance", 0.0, 3.0, 0.05),
        ("strategy.search.last_observation_max_age_secs", 0.0, 60.0, 0.5),
        ("strategy.ball_memory_timeout", 0.0, 10.0, 0.25),
    ]),
    ("OBSTACLE", [
        ("obstacle_avoidance.avoid_during_chase", None, None, None),
        ("obstacle_avoidance.chase_ao_safe_dist", 0.0, 5.0, 0.1),
        ("obstacle_avoidance.avoid_during_kick", None, None, None),
        ("obstacle_avoidance.kick_ao_safe_dist", 0.0, 5.0, 0.1),
        ("obstacle_avoidance.kick_ao_use_shoot", None, None, None),
        ("obstacle_avoidance.robot_path_clearance", 0.0, 2.0, 0.05),
        ("obstacle_avoidance.robot_avoidance_speed", 0.0, 2.0, 0.05),
        ("obstacle_avoidance.robot_avoidance_reverse_speed", 0.0, 2.0, 0.05),
        ("obstacle_avoidance.robot_avoidance_min_speed", 0.0, 2.0, 0.05),
        ("obstacle_avoidance.robot_avoidance_min_vy", 0.0, 2.0, 0.05),
        ("obstacle_avoidance.blocked_recovery_turn_rate", 0.0, 3.0, 0.05),
        ("obstacle_avoidance.avoidance_direction_lock_msecs", 0.0, 5000.0, 50.0),
        ("obstacle_avoidance.enable_ball_obstacle", None, None, None),
        ("obstacle_avoidance.ball_obstacle_radius", 0.0, 2.0, 0.05),
        ("obstacle_avoidance.avoid_person", None, None, None),
        ("obstacle_avoidance.enable_freekick_avoid", None, None, None),
    ]),
    ("COOPERATION", [
        ("strategy.cooperation.assist_avoid_obstacles", None, None, None),
        ("strategy.cooperation.assist_slot_switch_penalty", 0.0, 30.0, 0.5),
        ("strategy.cooperation.assist_anchor_switch_penalty", 0.0, 30.0, 0.5),
        ("strategy.cooperation.assist_lane_cross_penalty", 0.0, 40.0, 0.5),
        ("strategy.cooperation.assist_path_cross_penalty", 0.0, 40.0, 0.5),
        ("strategy.cooperation.assist_anchor_hold_ms", 0.0, 10000.0, 100.0),
        ("strategy.cooperation.assist_shadow_side_hold_ms", 0.0, 10000.0, 100.0),
        ("strategy.cooperation.assist_yield_timeout_ms", 0.0, 15000.0, 100.0),
        ("strategy.cooperation.assist_teammate_position_clearance", 0.0, 3.0, 0.05),
        ("strategy.cooperation.assist_teammate_path_clearance", 0.0, 3.0, 0.05),
        ("strategy.cooperation.assist_waypoint_lock_ms", 0.0, 10000.0, 100.0),
        ("strategy.cooperation.assist_waypoint_ttl_ms", 0.0, 10000.0, 100.0),
        ("strategy.cooperation.assist_waypoint_no_progress_ms", 0.0, 10000.0, 100.0),
        ("strategy.cooperation.leader_switch_cost_margin", 0.0, 5.0, 0.05),
        ("strategy.cooperation.leader_switch_cost_ratio", 0.0, 1.5, 0.01),
        ("strategy.cooperation.leader_switch_confirm_ms", 0.0, 5000.0, 50.0),
        ("strategy.cooperation.leader_min_hold_ms", 0.0, 10000.0, 100.0),
        ("strategy.cooperation.leader_ownerless_fallback_ms", 0.0, 5000.0, 50.0),
        ("strategy.cooperation.kickoff_owner_select_ms", 0.0, 5000.0, 50.0),
        ("strategy.cooperation.kickoff_ball_move_threshold", 0.0, 2.0, 0.05),
        ("strategy.cooperation.kickoff_timeout_ms", 0.0, 30000.0, 500.0),
        ("strategy.cooperation.tactical_packet_timeout_ms", 0.0, 5000.0, 50.0),
        ("strategy.freekick.execution_timeout_ms", 0.0, 30000.0, 500.0),
        ("strategy.freekick.plan_select_ms", 0.0, 3000.0, 50.0),
    ]),
    ("RECOVERY", [
        ("recovery.retry_max_count", 0.0, 20.0, 1.0),
        ("recovery.getup_version", 0.0, 5.0, 1.0),
        ("recovery.getup_no_movement_grace_sec", 0.0, 30.0, 0.5),
        ("recovery.localization_resume_delay_ms", 0.0, 10000.0, 100.0),
        ("recovery.prepare_wait_ms", 0.0, 10000.0, 100.0),
        ("recovery.heading_realign_attempts", 0.0, 20.0, 1.0),
        ("recovery.heading_realign_confirmations", 0.0, 20.0, 1.0),
        ("recovery.heading_realign_interval_ms", 0.0, 10000.0, 100.0),
        ("recovery.heading_realign_max_delta_rad", 0.0, PI, 0.05),
        ("recovery.heading_realign_min_delta_rad", 0.0, PI, 0.05),
    ]),
    # Kind B. Grouped separately so the [restart] tag is impossible to miss --
    # these read back the value you set while the robot keeps its old behaviour.
    ("MOTION [restart]", [
        ("robot.vx_limit", 0.0, 4.0, 0.1),
        ("robot.vy_limit", 0.0, 3.0, 0.1),
        ("robot.vtheta_limit", 0.0, 3.0, 0.1),
        ("robot.vx_factor", 0.0, 2.0, 0.05),
        ("robot.min_vx", 0.0, 1.0, 0.05),
        ("robot.min_vy", 0.0, 1.0, 0.05),
        ("robot.min_vtheta", 0.0, 1.0, 0.05),
        ("strategy.limit_near_ball_speed", None, None, None),
        ("strategy.near_ball_speed_limit", 0.0, 2.0, 0.05),
        ("strategy.near_ball_range", 0.0, 5.0, 0.1),
    ]),
]

HELP = """
  MOVING AROUND
    up/down     select knob            left/right   step value
    shift+arrow coarse step (10x)      tab          next group
    enter       type an exact value    space        toggle a bool
    /           search all parameters  esc          leave search
    r           revert knob to the value it had when the tuner started
    s           save changed knobs to config_local.yaml
    q           quit (values already set stay set; brain is untouched)

  WHAT THE TAGS MEAN
    (none)      Read every tick. Your change is already in effect.
    [restart]   Snapshotted by loadConfig() at startup. The set succeeds and
                reads back, but brain keeps using the old value until it is
                relaunched. Not a bug in the tuner -- it is how brain loads it.
    [unused]    Declared in brain.cpp, never read. Does nothing at all.

  WHAT IS MISSING, AND WHY
    Behaviour-tree port values are not ROS parameters, so they cannot be shown
    or set here. To change any of these, edit the XML and restart brain:

      Chase   vx_limit, vy_limit, vtheta_limit, dist, safe_dist
      Adjust  turn_threshold, range, vx/vy/vtheta_limit, tangential_speed_*
      Kick    speed_limit, min_msec_kick, msecs_stablize
      *Decide chase_threshold

      src/brain/behavior_trees/subtrees/subtree_striker_play.xml
      src/brain/behavior_trees/subtrees/subtree_goal_keeper_play.xml

  Press any key to go back.
"""


def flatten(tree, prefix=""):
    """config.yaml nests parameters; brain addresses them with dots."""
    out = {}
    for key, value in (tree or {}).items():
        name = "%s.%s" % (prefix, key) if prefix else str(key)
        if isinstance(value, dict):
            out.update(flatten(value, name))
        else:
            out[name] = value
    return out


def nest(flat):
    """Inverse of flatten(), so config_local.yaml matches config.yaml's shape."""
    root = {}
    for name, value in sorted(flat.items()):
        parts = name.split(".")
        node = root
        for part in parts[:-1]:
            node = node.setdefault(part, {})
        node[parts[-1]] = value
    return root


def to_python(pv):
    if pv.type == ParameterType.PARAMETER_BOOL:
        return bool(pv.bool_value)
    if pv.type == ParameterType.PARAMETER_INTEGER:
        return int(pv.integer_value)
    if pv.type == ParameterType.PARAMETER_DOUBLE:
        return float(pv.double_value)
    if pv.type == ParameterType.PARAMETER_STRING:
        return pv.string_value
    return None


def to_value(python_value, ptype):
    pv = ParameterValue()
    pv.type = ptype
    if ptype == ParameterType.PARAMETER_BOOL:
        pv.bool_value = bool(python_value)
    elif ptype == ParameterType.PARAMETER_INTEGER:
        pv.integer_value = int(round(float(python_value)))
    elif ptype == ParameterType.PARAMETER_DOUBLE:
        pv.double_value = float(python_value)
    elif ptype == ParameterType.PARAMETER_STRING:
        pv.string_value = str(python_value)
    return pv


class Brain:
    """Persistent service clients.

    Shelling out to `ros2 param set` costs 1-2 s per change because each
    invocation builds a node and waits for discovery. Holding the clients open
    makes a change cost milliseconds, which is what makes live tuning usable.
    """

    def __init__(self, node):
        self.node = node
        self.list = node.create_client(ListParameters, NODE + "/list_parameters")
        self.describe = node.create_client(
            DescribeParameters, NODE + "/describe_parameters")
        self.get = node.create_client(GetParameters, NODE + "/get_parameters")
        self.set = node.create_client(SetParameters, NODE + "/set_parameters")
        self.types = {}

    def wait(self, timeout=30.0):
        for client in (self.list, self.describe, self.get, self.set):
            if not client.wait_for_service(timeout_sec=timeout):
                return False
        return True

    def _call(self, client, request, timeout=5.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        return future.result()

    def names(self):
        response = self._call(self.list, ListParameters.Request())
        return sorted(response.result.names) if response else []

    def load_types(self, names):
        for chunk in [names[i:i + 60] for i in range(0, len(names), 60)]:
            request = DescribeParameters.Request()
            request.names = chunk
            response = self._call(self.describe, request)
            if not response:
                continue
            for name, desc in zip(chunk, response.descriptors):
                self.types[name] = desc.type

    def read(self, names):
        values = {}
        for chunk in [names[i:i + 60] for i in range(0, len(names), 60)]:
            request = GetParameters.Request()
            request.names = chunk
            response = self._call(self.get, request)
            if not response:
                continue
            for name, pv in zip(chunk, response.values):
                values[name] = to_python(pv)
        return values

    def write(self, name, python_value):
        ptype = self.types.get(name)
        if ptype is None:
            return "unknown parameter"
        parameter = Parameter()
        parameter.name = name
        parameter.value = to_value(python_value, ptype)
        request = SetParameters.Request()
        request.parameters = [parameter]
        response = self._call(self.set, request)
        if not response or not response.results:
            return "no response from brain_node"
        result = response.results[0]
        return None if result.successful else (result.reason or "rejected")


class Tuner:
    def __init__(self, screen, brain):
        self.screen = screen
        self.brain = brain
        self.names = brain.names()
        brain.load_types(self.names)
        self.values = brain.read(self.names)
        self.baseline = dict(self.values)          # for 'r'
        self.file_baseline = self.load_config()    # for 's'
        self.groups = self.build_groups()
        self.group = 0
        self.row = 0
        self.status = "%d parameters -- %d live, %d [restart], %d [unused]" % (
            len(self.names),
            len(self.names) - len(RESTART_ONLY & set(self.names))
            - len(UNUSED & set(self.names)),
            len(RESTART_ONLY & set(self.names)),
            len(UNUSED & set(self.names)),
        )
        self.query = ""
        self.searching = False

    def load_config(self):
        if yaml is None or not os.path.isfile(CONFIG):
            return {}
        try:
            with open(CONFIG) as handle:
                doc = yaml.safe_load(handle) or {}
            return flatten(doc.get("brain_node", {}).get("ros__parameters", {}))
        except Exception:
            return {}

    def build_groups(self):
        known = set(self.names)
        groups = []
        curated = set()
        for title, knobs in GROUPS:
            rows = [k for k in knobs if k[0] in known]
            if rows:
                groups.append((title, rows))
                curated.update(row[0] for row in rows)
        # Everything live but not curated, so nothing is unreachable.
        rest = [n for n in self.names
                if n not in curated and n not in UNUSED]
        if rest:
            groups.append(("OTHER", [(n,) + self.generic(n) for n in rest]))
        return groups

    def generic(self, name):
        """Derive a clamp for uncurated knobs from their current value."""
        value = self.values.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return (None, None, None)
        magnitude = abs(float(value))
        if magnitude == 0.0:
            return (-1.0, 1.0, 0.05)
        lo = 0.0 if value > 0 else -4.0 * magnitude
        return (lo, 4.0 * magnitude, max(magnitude / 20.0, 0.01))

    def tag(self, name):
        if name in RESTART_ONLY:
            return "[restart]"
        if name in UNUSED:
            return "[unused]"
        return ""

    def rows(self):
        if self.searching:
            query = self.query.lower()
            hits = [n for n in self.names if query in n.lower()]
            return [(n,) + self.generic(n) for n in hits]
        return self.groups[self.group][1]

    # ---------------------------------------------------------------- editing

    def selected(self):
        rows = self.rows()
        if not rows:
            return None
        self.row = max(0, min(self.row, len(rows) - 1))
        return rows[self.row]

    def apply(self, name, value):
        error = self.brain.write(name, value)
        if error:
            self.status = "%s: %s" % (name, error)
            return
        self.values[name] = self.brain.read([name]).get(name, value)
        note = " (needs restart to take effect)" if name in RESTART_ONLY else ""
        self.status = "%s = %s%s" % (name, self.fmt(self.values[name]), note)

    def step(self, direction, coarse=False):
        row = self.selected()
        if not row:
            return
        name, lo, hi, step = row
        current = self.values.get(name)
        if isinstance(current, bool):
            self.apply(name, not current)
            return
        if step is None or not isinstance(current, (int, float)):
            self.status = "%s is not steppable -- press enter to type a value" % name
            return
        delta = step * (10.0 if coarse else 1.0) * direction
        target = float(current) + delta
        if lo is not None:
            target = max(lo, min(hi, target))
        self.apply(name, target)

    def prompt(self, label):
        height, width = self.screen.getmaxyx()
        curses.echo()
        curses.curs_set(1)
        self.screen.move(height - 1, 0)
        self.screen.clrtoeol()
        self.screen.addstr(height - 1, 0, label)
        try:
            text = self.screen.getstr(height - 1, len(label), 40).decode()
        except Exception:
            text = ""
        curses.noecho()
        curses.curs_set(0)
        return text.strip()

    def type_value(self):
        row = self.selected()
        if not row:
            return
        name = row[0]
        current = self.values.get(name)
        text = self.prompt("%s = " % name)
        if not text:
            return
        try:
            if isinstance(current, bool):
                value = text.lower() in ("1", "true", "yes", "on")
            elif isinstance(current, int):
                value = int(float(text))
            elif isinstance(current, float):
                value = float(text)
            else:
                value = text
        except ValueError:
            self.status = "not a number: %s" % text
            return
        self.apply(name, value)

    def revert(self):
        row = self.selected()
        if not row:
            return
        name = row[0]
        if name not in self.baseline:
            return
        self.apply(name, self.baseline[name])

    def save(self):
        if yaml is None:
            self.status = "PyYAML not installed -- cannot write config_local.yaml"
            return
        changed = {}
        for name, value in self.values.items():
            if name in UNUSED:
                continue
            base = self.file_baseline.get(name)
            if base is None or self.differs(value, base):
                changed[name] = value
        if not changed:
            self.status = "nothing differs from config.yaml -- not written"
            return
        document = {"brain_node": {"ros__parameters": nest(changed)}}
        try:
            with open(CONFIG_LOCAL, "w") as handle:
                handle.write(
                    "# Written by scripts/tune.py. Loaded by launch.py AFTER\n"
                    "# config.yaml, so every key here overrides config.yaml.\n"
                    "# Per-robot and gitignored: delete it to fall back to config.yaml.\n")
                yaml.safe_dump(document, handle, default_flow_style=False, sort_keys=True)
        except OSError as exc:
            self.status = "save failed: %s" % exc
            return
        self.status = "saved %d knob(s) to config_local.yaml" % len(changed)

    @staticmethod
    def differs(a, b):
        if isinstance(a, bool) or isinstance(b, bool):
            return bool(a) != bool(b)
        if isinstance(a, (int, float)) and isinstance(b, (int, float)):
            return abs(float(a) - float(b)) > 1e-9
        return a != b

    # ----------------------------------------------------------------- render

    @staticmethod
    def fmt(value):
        if isinstance(value, bool):
            return "true" if value else "false"
        if isinstance(value, float):
            return "%.3f" % value
        return str(value)

    def bar(self, value, lo, hi, width=10):
        if lo is None or hi is None or hi <= lo:
            return ""
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            return ""
        filled = int(round((float(value) - lo) / (hi - lo) * width))
        filled = max(0, min(width, filled))
        return "[" + "=" * filled + "-" * (width - filled) + "]"

    def draw(self):
        screen = self.screen
        screen.erase()
        height, width = screen.getmaxyx()
        dim = curses.A_DIM
        rev = curses.A_REVERSE

        title = " brain_node live tuner "
        screen.addstr(0, 0, title.ljust(width - 1)[:width - 1], rev)

        gutter = 22
        top = 2
        body = height - 5

        if self.searching:
            screen.addstr(top, 0, "SEARCH".ljust(gutter - 2)[:gutter - 2], rev)
        else:
            for index, (name, _) in enumerate(self.groups):
                if top + index >= top + body:
                    break
                attr = rev if index == self.group else curses.A_NORMAL
                screen.addstr(top + index, 0,
                              (" " + name).ljust(gutter - 2)[:gutter - 2], attr)

        rows = self.rows()
        if not rows:
            screen.addstr(top, gutter, "no matches")
        first = max(0, self.row - body + 2)
        for offset, row in enumerate(rows[first:first + body]):
            y = top + offset
            if y >= height - 3:
                break
            name, lo, hi, step = (list(row) + [None, None, None])[:4]
            value = self.values.get(name)
            tag = self.tag(name)
            attr = curses.A_NORMAL
            if first + offset == self.row:
                attr = rev
            elif tag:
                attr = dim
            label = name.split(".")[-1] if not self.searching else name
            text = "%-42s %10s %s %s" % (
                label[:42], self.fmt(value), self.bar(value, lo, hi), tag)
            screen.addstr(y, gutter, text[:max(0, width - gutter - 1)], attr)

        if self.searching:
            screen.addstr(height - 3, 0, ("/" + self.query)[:width - 1])
        screen.addstr(height - 2, 0, self.status[:width - 1], dim)
        keys = " arrows step  enter type  space toggle  / search  r revert  s save  ? help  q quit "
        screen.addstr(height - 1, 0, keys.ljust(width - 1)[:width - 1], rev)
        screen.refresh()

    def help(self):
        self.screen.erase()
        height, width = self.screen.getmaxyx()
        for index, line in enumerate(HELP.splitlines()):
            if index >= height - 1:
                break
            self.screen.addstr(index, 0, line[:width - 1])
        self.screen.refresh()
        self.screen.timeout(-1)
        self.screen.getch()
        self.screen.timeout(1500)

    # ------------------------------------------------------------------- loop

    def run(self):
        self.screen.timeout(1500)
        while True:
            self.draw()
            key = self.screen.getch()

            if key == -1:  # idle refresh; catches changes made from elsewhere
                self.values.update(self.brain.read(self.names))
                continue
            if self.searching and 32 <= key < 127 and key != ord("/"):
                self.query += chr(key)
                self.row = 0
                continue
            if self.searching and key in (curses.KEY_BACKSPACE, 127, 8):
                self.query = self.query[:-1]
                self.row = 0
                continue

            if key in (ord("q"), ord("Q")):
                return
            if key == ord("?"):
                self.help()
            elif key == ord("/"):
                self.searching = True
                self.query = ""
                self.row = 0
            elif key == 27:  # esc
                self.searching = False
                self.row = 0
            elif key == curses.KEY_UP:
                self.row = max(0, self.row - 1)
            elif key == curses.KEY_DOWN:
                self.row = min(len(self.rows()) - 1, self.row + 1)
            elif key == curses.KEY_LEFT:
                self.step(-1)
            elif key == curses.KEY_RIGHT:
                self.step(+1)
            elif key == curses.KEY_SLEFT:
                self.step(-1, coarse=True)
            elif key == curses.KEY_SRIGHT:
                self.step(+1, coarse=True)
            elif key == ord("\t") and not self.searching:
                self.group = (self.group + 1) % len(self.groups)
                self.row = 0
            elif key in (curses.KEY_ENTER, 10, 13):
                self.type_value()
            elif key == ord(" "):
                row = self.selected()
                if row and isinstance(self.values.get(row[0]), bool):
                    self.apply(row[0], not self.values[row[0]])
            elif key == ord("r"):
                self.revert()
            elif key == ord("s"):
                self.save()


def main():
    rclpy.init()
    node = Node("brain_tuner")
    brain = Brain(node)

    sys.stderr.write("waiting for %s parameter services...\n" % NODE)
    if not brain.wait(timeout=30.0):
        sys.stderr.write(
            "\n%s did not answer.\n"
            "  - is brain running?  ./scripts/start_brain.sh\n"
            "  - did you launch through ./scripts/tune.sh? without its\n"
            "    FASTDDS_DEFAULT_PROFILES_FILE the tuner cannot see brain_node\n" % NODE)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    try:
        curses.wrapper(lambda screen: Tuner(screen, brain).run())
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
