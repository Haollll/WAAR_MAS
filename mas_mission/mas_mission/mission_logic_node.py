"""
mission_logic_node.py
---------------------
Top-level mission coordinator.  Owns the StateMachine and orchestrates
the sync node + task node based on current state.

Responsibilities:
  - Track mission timer
  - Build MissionContext from team beliefs and drone states
  - Drive state machine tick
  - Issue high-level directives per state:
      SURVEY       → tell explorer to sweep assigned grid sector
      VERIFY_TAG   → announce VERIFY_TAG tasks for unconfirmed mines
      PATH_VERIFY  → announce path verification task
      CONVERGE     → broadcast final occupancy grid
      FINISH       → trigger land + submit

Parameters:
  drone_id          : str
  mission_duration  : float   total mission time in seconds (default 600)
  num_drones        : int     expected team size (default 4)
  grid_cols         : int     grid subdivision columns (default 2)
  grid_rows         : int     grid subdivision rows    (default 2)
  arena_width       : float   arena width in metres    (default 30.0)
  arena_height      : float   arena height in metres   (default 30.0)
"""

import rclpy
from rclpy.node import Node
import time
import json
import math
from typing import Dict, Optional, List

from mas_interfaces.msg import (
    PoseBeacon, TaskAnnounce, TaskResult, MineBelief, MineDelta
)
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

from .state_machine import StateMachine, MissionContext


# ── Grid sector assignment ────────────────────────────────────────────────────

def assign_sector(drone_index: int, num_drones: int,
                  cols: int, rows: int,
                  arena_w: float, arena_h: float) -> dict:
    """
    Divide arena into cols×rows grid; assign one cell to each drone.
    Returns {"x_min","x_max","y_min","y_max","cx","cy"}.
    Falls back to whole arena if more drones than cells.
    """
    cells = [(c, r) for r in range(rows) for c in range(cols)]
    if drone_index >= len(cells):
        drone_index = drone_index % len(cells)
    col, row = cells[drone_index]
    cell_w = arena_w / cols
    cell_h = arena_h / rows
    x_min = col * cell_w
    x_max = x_min + cell_w
    y_min = row * cell_h
    y_max = y_min + cell_h
    return {
        "x_min": x_min, "x_max": x_max,
        "y_min": y_min, "y_max": y_max,
        "cx": (x_min + x_max) / 2,
        "cy": (y_min + y_max) / 2,
    }


# ── Main node ─────────────────────────────────────────────────────────────────

class MissionLogicNode(Node):

    def __init__(self):
        super().__init__("mission_logic_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("drone_id",         "d1")
        self.declare_parameter("mission_duration", 600.0)
        self.declare_parameter("num_drones",        4)
        self.declare_parameter("drone_index",       0)    # 0-based position in team
        self.declare_parameter("grid_cols",         2)
        self.declare_parameter("grid_rows",         2)
        self.declare_parameter("arena_width",       30.0)
        self.declare_parameter("arena_height",      30.0)

        self.drone_id        = self.get_parameter("drone_id").value
        mission_duration     = self.get_parameter("mission_duration").value
        self.num_drones      = self.get_parameter("num_drones").value
        drone_index          = self.get_parameter("drone_index").value
        grid_cols            = self.get_parameter("grid_cols").value
        grid_rows            = self.get_parameter("grid_rows").value
        arena_w              = self.get_parameter("arena_width").value
        arena_h              = self.get_parameter("arena_height").value

        # ── Mission state ─────────────────────────────────────────────────────
        self.sm = StateMachine(self.drone_id, mission_duration)
        self.sm.on_transition(self._on_state_transition)

        self.mission_start: Optional[float] = None   # set on first SURVEY enter
        self.mission_duration = mission_duration

        # Sector this drone is responsible for
        self.sector = assign_sector(
            drone_index, self.num_drones,
            grid_cols, grid_rows, arena_w, arena_h)

        # Team awareness
        self.team_states: Dict[str, str] = {}        # drone_id → state
        self.team_last_seen: Dict[str, float] = {}   # drone_id → monotonic

        # Mine beliefs (mirrored from sync node via /team/mine_delta)
        self.mine_beliefs: Dict[str, dict] = {}      # mine_id → dict
        self.path_verified = False

        # PATH_VERIFY task tracking
        self._path_task_announced = False
        self._path_task_id: Optional[str] = None

        # Occupancy grid (built during CONVERGE)
        self._grid_published = False

        # ── Publishers ────────────────────────────────────────────────────────
        # High-level command to explorer_node
        self.pub_mission_cmd = self.create_publisher(
            String, f"/{self.drone_id}/mission_cmd", 10)

        # Final occupancy grid (for scoring)
        self.pub_grid = self.create_publisher(
            OccupancyGrid, f"/{self.drone_id}/safe_path_grid", 10)

        # Task announcements (path verify)
        self.pub_announce = self.create_publisher(
            TaskAnnounce, "/team/task_announce", 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            PoseBeacon, "/team/pose_beacon",
            self._on_pose_beacon, 20)

        self.create_subscription(
            MineDelta, "/team/mine_delta",
            self._on_mine_delta, 20)

        self.create_subscription(
            TaskResult, "/team/task_result",
            self._on_task_result, 10)

        # ── Timers ────────────────────────────────────────────────────────────
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f"[{self.drone_id}] mission_logic_node ready | "
            f"sector={self.sector}")

    # ── Team awareness ────────────────────────────────────────────────────────

    def _on_pose_beacon(self, msg: PoseBeacon):
        if msg.drone_id == self.drone_id:
            return
        self.team_states[msg.drone_id] = msg.state
        self.team_last_seen[msg.drone_id] = time.monotonic()

    # ── Belief mirroring ──────────────────────────────────────────────────────

    def _on_mine_delta(self, msg: MineDelta):
        for b in msg.beliefs:
            existing = self.mine_beliefs.get(b.mine_id)
            if existing is None or b.seq > existing.get("seq", -1):
                self.mine_beliefs[b.mine_id] = {
                    "mine_id":    b.mine_id,
                    "x": b.x, "y": b.y,
                    "confidence": b.confidence,
                    "status":     b.status,
                    "seq":        b.seq,
                }

    def _on_task_result(self, msg: TaskResult):
        if msg.task_id == self._path_task_id and msg.outcome == "confirmed":
            self.path_verified = True
            self.get_logger().info(
                f"[{self.drone_id}] Path verification confirmed!")

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _time_remaining(self) -> float:
        if self.mission_start is None:
            return self.mission_duration
        elapsed = time.monotonic() - self.mission_start
        return max(0.0, self.mission_duration - elapsed)

    def _all_drones_ready(self) -> bool:
        """True if we've heard from all expected drones recently (< 3s ago)."""
        now = time.monotonic()
        active = {did for did, t in self.team_last_seen.items()
                  if now - t < 3.0}
        return len(active) >= self.num_drones - 1   # -1 because we don't count ourselves

    def _all_converged(self) -> bool:
        """All drones are in CONVERGE or FINISH state."""
        if not self.team_states:
            return False
        return all(s in ("CONVERGE", "FINISH")
                   for s in self.team_states.values())

    def _mine_context(self):
        all_beliefs = list(self.mine_beliefs.values())
        candidates  = [b for b in all_beliefs if b["status"] == "candidate"]
        confirmed   = [b for b in all_beliefs if b["status"] == "confirmed"]
        return len(all_beliefs), len(confirmed)

    # ── Main tick ─────────────────────────────────────────────────────────────

    def _tick(self):
        mine_count, confirmed_count = self._mine_context()

        ctx = MissionContext(
            time_remaining=self._time_remaining(),
            mine_count=mine_count,
            confirmed_count=confirmed_count,
            all_drones_ready=self._all_drones_ready(),
            path_verified=self.path_verified,
            all_converged=self._all_converged(),
            team_belief_count=len(self.mine_beliefs),
        )

        self.sm.tick(ctx)

        # Per-state directives
        state = self.sm.state

        if state == "SURVEY":
            if self.mission_start is None:
                self.mission_start = time.monotonic()
            self._cmd_survey()

        elif state == "VERIFY_TAG":
            self._cmd_verify_tag()

        elif state == "PATH_VERIFY":
            self._cmd_path_verify()

        elif state == "CONVERGE":
            self._cmd_converge()

        elif state == "FINISH":
            self._cmd_finish()

    # ── State directives ──────────────────────────────────────────────────────

    def _cmd_survey(self):
        """Tell explorer to sweep our assigned sector."""
        cmd = json.dumps({
            "cmd":   "SWEEP_SECTOR",
            "x_min": self.sector["x_min"],
            "x_max": self.sector["x_max"],
            "y_min": self.sector["y_min"],
            "y_max": self.sector["y_max"],
        })
        self._publish_cmd(cmd)

    def _cmd_verify_tag(self):
        """
        For each unconfirmed candidate, the task node handles announcing
        VERIFY_TAG tasks.  Mission node just ensures explorer stays in
        'ready for tasking' mode.
        """
        cmd = json.dumps({"cmd": "STANDBY_FOR_TASK"})
        self._publish_cmd(cmd)

    def _cmd_path_verify(self):
        """Announce a PATH_VERIFY task once, then wait."""
        if not self._path_task_announced:
            self._path_task_announced = True
            self._path_task_id = f"path_verify_{self.drone_id}_0"

            msg = TaskAnnounce()
            msg.task_id        = self._path_task_id
            msg.task_type      = "PATH_VERIFY"
            msg.announcer_id   = self.drone_id
            msg.target_x       = self.sector["cx"]
            msg.target_y       = self.sector["cy"]
            msg.priority       = 1.0
            msg.claim_window_s = 3.0
            msg.stamp          = self.get_clock().now().to_msg()
            self.pub_announce.publish(msg)
            self.get_logger().info(
                f"[{self.drone_id}] Announced PATH_VERIFY task")

        cmd = json.dumps({"cmd": "AWAIT_PATH_VERIFY"})
        self._publish_cmd(cmd)

    def _cmd_converge(self):
        """
        Publish the occupancy grid once, then tell explorer to hold position.
        """
        if not self._grid_published:
            self._publish_occupancy_grid()
            self._grid_published = True

        cmd = json.dumps({"cmd": "HOLD_POSITION"})
        self._publish_cmd(cmd)

    def _cmd_finish(self):
        cmd = json.dumps({"cmd": "LAND_AND_SUBMIT"})
        self._publish_cmd(cmd)
        self.get_logger().info(f"[{self.drone_id}] FINISH — landing")

    # ── Occupancy grid builder ─────────────────────────────────────────────────

    def _publish_occupancy_grid(self):
        """
        Build a simple occupancy grid:
          -1 = unknown
           0 = safe (no mine)
         100 = mine confirmed
        """
        resolution = 0.5   # metres per cell
        width_cells  = int(self.sector["x_max"] - self.sector["x_min"]) * 2
        height_cells = int(self.sector["y_max"] - self.sector["y_min"]) * 2

        grid = OccupancyGrid()
        grid.header.stamp    = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"
        grid.info.resolution = resolution
        grid.info.width      = width_cells
        grid.info.height     = height_cells
        grid.info.origin.position.x = self.sector["x_min"]
        grid.info.origin.position.y = self.sector["y_min"]

        # Default: all safe
        data = [0] * (width_cells * height_cells)

        # Mark confirmed mines as occupied
        for b in self.mine_beliefs.values():
            if b["status"] == "confirmed":
                cx = int((b["x"] - self.sector["x_min"]) / resolution)
                cy = int((b["y"] - self.sector["y_min"]) / resolution)
                if 0 <= cx < width_cells and 0 <= cy < height_cells:
                    data[cy * width_cells + cx] = 100

        grid.data = data
        self.pub_grid.publish(grid)
        self.get_logger().info(
            f"[{self.drone_id}] Published occupancy grid "
            f"({width_cells}x{height_cells} cells)")

    # ── Utils ─────────────────────────────────────────────────────────────────

    def _publish_cmd(self, cmd_json: str):
        msg = String()
        msg.data = cmd_json
        self.pub_mission_cmd.publish(msg)

    def _on_state_transition(self, from_state: str, to_state: str):
        self.get_logger().info(
            f"[{self.drone_id}] State: {from_state} → {to_state} "
            f"(t_remaining={self._time_remaining():.0f}s)")


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = MissionLogicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
