# Multi-Agent Coordination Framework

## Overview

This repository implements the **team-level coordination and safe path planning system** for IARC Mission 10.

We are currently targeting a **4 Explorer configuration (MVP)** using:
- Rule-based method
- ROS2 (Roxy)
- Gazebo simulation
- GPS-based arena reference (~1.5m resolution)
- Deployment-ready architecture for Raspberry Pi

The objective of MAS system is to enable 4 drones to:

- Region partitioning (strip assignment)
- Dynamic reassignment
- Collision avoidance between drones
- Failure detection & recovery
- Drift correction for coverage maintenance
- Rolling safe-path generation
- Operational mode switching (Mission reassign)

## Core Idea
The coordination strategy is based on a fully decentralized multi-agent architecture.
- Information sharing follows a neighbor-based diffusion process similar to gossip protocols in distributed systems.
- Task allocation is implemented using a distributed auction mechanism, where agents independently compute task costs and compete through a claim process.
- Local decision making is implemented using a state machine / behavior-based control architecture, allowing reactive and robust operation without centralized planning.

## Flow
MVP
```
+-------------------------------------------------------------------------------+
|                               Mission Layer (Per-Drone)                       |
|   Goal: maximize coverage + produce reliable mine beliefs + endgame verify    |
|                                                                               |
|   State Machine: BOOT -> SURVEY -> VERIFY_TAG -> PATH_VERIFY -> CONVERGE -> FINISH
|                                                                               |
|   Behavior Tree (priority):                                                   |
|     1) Safety / Collision Guard                                               |
|     2) Geofence + Time Guard                                                  |
|     3) Failure Monitor                                                        |
|     4) Task Executor (VERIFY / RESCAN)                                        |
|     5) Exploration Policy (frontier)                                          |
|     6) P2P Sync Manager (neighbor sessions)                                   |
+-------------------------------------------------------------------------------+
                 ^                     ^                     ^
                 | task_cmd            | local belief         | status
                 | (interrupt)         | (fused)              | (health)
                 |                     |                     |
+----------------+---------------------+---------------------+------------------+
|                         Coordination Layer (Fully P2P)                        |
|     (No central controller; every drone runs identical coordination logic)    |
|                                                                               |
|  A) Neighbor Graph (from pose beacons)                                        |
|     - edge(i,j) if dist(i,j) < R_enter; remove if dist(i,j) > R_exit          |
|                                                                               |
|  B) Sync Window Protocol (Scheme B)                                           |
|     - hello/ack handshake -> SYNC_ACTIVE(T_sync)                              |
|     - exchange missing deltas (mine/tile) with seq + TTL + dedup cache        |
|                                                                               |
|  C) Decentralized Task Auction                                                |
|     - announce(task) -> claims(cost) -> winner executes -> result broadcast   |
|     - tasks: VERIFY_TAG, RESCAN, BECOME_VERIFIER (endgame)                    |
|                                                                               |
|  Shared ROS2 Topics (FastDDS):                                                |
|    /team/pose_beacon   /team/sync_hello  /team/sync_ack                       |
|    /team/mine_delta    /team/tile_delta (opt)                                 |
|    /team/task_announce /team/task_claim  /team/task_result                    |
+-------------------------------------------------------------------------------+
                 ^                     ^                     ^
                 | pose/sensors         | detections           | map updates
                 |                      |                      |
+----------------+---------------------+---------------------+-----------------+
|                         Per-Drone Autonomy Layer (x4)                        |
|                                                                              |
|  explorer_node (Kevin)                                                       |
|   - SLAM/VIO (or sim pose)                                                   |
|   - occupancy grid update                                                    |
|   - frontier selection (local belief-driven)                                 |
|   - A* to chosen frontier                                                    |
|   - tracking                                                                 |
|                                                                              |
|  Optional local mapping/detector                                             |
|   - mine candidate detection -> triggers task announce                       |
+------------------------------------------------------------------------------+
                 ^
                 |
+----------------+-------------------------------------------------------------+
|                               Simulation Layer                               |
|                           Gazebo + ROS2 Bridges                              |
|                  (world models, mines, obstacles, sensors)                   |
+------------------------------------------------------------------------------+
```

## ROS2 Architecture
```
                         +-------------------------------+
                         |           Gazebo              |
                         |   (world + physics + sensors) |
                         +--------------+----------------+
                                        |
                                pose / sensor topics
                                        |
--------------------------------------------------------------------------------
                    ROS2 / FastDDS P2P Network (Shared Topics)
--------------------------------------------------------------------------------

     /team/pose_beacon
     /team/sync_hello
     /team/sync_ack
     /team/mine_delta
     /team/tile_delta (optional)
     /team/task_announce
     /team/task_claim
     /team/task_result


 +-------------------+    +-------------------+
 |      DRONE 1      |    |      DRONE 2      |
 |                   |    |                   |
 |  explorer_node    |    |  explorer_node    |
 |  (frontier/A*)    |    |  (frontier/A*)    |
 |        |          |    |        |          |
 |        v          |    |        v          |
 |  p2p_task_node    |<-->|  p2p_task_node    |
 | (task auction)    |    | (task auction)    |
 |        |          |    |        |          |
 |        v          |    |        v          |
 |   p2p_sync_node   |<-->|   p2p_sync_node   |
 | (sync window +    |    | (sync window +    |
 |  map fusion)      |    |  map fusion)      |
 +-------------------+    +-------------------+
          |                         |
          |                         |
 +-------------------+    +-------------------+
 |      DRONE 3      |    |      DRONE 4      |
 |                   |    |                   |
 |  explorer_node    |    |  explorer_node    |
 |        |          |    |        |          |
 |        v          |    |        v          |
 |  p2p_task_node    |<-->|  p2p_task_node    |
 |        |          |    |        |          |
 |        v          |    |        v          |
 |   p2p_sync_node   |<-->|   p2p_sync_node   |
 |                   |    |                   |
 +-------------------+    +-------------------+


Legend
------
explorer_node     : per-drone exploration (frontier + planning)
p2p_sync_node     : map sharing (neighbor sync + diffusion)
p2p_task_node     : decentralized task auction
```

## Package Summary

| Package | Key Files | Responsibility |
|---|---|---|
| `mas_interfaces` | 7 `.msg` files | Custom ROS2 message definitions for all team communication |
| `mas_sync` | `p2p_sync_node.py`, `belief_fusion.py` | Neighbor graph, Scheme B sync window, mine belief fusion |
| `mas_task` | `p2p_task_node.py`, `auction_manager.py` | P2P task auction — bid, claim, resolve |
| `mas_mission` | `mission_logic_node.py`, `state_machine.py`, `bt_runner.py` | State machine, BT 6-layer priority selector, PATH_VERIFY role split, CONVERGE rescan |

## Custom Messages

| Message | Description |
|---|---|
| `MineBelief` | Single mine observation: position, confidence, status (`candidate`/`confirmed`/`rejected`), sequence number |
| `PoseBeacon` | Periodic drone pose broadcast (5 Hz): position, heading, battery, current mission state |
| `MineDelta` | Batch of `MineBelief` updates with TTL for multi-hop relay; used in sync windows |
| `SyncHello` | Initiates a sync window when a neighbour enters range; carries sender belief version |
| `SyncAck` | Acknowledges a `SyncHello` and opens the active sync window |
| `TaskAnnounce` | Broadcasts a new task to be auctioned: type, target position, priority, claim window |
| `TaskClaim` | A drone's bid on an open auction: task ID, bidder ID, computed cost |
| `TaskResult` | Winner's outcome report after task execution: outcome, mine ID, updated confidence |

## Packages

| Package | Type | Description |
|---|---|---|
| `mas_interfaces` | ament_cmake | Custom ROS2 message definitions for all team communication |
| `mas_task` | ament_python | P2P task auction node — distributed task allocation via announce/claim/result |
| `mas_sync` | ament_python | P2P sync node — Scheme B sync window + mine belief fusion |
| `mas_mission` | ament_python | Mission logic + state machine + BT priority selector (`bt_runner.py`) — BOOT -> SURVEY -> VERIFY_TAG -> PATH_VERIFY -> CONVERGE -> FINISH |

## Dev Setup & Commands

### VSCode
- Always open `/Users/lihao/ros2_ws/src/mas_coordinator` in Claude Code
- `Downloads/mas_coordinator` is a separate copy — changes there **won't reach Docker**
- If Claude Code made changes in the wrong folder, use diff + cp to sync

### Docker
```bash
# Start existing container (don't create new one)
docker start mas_dev
docker exec -it mas_dev bash

# Auto-source on every terminal open (run once)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# If source is needed manually
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
```

### Build
```bash
cd /ros2_ws
colcon build --packages-select mas_interfaces   # always build this first
source install/setup.bash
colcon build --packages-select mas_sync mas_task mas_mission
source install/setup.bash
```

### Launch
```bash
ros2 launch mas_mission team_launch.py
```

### Tests
```bash
cd /ros2_ws/src/mas_coordinator
python3 -m pytest tests/ -v
# Expected: 77 passed, 4 skipped
```

### Sync files from Downloads to ros2_ws (if needed)
```bash
diff -rq /Users/lihao/Downloads/mas_coordinator /Users/lihao/ros2_ws/src/mas_coordinator --exclude="__pycache__" --exclude="*.pyc"
cp /Users/lihao/Downloads/mas_coordinator/<file> /Users/lihao/ros2_ws/src/mas_coordinator/<file>
```

## Explorer Integration (Kevin)

- **Kevin's repo:** `waar_autonomy` (private)
- **Adapter file:** `waar_autonomy/src/adapters/ros2_adapter.py`

### Topics

| Direction | Topic | Message Type |
|---|---|---|
| Subscribes | `/{drone_id}/task_cmd` | `std_msgs/String` (JSON) |
| Subscribes | `/{drone_id}/mission_cmd` | `std_msgs/String` (JSON) |
| Publishes | `/{drone_id}/pose` | `geometry_msgs/PoseStamped` |
| Publishes | `/{drone_id}/mine_candidates` | `mas_interfaces/MineBelief` |

### Coordinate Conversion

Kevin uses `(row, col)` integers; MAS uses `(x, y)` floats.

```
x = col * CELL_SIZE
y = row * CELL_SIZE
CELL_SIZE = 1.0 m  (default)
```

### Integration Status

Kevin needs to fill in the `_step()` placeholder in `ros2_adapter.py`.

## Challenge
We decided to use a rule-based approach (eg. predifined rules, cost functions) for the multi-agent system, but there are very few relevant papers to reference, and most of them are outdated.
On the other hand, adopting a learning-based method is not very feasible for a minimum viable product (MVP), and it would significantly increase the complexity, also only four drones is not ideal to use GNN.
Therefore, we have to rely on scattered online resources to piece things together.
