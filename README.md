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

## Grid
MVP
```
┌───────────────────────────────────────────────────────────────────────────────┐
│                               Mission Layer (Per-Drone)                       │
│   Goal: maximize coverage + produce reliable mine beliefs + endgame verify    │
│                                                                               │
│   State Machine: BOOT → SURVEY → VERIFY_TAG → CONVERGE → FINISH               │
│                                                                               │
│   Behavior Tree (priority):                                                   │
│     1) Safety / Collision Guard                                               │
│     2) Geofence + Time Guard                                                  │
│     3) Failure Monitor                                                        │
│     4) Task Executor (VERIFY / RESCAN)                                        │
│     5) Exploration Policy (frontier)                                          │
│     6) P2P Sync Manager (neighbor sessions)                                   │
└───────────────────────────────────────────────────────────────────────────────┘
                 ▲                     ▲                     ▲
                 │ task_cmd            │ local belief         │ status
                 │ (interrupt)         │ (fused)              │ (health)
                 │                     │                     │
┌────────────────┴─────────────────────┴─────────────────────┴──────────────────┐
│                         Coordination Layer (Fully P2P)                        │
│     (No central controller; every drone runs identical coordination logic)    │
│                                                                               │
│  A) Neighbor Graph (from pose beacons)                                        │
│     - edge(i,j) if dist(i,j) < R_enter; remove if dist(i,j) > R_exit          │
│                                                                               │
│  B) Sync Window Protocol (Scheme B)                                           │
│     - hello/ack handshake → SYNC_ACTIVE(T_sync)                               │
│     - exchange missing deltas (mine/tile) with seq + TTL + dedup cache        │
│                                                                               │
│  C) Decentralized Task Auction                                                │
│     - announce(task) → claims(cost) → winner executes → result broadcast      │
│     - tasks: VERIFY_TAG, RESCAN, BECOME_VERIFIER (endgame)                    │
│                                                                               │
│  Shared ROS2 Topics (FastDDS):                                                │
│    /team/pose_beacon   /team/sync_hello  /team/sync_ack                       │
│    /team/mine_delta    /team/tile_delta (opt)                                 │
│    /team/task_announce /team/task_claim  /team/task_result                    │
└───────────────────────────────────────────────────────────────────────────────┘
                 ▲                     ▲                     ▲
                 │ pose/sensors         │ detections           │ map updates
                 │                      │                      │
┌────────────────┴─────────────────────┴─────────────────────┴─────────────────┐
│                         Per-Drone Autonomy Layer (x4)                        │
│                                                                              │
│  explorer_node (Kevin)                                                       │
│   - SLAM/VIO (or sim pose)                                                   │
│   - occupancy grid update                                                    │
│   - frontier selection (local belief-driven)                                 │
│   - A* to chosen frontier                                                    │
│   - tracking                                                                 │
│                                                                              │
│  Optional local mapping/detector                                             │
│   - mine candidate detection → triggers task announce                        │
└──────────────────────────────────────────────────────────────────────────────┘
                 ▲
                 │
┌────────────────┴─────────────────────────────────────────────────────────────┐
│                               Simulation Layer                               │
│                           Gazebo + ROS2 Bridges                              │
│                  (world models, mines, obstacles, sensors)                   │
└──────────────────────────────────────────────────────────────────────────────┘
```
## ROS2 archeitecture
```
iarc_ws/
└── src/
    ├── mission10_msgs/                         (shared interfaces)
    │   ├── msg/
    │   │   ├── PoseLite.msg
    │   │   ├── Coverage.msg
    │   │   ├── Assignment.msg
    │   │   ├── MineDet.msg              (
    │   │   ├── MineDetArray.msg    
    │   │   ├── MineBelief.msg           
    │   │   ├── MineBeliefArray.msg      
    │   │   ├── Path2D.msg               
    │   │   └── Path2DArray.msg        
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── mission10_agent/                        (per-drone autonomy, Kevin)
    │   ├── mission10_agent/
    │   │   ├── explorer_node.py     （ASSUMED)
    │   │   ├── frontier.py          （ASSUMED)
    │   │   ├── planner_astar.py     （ASSUMED)
    │   │   └── tracker.py           （ASSUMED)
    │   ├── launch/
    │   │   └── explorer.launch.py
    │   ├── setup.py
    │   └── package.xml
    │
    ├── mission10_coordinator/                  (team-level)
    │   ├── mission10_coordinator/
    │   │   ├── team_coordinator_node.py       
    │   │   ├── strip_partition.py
    │   │   ├── safety_shield.py
    │   │   ├── fault_manager.py
    │   │   ├── belief_fusion_node.py      
    │   │   └── team_planner_node.py        
    │   ├── launch/
    │   │   └── mission10_sim.launch.py
    │   ├── setup.py
    │   └── package.xml
    │
    └── mission10_sim/                          (gazebo integration)
        ├── worlds/
        │   └── mission10.world  (or .sdf)
        ├── models/
        │   ├── mines/
        │   ├── trees/
        │   └── obstacles/
        ├── config/    
        └── launch/
            └── sim.launch.py                   (spawn 4 drones)
```

## Challenge
We decided to use a rule-based approach (eg. predifined rules, cost functions) for the multi-agent system, but there are very few relevant papers to reference, and most of them are outdated.
On the other hand, adopting a learning-based method is not very feasible for a minimum viable product (MVP), and it would significantly increase the complexity, also only four drones is not ideal to use GNN.
Therefore, we have to rely on scattered online resources to piece things together. 
