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
                         ┌───────────────────────────────┐
                         │           Gazebo              │
                         │   (world + physics + sensors) │
                         └──────────────┬────────────────┘
                                        │
                                pose / sensor topics
                                        │
────────────────────────────────────────────────────────────────────────────────────
                    ROS2 / FastDDS P2P Network (Shared Topics)
────────────────────────────────────────────────────────────────────────────────────

     /team/pose_beacon
     /team/sync_hello
     /team/sync_ack
     /team/mine_delta
     /team/tile_delta (optional)
     /team/task_announce
     /team/task_claim
     /team/task_result


 ┌───────────────────┐    ┌───────────────────┐
 │      DRONE 1      │    │      DRONE 2      │
 │                   │    │                   │
 │  explorer_node    │    │  explorer_node    │
 │  (frontier/A*)    │    │  (frontier/A*)    │
 │        │          │    │        │          │
 │        ▼          │    │        ▼          │
 │  p2p_task_node    │◀──▶│  p2p_task_node    │
 │ (task auction)    │    │ (task auction)    │
 │        │          │    │        │          │
 │        ▼          │    │        ▼          │
 │   p2p_sync_node   │◀──▶│   p2p_sync_node   │
 │ (sync window +    │    │ (sync window +    │
 │  map fusion)      │    │  map fusion)      │
 └───────────────────┘    └───────────────────┘
          │                         │
          │                         │
 ┌───────────────────┐    ┌───────────────────┐
 │      DRONE 3      │    │      DRONE 4      │
 │                   │    │                   │
 │  explorer_node    │    │  explorer_node    │
 │        │          │    │        │          │
 │        ▼          │    │        ▼          │
 │  p2p_task_node    │◀──▶│  p2p_task_node    │
 │        │          │    │        │          │
 │        ▼          │    │        ▼          │
 │   p2p_sync_node   │◀──▶│   p2p_sync_node   │
 │                   │    │                   │
 └───────────────────┘    └───────────────────┘


Legend
------
explorer_node     : per-drone exploration (frontier + planning)
p2p_sync_node     : map sharing (neighbor sync + diffusion)
p2p_task_node     : decentralized task auction
```

## Challenge
We decided to use a rule-based approach (eg. predifined rules, cost functions) for the multi-agent system, but there are very few relevant papers to reference, and most of them are outdated.
On the other hand, adopting a learning-based method is not very feasible for a minimum viable product (MVP), and it would significantly increase the complexity, also only four drones is not ideal to use GNN.
Therefore, we have to rely on scattered online resources to piece things together. 
