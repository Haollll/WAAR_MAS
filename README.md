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

## ROS2 archeitecture

## Challenge
We decided to use a rule-based approach for the multi-agent system, but there are very few relevant papers to reference, and most of them are outdated.
On the other hand, adopting a learning-based method is not very feasible for a minimum viable product (MVP), and it would significantly increase the complexity.
Therefore, we have to rely on scattered online resources and LLMs to piece things together. 

## Grid
MVP
```

TEAM LEVEL - rule-based
┌──────────────────────────────────────────────┐
│ Strip Coordinator (macro assignment)         │
│ - 4 strips fixed partition (A/B/C/D)         │
│ - progress monitor + fault takeover          │
│ - collision/frequency/safety constraints     │
└───────────────┬──────────────────────────────┘
                │ Assignment: which strip, constraints
                ▼
PER-DRONE LEVEL (Kevin)
┌──────────────────────────────────────────────┐
│                ┌────────────────────┐        │
│                │    Mission Layer   │        │ 
│                │  (Build Safe Path) │        │
│                └─────────┬──────────┘        │ 
│                          │                   │
│                ┌─────────▼──────────┐        │ 
│                │ Corridor Planner   │        │
│                │  (Human-safe path) │        │
│                └─────────┬──────────┘        │
│                          │                   │
│                ┌─────────▼──────────┐        │
│                │ Exploration Policy │        │
│                │ (Which macro cell?)│        │
│                └─────────┬──────────┘        │
│                          │                   │
│                ┌─────────▼──────────┐        │
│                │  Belief / Risk Map │        │
│                │  mine density      │        │
│                │  obstacle density  │        │
│                │  uncertainty       │        │
│                └─────────┬──────────┘        │
│                          │                   │
│                ┌─────────▼──────────┐        │
│                │   Sensor Layer     │        │
│                │ (10 sec scanning)  │        │
│                └─────────┬──────────┘        │
│                          │                   │
│                ┌─────────▼──────────┐        │
│                │  Ground Truth Map  │        │
│                └────────────────────┘        │
│                                              │
└───────────────┬──────────────────────────────┘
                │ Local belief/coverage summary
                ▼
TEAM OUTPUT
┌──────────────────────────────────────────────┐
│ Collective Fusion (merge 4 drones)           │
│  ↓                                           │
│ Corridor Planner / Mission Layer (team)      │
│  ↓                                           │
│ possible paths + optimum safe path           │
└──────────────────────────────────────────────┘
