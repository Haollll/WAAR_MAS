# Multi-Agent Coordination Framework

## Overview

This repository implements the **team-level coordination and safe path planning system** for IARC Mission 10.

We are currently targeting a **4 Explorer configuration (MVP)** using:
- Rule-based method
- ROS2 (Roxy)
- Gazebo simulation
- GPS-based arena reference (~1.5m resolution)
- Deployment-ready architecture for Raspberry Pi

The objective is to enable 4 drones to:

- Divide the exploration area
- Perform parallel coverage
- Avoid collisions
- Handle drone failures
- Maintain coverage despite GPS drift
- Produce rolling team-level outputs (collective map + safe paths)


# Grid
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
