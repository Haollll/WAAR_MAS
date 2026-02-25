# WAAR_MAS

TEAM LEVEL - rule-based(MVP)
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
