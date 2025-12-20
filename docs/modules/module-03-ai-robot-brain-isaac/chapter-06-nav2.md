---
title: 'Chapter 6: Autonomous Navigation with Nav2'
description: 'Configure Nav2 stack for humanoid robot autonomous navigation'
sidebar_position: 6
---

# Chapter 6: Autonomous Navigation with Nav2

## Learning Objectives

1. **Configure** Nav2 for humanoid footprint
2. **Tune** planners and controllers
3. **Integrate** VSLAM with Nav2

## Nav2 Stack

Nav2 provides:
- Global planner (Dijkstra, A*, Theta*)
- Local planner (DWB, TEB)
- Costmap generation (obstacles from sensors)

**Launch**:
```bash
ros2 launch nav2_bringup navigation_launch.py
```

**Next**: [Chapter 7: Module Summary](./chapter-07-summary.md)

---

**Reading Time**: 25 minutes
**Hands-On Time**: 60 minutes
