---
title: 'Chapter 2: Gazebo Fortress Basics'
description: 'Master Gazebo interface, world creation, and basic simulation concepts'
sidebar_position: 2
---

# Chapter 2: Gazebo Fortress Basics

## Learning Objectives

1. **Navigate** the Gazebo GUI and understand its components
2. **Create** custom simulation worlds using SDF (Simulation Description Format)
3. **Insert** models and configure their properties
4. **Control** simulation playback (play, pause, reset)

## Gazebo GUI Overview

**Main Components**:
- **3D Viewport**: Visualize simulation
- **World Tree**: Hierarchical list of entities
- **Component Inspector**: View/edit properties
- **Entity Creator**: Add models, lights, sensors

## SDF World Files

SDF (Simulation Description Format) defines simulation environments[^1]:

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="simple_world">
    <!-- Physics engine -->
    <physics name="default_physics" type="bullet">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <direction>-0.5 0.1 -0.9</direction>
    </light>
  </world>
</sdf>
```

[^1]: Gazebo Documentation. "SDF Specification." http://sdformat.org/

## Adding Models

### Via GUI
1. Click "Insert" in Entity Creator
2. Select model from Fuel library
3. Click in viewport to place

### Via SDF
```xml
<model name="box">
  <pose>0 0 0.5 0 0 0</pose>
  <link name="box_link">
    <inertial>
      <mass>1.0</mass>
    </inertial>
    <collision name="collision">
      <geometry>
        <box><size>1 1 1</size></box>
      </geometry>
    </collision>
    <visual name="visual">
      <geometry>
        <box><size>1 1 1</size></box>
      </geometry>
      <material>
        <ambient>0 0 1 1</ambient>
      </material>
    </visual>
  </link>
</model>
```

## Simulation Control

```bash
# Launch world
gz sim my_world.sdf

# Headless mode (no GUI, faster)
gz sim -s my_world.sdf

# Set initial pose via CLI
gz model --spawn-file=model.sdf --model-name=my_model --pose "1 2 0.5 0 0 0"
```

## Physics Configuration

**Key Parameters**:
- `max_step_size`: Simulation timestep (default: 0.001s)
- `real_time_factor`: Speed multiplier (1.0 = real-time, 2.0 = 2x speed)
- Engine types: `bullet` (default), `ode`, `dart`

## Exercise

**Task**: Create a world file with:
- Ground plane
- One box (1m cube) at position (0, 0, 0.5)
- One sphere (0.5m radius) at position (2, 0, 1)
- Directional lighting

Save as `my_world.sdf` and launch with `gz sim my_world.sdf`.

**Code Example**: See `examples/module-02-digital-twin/example-01-gazebo-humanoid/worlds/`

## Summary

Gazebo uses SDF to define simulation worlds with models, physics, and lighting.

**Next**: [Chapter 3: Loading URDF in Gazebo](./chapter-03-urdf-gazebo.md)

---

**Reading Time**: 20 minutes
**Hands-On Time**: 40 minutes
