---
title: 'Chapter 6: Physics Parameter Tuning'
description: 'Calibrate simulation physics for realistic robot behavior'
sidebar_position: 6
---

# Chapter 6: Physics Parameter Tuning

## Learning Objectives

1. **Adjust** friction and contact parameters for realistic locomotion
2. **Tune** joint damping and limits
3. **Configure** solver parameters for stability
4. **Validate** simulation accuracy against real-world data

## Friction Tuning

### Surface Properties

```xml
<gazebo reference="foot_link">
  <mu1>0.8</mu1>  <!-- Friction coefficient (direction 1) -->
  <mu2>0.8</mu2>  <!-- Friction coefficient (direction 2) -->
  <kp>1000000.0</kp>  <!-- Contact stiffness -->
  <kd>1.0</kd>  <!-- Contact damping -->
  <minDepth>0.001</minDepth>  <!-- Minimum penetration depth -->
</gazebo>
```

**Typical Values**:
- Rubber on concrete: μ = 0.8-1.0
- Metal on metal: μ = 0.15-0.25
- Ice: μ = 0.02-0.05

## Joint Dynamics

### Damping and Friction

```xml
<joint name="knee" type="revolute">
  <dynamics damping="0.7" friction="0.1"/>
  <limit effort="100" velocity="5.0" lower="-1.57" upper="0"/>
</joint>
```

**Guidelines**:
- Higher damping = slower joint movement
- Joint friction opposes motion (energy loss)
- Tune by comparing sim vs. real joint velocities

## Physics Engine Tuning

```xml
<physics type="bullet">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <max_contacts>20</max_contacts>

  <bullet>
    <solver>
      <type>sequential_impulse</type>
      <iters>50</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </bullet>
</physics>
```

**Key Parameters**:
- `iters`: Solver iterations (higher = more accurate but slower)
- `cfm`: Constraint force mixing (softer constraints)
- `erp`: Error reduction parameter (penetration correction)

## Validation Workflow

1. **Collect Real Data**: Measure joint trajectories on physical robot
2. **Run Simulation**: Execute same commands in Gazebo
3. **Compare Metrics**:
   - Joint position error (RMSE)
   - Velocity profiles
   - Contact forces
4. **Iterate**: Adjust parameters, repeat

## Exercise

**Task**: Tune friction for bipedal walking:
1. Create flat ground with μ = 0.8
2. Make humanoid take 5 steps
3. Measure slip distance (should be &lt;5cm per step)
4. Adjust friction if slipping occurs

## Summary

Physics tuning minimizes the reality gap between simulation and hardware.

**Next**: [Chapter 7: Module Summary](./chapter-07-summary.md)

---

**Reading Time**: 20 minutes
**Hands-On Time**: 45 minutes
