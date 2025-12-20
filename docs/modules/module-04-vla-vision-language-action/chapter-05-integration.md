---
title: 'Chapter 5: End-to-End VLA Integration'
description: 'Combine Whisper, LLM, and ROS executor into complete VLA pipeline'
sidebar_position: 5
---

# Chapter 5: End-to-End VLA Integration

## Learning Objectives

1. **Connect** all VLA components
2. **Test** end-to-end workflows
3. **Measure** success rate and latency

## Launch File

```python
def generate_launch_description():
    return LaunchDescription([
        Node(package='vla_pipeline', executable='whisper_node'),
        Node(package='vla_pipeline', executable='llm_planner'),
        Node(package='vla_pipeline', executable='ros_executor'),
        Node(package='gazebo_ros', executable='gzserver', arguments=['vla_demo.world'])
    ])
```

## Testing

**Test Commands**:
- "Pick up the red cube"
- "Navigate to the table"
- "Place the object on the shelf"

**Success Metrics**:
- Task completion rate >60%
- End-to-end latency &lt;30s

**Code Example**: See `examples/module-04-vla/example-01-vla-pipeline/`

**Next**: [Chapter 6: Jetson Deployment](./chapter-06-jetson-deploy.md)

---

**Reading Time**: 20 minutes
**Hands-On Time**: 60 minutes
