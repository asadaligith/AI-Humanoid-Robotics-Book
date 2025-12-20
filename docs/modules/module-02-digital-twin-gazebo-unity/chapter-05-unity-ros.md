---
title: 'Chapter 5: Unity Visualization with ROS'
description: 'Connect Unity for photorealistic robot visualization using ROS-TCP Connector'
sidebar_position: 5
---

# Chapter 5: Unity Visualization with ROS

## Learning Objectives

1. **Install** Unity 2022.3 LTS and Unity Robotics Hub
2. **Import** robot URDF into Unity
3. **Connect** Unity to ROS 2 via TCP
4. **Synchronize** joint states between ROS and Unity

## Why Unity?

Unity provides photorealistic rendering for:
- Stakeholder demonstrations
- Perception training (synthetic images)
- VR/AR applications

## Setup Unity Robotics Hub

### Install Unity

Download Unity Hub and Unity 2022.3 LTS from https://unity.com/download

### Import Robotics Packages

In Unity Editor:
1. Window → Package Manager
2. Add package from git URL:
   - `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
   - `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`

## Importing URDF

1. Assets → Import Robot from URDF
2. Select your humanoid.urdf file
3. Configure import settings:
   - Mesh Decomposer: VHACD
   - Axis Type: Y Axis
   - Orient to Identity: Checked

## ROS-TCP Connector Setup

### Unity Side

Create GameObject with `ROS Connection` component:
- ROS IP Address: `127.0.0.1` (localhost)
- ROS Port: `10000`
- Protocol: ROS 2

### ROS 2 Side

```bash
# Install TCP endpoint
pip install ros-tcp-endpoint

# Run endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

## Subscribing to Joint States

Unity C# script:

```csharp
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;

public class JointStateSubscriber : MonoBehaviour
{
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>(
            "/joint_states",
            UpdateJointStates
        );
    }

    void UpdateJointStates(JointStateMsg msg)
    {
        for (int i = 0; i < msg.name.Length; i++)
        {
            // Update Unity articulation bodies
        }
    }
}
```

## Exercise

**Task**:
1. Import your humanoid URDF into Unity
2. Set up ROS-TCP connection
3. Publish joint states from ROS 2
4. Verify Unity robot mirrors ROS joint positions

**Code Example**: See `examples/module-02-digital-twin/example-03-unity-ros/`

## Summary

Unity provides photorealistic visualization by subscribing to ROS 2 topics via TCP.

**Next**: [Chapter 6: Physics Parameter Tuning](./chapter-06-physics-tuning.md)

---

**Reading Time**: 25 minutes
**Hands-On Time**: 60 minutes
