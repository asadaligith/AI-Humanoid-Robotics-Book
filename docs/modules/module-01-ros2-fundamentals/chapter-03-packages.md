---
title: 'Chapter 3: Creating ROS 2 Packages'
description: 'Learn how to structure ROS 2 packages for modular and maintainable robotics code'
sidebar_position: 3
---

# Chapter 3: Creating ROS 2 Packages

## Learning Objectives

By the end of this chapter, you will be able to:

1. **Create** Python and C++ ROS 2 packages using standard templates
2. **Configure** package dependencies and metadata
3. **Build** packages using colcon build system
4. **Organize** code following ROS 2 best practices

## Introduction

A ROS 2 package is the atomic unit of software organization. Packages group related nodes, messages, services, and configuration files, making code reusable and shareable across projects[^1].

[^1]: ROS 2 Documentation. "Creating a Package." https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html

## Package Structure

### Python Package Layout

```
my_robot_controller/
├── package.xml          # Package metadata
├── setup.py            # Python setup script
├── setup.cfg           # Configuration
├── my_robot_controller/
│   ├── __init__.py
│   ├── controller_node.py
│   └── utils.py
├── resource/
│   └── my_robot_controller
├── test/
│   └── test_controller.py
└── README.md
```

### C++ Package Layout

```
my_robot_driver/
├── package.xml
├── CMakeLists.txt
├── include/
│   └── my_robot_driver/
│       └── driver.hpp
├── src/
│   ├── driver.cpp
│   └── main.cpp
└── README.md
```

## Creating a Python Package

### Using ros2 pkg create

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create Python package
ros2 pkg create --build-type ament_python my_robot_controller

# Directory structure created automatically
```

### Package Dependencies

Edit `package.xml`:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_robot_controller</name>
  <version>0.1.0</version>
  <description>Robot controller package</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>

  <test_depend>pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Entry Points Configuration

Edit `setup.py`:

```python
from setuptools import setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Robot controller package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = my_robot_controller.controller_node:main',
        ],
    },
)
```

## Building Packages

### Using colcon

```bash
# Build all packages in workspace
cd ~/ros2_ws
colcon build

# Build specific package
colcon build --packages-select my_robot_controller

# Build with symlink (for faster iteration)
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### Running Package Nodes

```bash
# Run node using package name and entry point
ros2 run my_robot_controller controller
```

## Creating a C++ Package

```bash
# Create C++ package with dependencies
ros2 pkg create --build-type ament_cmake \
    --dependencies rclcpp std_msgs geometry_msgs \
    my_robot_driver
```

### CMakeLists.txt Configuration

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_driver)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Add executable
add_executable(driver_node src/driver.cpp src/main.cpp)
ament_target_dependencies(driver_node rclcpp std_msgs)

# Install executable
install(TARGETS driver_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

## Best Practices

### Package Organization

✅ **DO**:
- One package per logical component (perception, navigation, etc.)
- Include README.md with usage instructions
- Add launch files in `launch/` directory
- Store configuration files in `config/`

❌ **DON'T**:
- Put all code in a single package
- Mix Python and C++ in one package
- Omit package documentation

### Naming Conventions

- Package names: lowercase with underscores (`my_robot_controller`)
- Node names: descriptive and unique (`obstacle_detector`)
- Topic names: hierarchical (`/robot/sensors/lidar`)

## Exercise

**Task**: Create a ROS 2 package called `battery_monitor` with:
1. A publisher node that broadcasts battery voltage (simulated 12V ± random noise)
2. A subscriber node that logs battery status and warns if voltage < 11V
3. Proper package.xml with dependencies
4. setup.py with entry points for both nodes

**Build and run** your package to verify it works.

**Code Example**: See `examples/module-01-ros2-fundamentals/example-02-package/`

## Summary

In this chapter, you learned:
- ROS 2 package structure for Python and C++
- How to create packages using ros2 pkg create
- Building packages with colcon
- Best practices for package organization

**Next**: [Chapter 4: Services for Synchronous Communication](./chapter-04-services.md)

## Further Reading

- [ROS 2 Package Creation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [colcon Documentation](https://colcon.readthedocs.io/)
- [ament Build System](https://design.ros2.org/articles/ament.html)

---

**Estimated Reading Time**: 20 minutes
**Hands-On Time**: 40 minutes
