# Example 02: Creating a ROS 2 Package

## Learning Objectives

By completing this example, you will:
- Understand ROS 2 package structure and organization
- Create a proper ROS 2 Python package using `ros2 pkg create`
- Configure `package.xml` for dependencies and metadata
- Set up `setup.py` for Python package installation
- Organize code into reusable modules
- Build and install packages using `colcon`

## Overview

A ROS 2 package is the basic unit of organization for ROS code. It contains nodes, libraries, configuration files, and metadata necessary for building and distributing your robotics application.

**Key Concepts:**
- **Package**: A directory containing code, data, and metadata
- **package.xml**: Manifest file with dependencies and metadata
- **setup.py**: Python package configuration (for Python packages)
- **CMakeLists.txt**: Build configuration (for C++ packages)
- **Workspace**: A directory containing one or more packages

## Prerequisites

- ROS 2 Humble installed
- Completed Example 01 (Publisher-Subscriber)
- Completed Module 01 Chapter 03

## Package Structure

```
my_robot_pkg/
├── package.xml                    # Package manifest
├── setup.py                       # Python setup configuration
├── setup.cfg                      # Setup configuration
├── resource/                      # Package marker
│   └── my_robot_pkg
├── my_robot_pkg/                  # Python module directory
│   ├── __init__.py
│   ├── publisher_node.py          # Publisher implementation
│   └── subscriber_node.py         # Subscriber implementation
└── test/                          # Unit tests
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

## Creating a Package

### Step 1: Set Up Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Source ROS 2
source /opt/ros/humble/setup.bash
```

### Step 2: Create Package

```bash
# Create a Python package with dependencies
ros2 pkg create --build-type ament_python \
  --dependencies rclpy std_msgs \
  my_robot_pkg

# Navigate into the package
cd my_robot_pkg
```

This command creates:
- `package.xml` with rclpy and std_msgs dependencies
- `setup.py` and `setup.cfg` for Python packaging
- Basic directory structure
- Template test files

### Step 3: Add Nodes to the Package

Move your Python nodes into the package module directory:

```bash
# Copy publisher from Example 01
cp /path/to/example-01/publisher.py my_robot_pkg/publisher_node.py

# Copy subscriber from Example 01
cp /path/to/example-01/subscriber.py my_robot_pkg/subscriber_node.py
```

### Step 4: Configure Entry Points in setup.py

Edit `setup.py` to register your nodes as executable scripts:

```python
from setuptools import setup

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Example ROS 2 package with publisher and subscriber',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = my_robot_pkg.publisher_node:main',
            'subscriber = my_robot_pkg.subscriber_node:main',
        ],
    },
)
```

**Entry Points Explained:**
- `publisher` = executable name (what you'll type in the terminal)
- `my_robot_pkg.publisher_node` = module path
- `main` = function to call

### Step 5: Update package.xml

Ensure your `package.xml` has the correct dependencies:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_pkg</name>
  <version>0.0.1</version>
  <description>Example ROS 2 package with publisher and subscriber</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <!-- Build tool dependency -->
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Runtime dependencies -->
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <!-- Testing dependencies -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## Building the Package

### Step 1: Build with colcon

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Build all packages
colcon build

# Build specific package
colcon build --packages-select my_robot_pkg

# Build with verbose output
colcon build --packages-select my_robot_pkg --event-handlers console_direct+
```

### Step 2: Source the Workspace

```bash
# Source the workspace overlay
source ~/ros2_ws/install/setup.bash

# Verify package is found
ros2 pkg list | grep my_robot_pkg
```

## Running Nodes from the Package

```bash
# Run the publisher
ros2 run my_robot_pkg publisher

# Run the subscriber (in another terminal)
ros2 run my_robot_pkg subscriber
```

## Package Dependencies

### Types of Dependencies

**buildtool_depend**: Tools needed to build the package
```xml
<buildtool_depend>ament_python</buildtool_depend>
```

**depend**: Runtime and build dependency (shorthand)
```xml
<depend>rclpy</depend>
```

**exec_depend**: Runtime-only dependency
```xml
<exec_depend>std_msgs</exec_depend>
```

**build_depend**: Build-time-only dependency
```xml
<build_depend>rosidl_default_generators</build_depend>
```

**test_depend**: Testing dependency
```xml
<test_depend>ament_flake8</test_depend>
```

## Workspace Overlay

ROS 2 uses an overlay system where workspaces stack on top of each other:

```
Your Workspace (~/ros2_ws)
    ↓ overlays
ROS 2 Installation (/opt/ros/humble)
```

When you source your workspace, packages in `~/ros2_ws/install` take precedence over packages in `/opt/ros/humble`.

## Best Practices

### 1. Package Naming
- Use lowercase with underscores: `my_robot_pkg`
- Be descriptive: `mobile_robot_navigation`
- Avoid generic names: `utils`, `helpers`

### 2. Code Organization
```
my_robot_pkg/
├── my_robot_pkg/
│   ├── __init__.py
│   ├── nodes/              # Node implementations
│   │   ├── __init__.py
│   │   ├── sensor_node.py
│   │   └── control_node.py
│   ├── utils/              # Utility modules
│   │   ├── __init__.py
│   │   └── math_helpers.py
│   └── config/             # Configuration classes
│       ├── __init__.py
│       └── robot_config.py
```

### 3. Version Control
Add to `.gitignore`:
```
build/
install/
log/
*.pyc
__pycache__/
```

### 4. Documentation
- Maintain a comprehensive README.md
- Document each node's purpose and parameters
- Include usage examples

## Common Commands

```bash
# Create package
ros2 pkg create --build-type ament_python --dependencies rclpy PKG_NAME

# List all packages
ros2 pkg list

# Get package information
ros2 pkg xml my_robot_pkg

# Find package installation directory
ros2 pkg prefix my_robot_pkg

# Build workspace
colcon build

# Clean build
rm -rf build/ install/ log/
colcon build

# Test package
colcon test --packages-select my_robot_pkg
```

## Exercises

### Exercise 1: Create Your Own Package
Create a package named `greeting_pkg` with a publisher that sends greeting messages.

### Exercise 2: Add a Launch File
Create a launch file that starts both publisher and subscriber together.

### Exercise 3: Multi-Package Workspace
Create a second package in the same workspace that depends on `my_robot_pkg`.

### Exercise 4: Add Configuration Files
Add a `config/` directory with YAML configuration files and load them in your nodes.

### Exercise 5: Custom Message Types
Create a package with custom message definitions (covered in advanced modules).

## Common Issues

### Issue 1: "Package not found" after building
**Solution**: Source the workspace: `source ~/ros2_ws/install/setup.bash`

### Issue 2: Entry point not working
**Solution**: Verify the entry point in `setup.py` matches your module structure and function name.

### Issue 3: "No module named 'my_robot_pkg'"
**Solution**: Ensure `__init__.py` exists in your Python module directory.

### Issue 4: Build fails with dependency errors
**Solution**: Add missing dependencies to both `package.xml` and `setup.py` (for Python dependencies).

## Key Takeaways

1. **Packages are the Unit of Organization**: All ROS 2 code lives in packages
2. **Proper Structure Matters**: Follow conventions for discoverability and maintainability
3. **Dependencies Must Be Declared**: Both in `package.xml` and `setup.py`
4. **Build with colcon**: The standard ROS 2 build tool
5. **Source Your Workspace**: Always source after building

## Next Steps

- **Example 03**: Services for request-response communication
- **Module 01 Chapter 04**: Advanced package features
- **Launch Files**: Starting multiple nodes together

## References

- [Creating a ROS 2 Package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)
- [ament_python Build Type](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Python-Documentation.html)
- [colcon Documentation](https://colcon.readthedocs.io/)
