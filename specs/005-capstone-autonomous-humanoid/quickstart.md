# Quickstart: Running the Capstone Demo

**Estimated Time**: 15-20 minutes
**Goal**: Run the complete voice-commanded fetch-and-deliver demo in Gazebo simulation

## Prerequisites Checklist

Before starting, ensure you have:

- ✅ **Ubuntu 22.04 LTS** (native or VM recommended, WSL2 may have issues)
- ✅ **ROS 2 Humble** installed and sourced
- ✅ **Gazebo Fortress** installed (or Isaac Sim 2023.1+ with NVIDIA GPU)
- ✅ **Python 3.10+** with pip
- ✅ **Git** for cloning repository
- ✅ **OpenAI API key** for LLM planning (get from [platform.openai.com](https://platform.openai.com/api-keys))
- ✅ **Microphone** configured and accessible (test with `arecord -l`)

## Step 1: Clone Repository

```bash
# Clone the AI Humanoid Robotics Book repository
git clone https://github.com/asadaligith/AI-Humanoid-Robotics-Book.git
cd AI-Humanoid-Robotics-Book
```

**Expected Output**:
```
Cloning into 'AI-Humanoid-Robotics-Book'...
remote: Enumerating objects: ...
```

## Step 2: Install ROS 2 Dependencies

```bash
# Navigate to capstone module
cd examples/module-05-capstone

# Install ROS 2 package dependencies
rosdep install --from-paths . --ignore-src -r -y
```

**Expected Output**:
```
#All required rosdeps installed successfully
```

**Common Issue**: If `rosdep: command not found`, install it:
```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

## Step 3: Install Python Dependencies

```bash
# Install Python packages
pip install -r requirements.txt
```

**This will install**:
- openai-whisper (voice recognition)
- sounddevice (audio capture)
- openai (LLM API client)
- opencv-python (computer vision)
- jsonschema (plan validation)
- pytest (testing)

**Expected Duration**: 2-5 minutes (Whisper model downloads ~500MB)

**Troubleshooting**: If pip install fails, ensure you're using Python 3.10+:
```bash
python3 --version  # Should show 3.10 or higher
```

## Step 4: Set Environment Variables

Create a `.env` file in the capstone directory:

```bash
# Create .env file with your OpenAI API key
cat > .env <<EOF
OPENAI_API_KEY="sk-your-api-key-here"
ROS_DOMAIN_ID=42
EOF
```

**Load environment variables**:
```bash
source .env
export OPENAI_API_KEY="sk-your-api-key-here"
export ROS_DOMAIN_ID=42
```

⚠️ **IMPORTANT**: Never commit `.env` to git! It's already in `.gitignore`.

## Step 5: Build ROS 2 Workspace (if using colcon)

If you've structured this as a ROS 2 package:

```bash
# From repository root
colcon build --packages-select capstone_demo
source install/setup.bash
```

**Alternative**: Run nodes directly with `python3 node_name.py` (skip this step)

## Step 6: Launch Gazebo Demo

Open a new terminal and launch the complete system:

```bash
# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash  # If using colcon

# Launch Gazebo demo
ros2 launch capstone_demo gazebo_demo.launch.py
```

**Expected Behavior**:
1. Gazebo window opens with kitchen environment
2. Unitree G1 humanoid robot spawns at origin
3. 7 ROS 2 nodes initialize (check with `ros2 node list`)
4. State machine enters `IDLE` state

**Verify nodes are running**:
```bash
# In a new terminal
ros2 node list
```

**Expected Output**:
```
/voice_input_node
/llm_planner_node
/navigation_controller
/object_detection_node
/manipulation_controller
/integration_demo
/robot_state_publisher
```

## Step 7: Issue Voice Command

With all nodes running:

**Option A: Using Microphone** (Default)
```bash
# Speak clearly into your microphone:
"Bring me the red cup from the kitchen table"
```

**Option B: Publish Directly** (Testing/Debugging)
```bash
# In a new terminal
ros2 topic pub --once /voice/transcribed_text std_msgs/msg/String \
  "{data: 'Bring me the red cup from the kitchen table'}"
```

## Step 8: Observe Execution

Watch the terminal logs for state transitions:

```
[INFO] [integration_demo]: State transition: IDLE → PLANNING
[INFO] [llm_planner_node]: Calling LLM for plan generation...
[INFO] [llm_planner_node]: Plan generated with 5 actions
[INFO] [integration_demo]: State transition: PLANNING → VALIDATING
[INFO] [integration_demo]: Plan validation passed
[INFO] [integration_demo]: State transition: VALIDATING → NAVIGATING
[INFO] [integration_demo]: Executing action 1/5: navigate_to
[INFO] [integration_demo]: Navigating to: kitchen_table
...
[INFO] [integration_demo]: State transition: MANIPULATING → COMPLETED
[INFO] [integration_demo]: Task completed successfully!
```

**In Gazebo**:
- Robot navigates to table (~10 seconds)
- Robot detects red cup
- Robot picks up cup (arm movement)
- Robot returns to user location
- Robot places cup

## Expected Timeline

| Time | Event |
|------|-------|
| T+0s | Launch completes, system in IDLE |
| T+10s | User issues voice command |
| T+13s | Whisper transcription complete |
| T+20s | LLM plan generated and validated |
| T+30s | Robot arrives at kitchen table |
| T+35s | Red cup detected and localized |
| T+42s | Cup picked successfully |
| T+55s | Robot returns to user |
| T+60s | Cup delivered, task COMPLETED |

**Acceptable Success Rate**: ≥60% (per AC-001)
**Acceptable Failure Modes**: Navigation blocked (40%), object not found (15%), grasp failure (30%)

## Step 9: Monitor with ROS 2 Tools

**Visualize node graph**:
```bash
rqt_graph
```

**Monitor topics**:
```bash
# Watch transcriptions
ros2 topic echo /voice/transcribed_text

# Watch action plans
ros2 topic echo /planning/action_sequence

# Watch system status
ros2 topic echo /system/status
```

**Check action status**:
```bash
ros2 action list
ros2 action info /navigation/navigate_to_pose
```

## Troubleshooting

### Issue: "No module named 'whisper'"

**Solution**: Reinstall Whisper
```bash
pip install --upgrade openai-whisper
```

### Issue: "OPENAI_API_KEY not set"

**Solution**: Verify environment variable
```bash
echo $OPENAI_API_KEY  # Should print your key
export OPENAI_API_KEY="sk-your-key"
```

### Issue: "Voice not detected"

**Solution**: Check microphone and adjust threshold
```bash
# Test microphone
arecord -l  # List audio devices
arecord -d 3 test.wav && aplay test.wav  # Record and playback

# Adjust silence_threshold in config/voice_config.yaml
silence_threshold: 0.005  # Lower = more sensitive
```

### Issue: "LLM timeout"

**Solution**: Check API key and network
```bash
# Test OpenAI API
curl https://api.openai.com/v1/models \
  -H "Authorization: Bearer $OPENAI_API_KEY"
```

### Issue: "Navigation fails immediately"

**Solution**: Verify map is loaded
```bash
ros2 topic echo /map --once  # Should show map data
ros2 param get /navigation_controller use_sim_time  # Should be true
```

### Issue: "Gazebo crashes or freezes"

**Solution**: Reduce graphics quality or use headless mode
```bash
# Launch without GUI
ros2 launch capstone_demo gazebo_demo.launch.py gui:=false

# Use RViz for visualization instead
rviz2
```

## Running Automated Tests

Validate the system with pytest:

```bash
cd tests/module-05-capstone
pytest -v
```

**Expected**: All tests pass (may take 5-10 minutes)

## Next Steps

✅ **You've successfully run the MVP demo!**

Continue learning:

1. **Read the chapters**: Understand how each component works
   - [Chapter 1: System Architecture](../docs/modules/module-05-capstone/chapter-01-architecture.md)
   - [Chapter 2: Voice & LLM Pipeline](../docs/modules/module-05-capstone/chapter-02-voice-llm.md)
   - [Chapter 5: Simulation Deployment](../docs/modules/module-05-capstone/chapter-05-simulation-deployment.md)

2. **Modify the system**:
   - Add new objects to the capability manifest
   - Extend the state machine with new states
   - Implement retry logic for failures

3. **Deploy to Isaac Sim** (if you have NVIDIA GPU):
   ```bash
   ros2 launch capstone_demo isaac_demo.launch.py
   ```

4. **Deploy to Jetson Orin** (if you have hardware):
   - Follow [Chapter 6: Jetson Deployment](../docs/modules/module-05-capstone/chapter-06-jetson-deployment.md)

5. **Run performance benchmarks**:
   ```bash
   python scripts/testing/benchmark_capstone.py --trials 20
   ```

## Getting Help

- **GitHub Issues**: [Report bugs](https://github.com/asadaligith/AI-Humanoid-Robotics-Book/issues)
- **ROS 2 Discourse**: [Ask ROS questions](https://discourse.ros.org/)
- **Book Appendices**: [Troubleshooting Guide](../docs/modules/module-05-capstone/troubleshooting.md)

---

**Congratulations! You've deployed an autonomous voice-commanded humanoid robot!** 🎉🤖
