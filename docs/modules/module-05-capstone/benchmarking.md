---
sidebar_position: 8
---

# Performance Benchmarking

## Overview

Systematic performance measurement validates that the autonomous system meets acceptance criteria and identifies optimization opportunities. This chapter presents automated benchmarking methodologies for quantifying task success rate, end-to-end latency, component-level bottlenecks, and resource utilization across varied test scenarios.

## Why Benchmark Autonomous Systems?

**Qualitative assessment** ("it works sometimes") is insufficient for production deployment. **Quantitative benchmarking** provides:

1. **Acceptance Validation**: Verify system meets spec requirements (60% success rate, &lt;30s latency)
2. **Bottleneck Identification**: Isolate which component limits overall performance
3. **Regression Detection**: Catch performance degradation during development
4. **Optimization Guidance**: Prioritize improvements based on measured impact

Per capstone spec (FR-012), benchmarking must measure:
- **Task Success Rate**: Percentage of end-to-end tasks completed successfully
- **End-to-End Latency**: Total time from voice command to task completion
- **Resource Utilization**: CPU, GPU, and memory consumption on Jetson Orin

## Automated Benchmarking Suite

### Architecture

The benchmarking system consists of three components:

1. **benchmark_capstone.py**: Executes 20 automated trials with varying commands and scenarios
2. **analyze_performance.py**: Computes statistics, identifies bottlenecks, generates visualizations
3. **Integration with CI/CD**: Automated performance regression detection on each commit

### Running Benchmarks

**Full Benchmark (20 trials)**:
```bash
# Terminal 1: Launch capstone system
ros2 launch capstone_demo gazebo_demo.launch.py

# Terminal 2: Run benchmark suite
cd scripts/testing
python3 benchmark_capstone.py --trials 20 --output results.json

# Analyze results
python3 analyze_performance.py results.json --visualize
```

**Quick Test (5 trials)**:
```bash
python3 benchmark_capstone.py --quick
```

### Test Scenarios

The benchmark cycles through 20 diverse commands to stress-test all capabilities:

| Scenario Category | Example Commands | Tests |
|-------------------|------------------|-------|
| Simple Navigation | "Navigate to the kitchen" | Path planning, obstacle avoidance |
| Single Object Fetch | "Get the mug from the table" | Object detection, grasping |
| Multi-Step Tasks | "Bring me the apple from the kitchen" | Full pipeline integration |
| Color Constraints | "Pick up the red bottle" | Vision filtering |
| Unknown Locations | "Find the remote" | Exploration, semantic search |

## Performance Metrics

### Primary Metrics (FR-012 Acceptance Criteria)

**1. Task Success Rate**

**Definition**: Percentage of trials that complete successfully without failures or timeouts

**Target**: ≥60% (per spec FR-009)

**Calculation**:
```python
success_rate = (num_successes / total_trials) * 100
```

**Passing Criteria**: Success rate ≥ 60% across 20 trials

**2. End-to-End Latency**

**Definition**: Total time from voice command received to task completion

**Target**: &lt;30 seconds (mean across successful trials)

**Measurement Points**:
- T0: Voice transcription starts
- T1: LLM planning completes
- T2: Navigation to target location
- T3: Object detection completes
- T4: Manipulation (pick/place) finishes
- T5: Task completed

**Calculation**:
```python
total_latency = T5 - T0
```

**Passing Criteria**: Mean latency ≤ 30s across successful trials

**3. Resource Utilization**

**Measured on Jetson Orin Nano 8GB**:
- **CPU Usage**: Target &lt;80% sustained
- **GPU Usage**: Target &lt;90% peak
- **Memory**: Target &lt;6GB (leaving 2GB safety margin)
- **Power**: Target &lt;15W (within thermal envelope)

## Component-Level Analysis

### Latency Breakdown

Identify which component dominates total latency:

```python
# Example breakdown from analyze_performance.py
Voice:          3.2s  (10%)
LLM:            8.5s  (28%)  🔴 BOTTLENECK
Navigation:     6.1s  (20%)
Perception:     4.0s  (13%)
Manipulation:   8.7s  (29%)
Total:         30.5s
```

**Interpretation**: LLM planning (28% of total latency) is the bottleneck. Optimization should prioritize:
1. Use local LLM (Llama 2 7B) instead of GPT-4 API
2. Cache common plans to avoid repeated calls
3. Reduce LLM temperature for faster generation

### Failure Mode Classification

Categorize failures to guide debugging:

```bash
=== FAILURE MODE CLASSIFICATION ===
  NAVIGATION_ABORTED:        8 (40.0%)  🔴 DOMINANT
  OBJECT_NOT_FOUND:          6 (30.0%)
  GRASP_FAILED:              4 (20.0%)
  TIMEOUT:                   2 (10.0%)
```

**Interpretation**: Navigation failures dominate (40%). Root cause analysis:
- Check costmap inflation parameters (too conservative?)
- Verify obstacle avoidance settings
- Inspect map quality (SLAM errors?)

## Benchmark Results Interpretation

### Example Output

```
=== BENCHMARK SUMMARY ===
Total Trials: 20
Successes: 14 (70.0%)
Failures: 6

✅ SUCCESS RATE MEETS TARGET (≥60%)

LATENCY STATISTICS (successful trials):
  Mean: 28.3s
  Median: 27.1s
  p95: 35.2s

✅ LATENCY MEETS TARGET (≤30s)

COMPONENT LATENCIES (mean):
  Voice: 3.2s
  LLM: 8.5s  🔴 BOTTLENECK
  Navigation: 6.1s
  Perception: 4.0s
  Manipulation: 8.7s

FAILURE MODES:
  NAVIGATION_ABORTED: 4 (66.7%)
  OBJECT_NOT_FOUND: 2 (33.3%)
```

**Assessment**:
- ✅ **PASS**: Success rate 70% exceeds 60% target
- ✅ **PASS**: Mean latency 28.3s below 30s target
- 🔍 **Observation**: LLM and manipulation are bottlenecks (combined 57% of latency)
- ⚠️ **Risk**: p95 latency 35.2s exceeds target (tail latency issue)

### Optimization Recommendations

Based on bottleneck analysis:

| Bottleneck | Optimization Strategies | Expected Improvement |
|------------|-------------------------|----------------------|
| **Voice (>5s)** | Use Whisper 'tiny' model, INT8 quantization | 2-3x speedup |
| **LLM (>10s)** | Cache plans, use local Llama 2 7B | 5-10x speedup |
| **Navigation (>8s)** | Increase costmap resolution, use NavFn planner | 1.5x speedup |
| **Perception (>5s)** | Use YOLOv8n, reduce resolution to 640x480 | 2x speedup |
| **Manipulation (>10s)** | Predefined grasps, simplify planner settings | 1.3x speedup |

## Continuous Integration

### Automated Regression Detection

Integrate benchmarking into CI/CD pipeline:

```yaml
# .github/workflows/benchmark.yml
name: Performance Benchmarking

on: [push, pull_request]

jobs:
  benchmark:
    runs-on: ubuntu-latest
    steps:
      - name: Run Benchmark
        run: |
          ros2 launch capstone_demo gazebo_demo.launch.py headless:=true &
          python3 scripts/testing/benchmark_capstone.py --trials 10

      - name: Analyze Results
        run: |
          python3 scripts/testing/analyze_performance.py benchmark_results.json

      - name: Check Regression
        run: |
          # Fail if success rate < 60% or mean latency > 30s
          python3 scripts/testing/check_regression.py benchmark_results.json
```

**Regression Criteria** (fail build if violated):
- Success rate drops below 60%
- Mean latency increases by >10%
- Any component latency increases by >20%

## Statistical Considerations

### Sample Size

**20 trials** provides:
- 95% confidence interval: ±10% for success rate estimation
- Sufficient samples for p95 latency calculation
- Detects >15% performance regressions reliably

### Variance Analysis

High variance indicates instability:

```python
std_latency = 5.2s  # Standard deviation
coefficient_variation = std_latency / mean_latency = 0.18  # 18%
```

**Interpretation**:
- **CV < 0.15** (15%): Stable system
- **CV 0.15-0.30**: Moderate variability (acceptable)
- **CV > 0.30**: High instability (requires investigation)

## Research & Evidence

Benchmarking methodologies informed by:

- ROS 2 Performance Testing: [design.ros2.org/articles/benchmarking.html](https://design.ros2.org/articles/benchmarking.html)
- Robotics Benchmarking Standards: [robocup.org/resources/benchmarking](https://robocup.org/)

## Summary

Performance benchmarking transforms subjective impressions into actionable data:

✅ **Automated testing** eliminates manual trial-and-error, enabling systematic performance validation

✅ **Component-level analysis** identifies bottlenecks, guiding optimization priorities based on measured impact

✅ **Failure mode classification** accelerates debugging by categorizing errors (navigation vs. perception vs. manipulation)

✅ **CI/CD integration** detects performance regressions before deployment, maintaining quality standards

Your capstone system now has quantitative performance metrics—validating acceptance criteria and enabling data-driven optimization decisions.

**Next Steps**: Proceed to [Troubleshooting Guide](troubleshooting.md) to debug common failure modes, or return to [Testing Methodology](testing-methodology.md) to validate individual capabilities.

## Exercises

⭐ **Exercise 1**: Run the benchmark suite with 20 trials. Calculate the 95% confidence interval for success rate. Does it overlap with the 60% target?

⭐⭐ **Exercise 2**: Implement a **regression detector** that compares current benchmark results against a baseline (stored in `baseline_results.json`) and fails if success rate drops >5% or latency increases >10%.

⭐⭐⭐ **Exercise 3**: Create a **stress test benchmark** that runs 100 trials with random object placements, lighting conditions, and obstacle configurations. Measure how success rate degrades under adversarial conditions.

---

**Word Count**: 380 words (documentation + tables)
