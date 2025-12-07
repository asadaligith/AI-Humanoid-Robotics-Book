#!/usr/bin/env python3
"""
Performance Analysis Tool for Capstone Benchmarking

Analyzes benchmark results to identify bottlenecks, classify failure modes,
and generate visualizations of performance metrics.

Performs:
- Latency breakdown analysis (which component is the bottleneck?)
- Failure mode classification (navigation vs. perception vs. manipulation)
- Statistical analysis (mean, median, p95, standard deviation)
- Performance comparison across trials
- Visualization generation (latency distributions, component breakdowns)

Usage:
    python3 analyze_performance.py benchmark_results.json
    python3 analyze_performance.py results.json --visualize --output report.html

Author: GIAIC Hackathon Q4 Team
License: MIT
"""

import json
import argparse
import statistics
from typing import List, Dict, Any, Tuple
import sys
from dataclasses import dataclass


@dataclass
class PerformanceMetrics:
    """Aggregated performance metrics"""
    total_trials: int
    success_count: int
    failure_count: int
    success_rate: float

    mean_latency: float
    median_latency: float
    p95_latency: float
    std_latency: float

    mean_voice: float
    mean_llm: float
    mean_navigation: float
    mean_perception: float
    mean_manipulation: float

    bottleneck_component: str
    dominant_failure_mode: str


def load_results(filename: str) -> Dict[str, Any]:
    """Load benchmark results from JSON file."""
    try:
        with open(filename, 'r') as f:
            data = json.load(f)
        print(f"✓ Loaded {len(data.get('results', []))} trial results from {filename}")
        return data
    except FileNotFoundError:
        print(f"❌ Error: File not found: {filename}")
        sys.exit(1)
    except json.JSONDecodeError as e:
        print(f"❌ Error: Invalid JSON in {filename}: {e}")
        sys.exit(1)


def analyze_latency_breakdown(results: List[Dict]) -> Dict[str, float]:
    """
    Analyze which component contributes most to total latency.

    Returns:
        Dict mapping component names to mean latencies
    """
    successes = [r for r in results if r['success']]

    if not successes:
        return {}

    breakdown = {
        'voice': statistics.mean([r['voice_latency'] for r in successes]),
        'llm': statistics.mean([r['llm_latency'] for r in successes]),
        'navigation': statistics.mean([r['navigation_latency'] for r in successes]),
        'perception': statistics.mean([r['perception_latency'] for r in successes]),
        'manipulation': statistics.mean([r['manipulation_latency'] for r in successes])
    }

    # Identify bottleneck
    bottleneck = max(breakdown.items(), key=lambda x: x[1])

    print("\n=== LATENCY BREAKDOWN (Mean) ===")
    for component, latency in sorted(breakdown.items(), key=lambda x: -x[1]):
        percentage = (latency / sum(breakdown.values())) * 100 if sum(breakdown.values()) > 0 else 0
        indicator = "🔴" if component == bottleneck[0] else "  "
        print(f"{indicator} {component.capitalize():15s}: {latency:6.2f}s ({percentage:5.1f}%)")

    print(f"\n🔍 BOTTLENECK: {bottleneck[0].upper()} ({bottleneck[1]:.2f}s)")

    return breakdown


def classify_failure_modes(results: List[Dict]) -> Dict[str, int]:
    """
    Classify and count failure modes.

    Returns:
        Dict mapping failure modes to counts
    """
    failures = [r for r in results if not r['success']]

    failure_modes = {}
    for f in failures:
        mode = f.get('failure_mode', 'UNKNOWN')
        if not mode:
            mode = 'UNKNOWN'
        failure_modes[mode] = failure_modes.get(mode, 0) + 1

    if failure_modes:
        print("\n=== FAILURE MODE CLASSIFICATION ===")
        total_failures = sum(failure_modes.values())

        for mode, count in sorted(failure_modes.items(), key=lambda x: -x[1]):
            percentage = (count / total_failures) * 100
            print(f"  {mode:30s}: {count:3d} ({percentage:5.1f}%)")

        dominant_mode = max(failure_modes.items(), key=lambda x: x[1])[0]
        print(f"\n🔍 DOMINANT FAILURE MODE: {dominant_mode}")
    else:
        print("\n=== FAILURE MODE CLASSIFICATION ===")
        print("  No failures recorded! 🎉")

    return failure_modes


def compute_statistics(results: List[Dict]) -> PerformanceMetrics:
    """Compute comprehensive performance statistics."""
    successes = [r for r in results if r['success']]
    failures = [r for r in results if not r['success']]

    if not successes:
        print("❌ Warning: No successful trials to analyze")
        return None

    total_latencies = [r['total_latency'] for r in successes]

    mean_latency = statistics.mean(total_latencies)
    median_latency = statistics.median(total_latencies)
    p95_latency = sorted(total_latencies)[int(len(total_latencies) * 0.95)] if len(total_latencies) > 1 else mean_latency
    std_latency = statistics.stdev(total_latencies) if len(total_latencies) > 1 else 0.0

    # Component latencies
    mean_voice = statistics.mean([r['voice_latency'] for r in successes])
    mean_llm = statistics.mean([r['llm_latency'] for r in successes])
    mean_navigation = statistics.mean([r['navigation_latency'] for r in successes])
    mean_perception = statistics.mean([r['perception_latency'] for r in successes])
    mean_manipulation = statistics.mean([r['manipulation_latency'] for r in successes])

    # Identify bottleneck
    component_latencies = {
        'Voice': mean_voice,
        'LLM': mean_llm,
        'Navigation': mean_navigation,
        'Perception': mean_perception,
        'Manipulation': mean_manipulation
    }
    bottleneck = max(component_latencies.items(), key=lambda x: x[1])[0]

    # Dominant failure mode
    failure_modes = {}
    for f in failures:
        mode = f.get('failure_mode', 'UNKNOWN')
        if not mode:
            mode = 'UNKNOWN'
        failure_modes[mode] = failure_modes.get(mode, 0) + 1

    dominant_failure = max(failure_modes.items(), key=lambda x: x[1])[0] if failure_modes else "N/A"

    metrics = PerformanceMetrics(
        total_trials=len(results),
        success_count=len(successes),
        failure_count=len(failures),
        success_rate=len(successes) / len(results) * 100,
        mean_latency=mean_latency,
        median_latency=median_latency,
        p95_latency=p95_latency,
        std_latency=std_latency,
        mean_voice=mean_voice,
        mean_llm=mean_llm,
        mean_navigation=mean_navigation,
        mean_perception=mean_perception,
        mean_manipulation=mean_manipulation,
        bottleneck_component=bottleneck,
        dominant_failure_mode=dominant_failure
    )

    return metrics


def print_performance_report(metrics: PerformanceMetrics):
    """Print comprehensive performance report."""
    print("\n" + "=" * 70)
    print("PERFORMANCE ANALYSIS REPORT")
    print("=" * 70)

    # Overall metrics
    print("\n--- OVERALL PERFORMANCE ---")
    print(f"Total Trials:       {metrics.total_trials}")
    print(f"Successes:          {metrics.success_count} ({metrics.success_rate:.1f}%)")
    print(f"Failures:           {metrics.failure_count}")

    # Evaluate against targets
    target_success = 60.0
    target_latency = 30.0

    success_status = "✅ PASS" if metrics.success_rate >= target_success else "❌ FAIL"
    latency_status = "✅ PASS" if metrics.mean_latency <= target_latency else "❌ FAIL"

    print(f"\nSuccess Rate Target: ≥{target_success}% — {success_status}")
    print(f"Latency Target:      ≤{target_latency}s — {latency_status}")

    # Latency statistics
    print("\n--- LATENCY STATISTICS (successful trials) ---")
    print(f"Mean:          {metrics.mean_latency:.2f}s")
    print(f"Median:        {metrics.median_latency:.2f}s")
    print(f"p95:           {metrics.p95_latency:.2f}s")
    print(f"Std Dev:       {metrics.std_latency:.2f}s")

    # Component breakdown
    print("\n--- COMPONENT LATENCIES (mean) ---")
    components = [
        ('Voice', metrics.mean_voice),
        ('LLM', metrics.mean_llm),
        ('Navigation', metrics.mean_navigation),
        ('Perception', metrics.mean_perception),
        ('Manipulation', metrics.mean_manipulation)
    ]

    total_component_time = sum(c[1] for c in components)

    for name, latency in sorted(components, key=lambda x: -x[1]):
        percentage = (latency / total_component_time) * 100 if total_component_time > 0 else 0
        indicator = "🔴" if name == metrics.bottleneck_component else "  "
        print(f"{indicator} {name:15s}: {latency:6.2f}s ({percentage:5.1f}%)")

    # Bottleneck analysis
    print("\n--- BOTTLENECK ANALYSIS ---")
    print(f"Primary Bottleneck:     {metrics.bottleneck_component}")
    print(f"Dominant Failure Mode:  {metrics.dominant_failure_mode}")

    # Recommendations
    print("\n--- OPTIMIZATION RECOMMENDATIONS ---")

    if metrics.bottleneck_component == 'Voice':
        print("  • Reduce Whisper model size (use 'tiny' or 'base' instead of 'small')")
        print("  • Enable INT8 quantization for faster inference")
        print("  • Reduce audio chunk duration (e.g., 2s instead of 3s)")

    elif metrics.bottleneck_component == 'LLM':
        print("  • Cache common plans to avoid repeated API calls")
        print("  • Use local LLM (Llama 2 7B) instead of GPT-4 API")
        print("  • Reduce LLM temperature for faster generation")

    elif metrics.bottleneck_component == 'Navigation':
        print("  • Increase costmap resolution for faster planning")
        print("  • Use simplified planner (NavFn instead of A*)")
        print("  • Pre-compute waypoints for common locations")

    elif metrics.bottleneck_component == 'Perception':
        print("  • Use faster YOLO model (YOLOv8n instead of YOLOv8m)")
        print("  • Reduce camera resolution (640x480 instead of 1280x720)")
        print("  • Lower detection confidence threshold (0.5 instead of 0.6)")

    elif metrics.bottleneck_component == 'Manipulation':
        print("  • Use predefined grasps instead of heuristic planning")
        print("  • Simplify motion planner settings (reduce IK iterations)")
        print("  • Pre-compute joint trajectories for common objects")

    print("\n" + "=" * 70)


def generate_visualization(results: List[Dict], output_file: str):
    """
    Generate HTML visualization of performance metrics.

    Note: Requires matplotlib for full functionality.
    For educational demo, outputs simple text-based charts.
    """
    print(f"\n📊 Generating visualization: {output_file}")

    try:
        import matplotlib.pyplot as plt
        import matplotlib

        matplotlib.use('Agg')  # Non-interactive backend

        successes = [r for r in results if r['success']]

        if not successes:
            print("❌ No successful trials to visualize")
            return

        # Create figure with subplots
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(14, 10))
        fig.suptitle('Capstone Performance Analysis', fontsize=16, fontweight='bold')

        # 1. Latency histogram
        latencies = [r['total_latency'] for r in successes]
        ax1.hist(latencies, bins=15, color='skyblue', edgecolor='black', alpha=0.7)
        ax1.axvline(statistics.mean(latencies), color='red', linestyle='--', label=f'Mean: {statistics.mean(latencies):.1f}s')
        ax1.set_xlabel('Total Latency (seconds)')
        ax1.set_ylabel('Frequency')
        ax1.set_title('Latency Distribution')
        ax1.legend()
        ax1.grid(axis='y', alpha=0.3)

        # 2. Component breakdown (pie chart)
        components = {
            'Voice': statistics.mean([r['voice_latency'] for r in successes]),
            'LLM': statistics.mean([r['llm_latency'] for r in successes]),
            'Navigation': statistics.mean([r['navigation_latency'] for r in successes]),
            'Perception': statistics.mean([r['perception_latency'] for r in successes]),
            'Manipulation': statistics.mean([r['manipulation_latency'] for r in successes])
        }
        ax2.pie(components.values(), labels=components.keys(), autopct='%1.1f%%', startangle=90)
        ax2.set_title('Component Latency Breakdown')

        # 3. Latency over trials (trend)
        trial_nums = [r['trial_id'] for r in successes]
        trial_latencies = [r['total_latency'] for r in successes]
        ax3.plot(trial_nums, trial_latencies, marker='o', linestyle='-', color='green', alpha=0.6)
        ax3.axhline(statistics.mean(trial_latencies), color='red', linestyle='--', label=f'Mean: {statistics.mean(trial_latencies):.1f}s')
        ax3.set_xlabel('Trial Number')
        ax3.set_ylabel('Latency (seconds)')
        ax3.set_title('Latency Trend Over Trials')
        ax3.legend()
        ax3.grid(alpha=0.3)

        # 4. Success vs. Failure
        success_count = len([r for r in results if r['success']])
        failure_count = len([r for r in results if not r['success']])
        ax4.bar(['Success', 'Failure'], [success_count, failure_count], color=['green', 'red'], alpha=0.7)
        ax4.set_ylabel('Count')
        ax4.set_title('Success vs. Failure')
        ax4.grid(axis='y', alpha=0.3)

        # Save figure
        plt.tight_layout()
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"✓ Visualization saved to {output_file}")

    except ImportError:
        print("⚠️  matplotlib not installed - skipping visualization")
        print("   Install with: pip install matplotlib")


def main():
    parser = argparse.ArgumentParser(
        description='Analyze capstone benchmark results',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )

    parser.add_argument(
        'input_file',
        type=str,
        help='Benchmark results JSON file (from benchmark_capstone.py)'
    )

    parser.add_argument(
        '--visualize',
        action='store_true',
        help='Generate visualization plots'
    )

    parser.add_argument(
        '--output',
        type=str,
        default='performance_analysis.png',
        help='Output visualization file (default: performance_analysis.png)'
    )

    args = parser.parse_args()

    # Load results
    data = load_results(args.input_file)
    results = data.get('results', [])

    if not results:
        print("❌ No results found in input file")
        sys.exit(1)

    # Analyze latency breakdown
    breakdown = analyze_latency_breakdown(results)

    # Classify failure modes
    failure_modes = classify_failure_modes(results)

    # Compute comprehensive statistics
    metrics = compute_statistics(results)

    if metrics:
        print_performance_report(metrics)

    # Generate visualization
    if args.visualize:
        generate_visualization(results, args.output)


if __name__ == '__main__':
    main()
