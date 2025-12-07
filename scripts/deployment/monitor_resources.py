#!/usr/bin/env python3
"""
Resource Monitoring for Jetson Orin Deployment

Monitors CPU, GPU, memory, and temperature metrics during capstone execution.
Outputs CSV format for analysis and visualization.

Usage:
    python3 monitor_resources.py                    # Print to stdout
    python3 monitor_resources.py > metrics.csv      # Save to file
    python3 monitor_resources.py --interval 0.5     # Custom interval

Author: GIAIC Hackathon Q4 Team
License: MIT
"""

import time
import subprocess
import re
import argparse
from datetime import datetime


def get_gpu_stats():
    """
    Query Jetson GPU utilization using tegrastats.

    Returns:
        dict: GPU frequency and utilization percentage
    """
    try:
        # Run tegrastats for 1 second and capture output
        result = subprocess.run(
            ['tegrastats', '--interval', '1000'],
            capture_output=True,
            text=True,
            timeout=2
        )
        output = result.stdout

        # Parse GPU frequency utilization (example: GR3D_FREQ 99%)
        gpu_match = re.search(r'GR3D_FREQ (\d+)%', output)
        gpu_util = int(gpu_match.group(1)) if gpu_match else 0

        return {
            'gpu_util_pct': gpu_util
        }

    except subprocess.TimeoutExpired:
        return {'gpu_util_pct': 0}
    except Exception as e:
        print(f"# GPU stats error: {e}", file=sys.stderr)
        return {'gpu_util_pct': 0}


def get_memory_stats():
    """
    Get system memory usage from /proc/meminfo.

    Returns:
        dict: Total, used, and available memory in MB and percentage
    """
    try:
        with open('/proc/meminfo', 'r') as f:
            lines = f.readlines()

        mem_total = int([l for l in lines if 'MemTotal' in l][0].split()[1]) // 1024  # KB to MB
        mem_avail = int([l for l in lines if 'MemAvailable' in l][0].split()[1]) // 1024
        mem_used = mem_total - mem_avail
        mem_used_pct = (mem_used / mem_total) * 100

        return {
            'mem_total_mb': mem_total,
            'mem_used_mb': mem_used,
            'mem_used_pct': mem_used_pct
        }

    except Exception as e:
        print(f"# Memory stats error: {e}", file=sys.stderr)
        return {'mem_total_mb': 0, 'mem_used_mb': 0, 'mem_used_pct': 0}


def get_cpu_stats():
    """
    Get CPU utilization and temperature.

    Returns:
        dict: CPU usage percentage and temperature in Celsius
    """
    try:
        # CPU temperature from thermal zone
        with open('/sys/devices/virtual/thermal/thermal_zone0/temp', 'r') as f:
            temp_millicelsius = int(f.read().strip())
            temp_celsius = temp_millicelsius / 1000.0

        # CPU usage from /proc/stat (simplified)
        with open('/proc/stat', 'r') as f:
            cpu_line = f.readline()
            cpu_vals = list(map(int, cpu_line.split()[1:]))
            cpu_idle = cpu_vals[3]
            cpu_total = sum(cpu_vals)

        # Store for next iteration (basic implementation)
        if not hasattr(get_cpu_stats, 'prev_idle'):
            get_cpu_stats.prev_idle = cpu_idle
            get_cpu_stats.prev_total = cpu_total
            cpu_util = 0.0
        else:
            idle_delta = cpu_idle - get_cpu_stats.prev_idle
            total_delta = cpu_total - get_cpu_stats.prev_total
            cpu_util = 100.0 * (1.0 - idle_delta / total_delta) if total_delta > 0 else 0.0

            get_cpu_stats.prev_idle = cpu_idle
            get_cpu_stats.prev_total = cpu_total

        return {
            'cpu_util_pct': cpu_util,
            'cpu_temp_c': temp_celsius
        }

    except Exception as e:
        print(f"# CPU stats error: {e}", file=sys.stderr)
        return {'cpu_util_pct': 0, 'cpu_temp_c': 0}


def get_power_stats():
    """
    Get power consumption (if available on Jetson).

    Returns:
        dict: Power consumption in Watts
    """
    try:
        # Jetson power monitoring via INA3221 (if available)
        # This is a simplified version - actual path may vary
        power_paths = [
            '/sys/bus/i2c/drivers/ina3221x/1-0040/iio:device0/in_power0_input',
            '/sys/bus/i2c/drivers/ina3221/0-0040/iio_device/in_power0_input'
        ]

        for path in power_paths:
            try:
                with open(path, 'r') as f:
                    power_mw = int(f.read().strip())
                    power_w = power_mw / 1000.0
                    return {'power_w': power_w}
            except FileNotFoundError:
                continue

        return {'power_w': 0}  # Power monitoring not available

    except Exception as e:
        return {'power_w': 0}


def monitor_loop(interval=1.0):
    """
    Continuous monitoring loop that prints metrics as CSV.

    Args:
        interval (float): Sampling interval in seconds
    """
    # Print CSV header
    print("timestamp,datetime,cpu_pct,cpu_temp_c,gpu_pct,mem_used_mb,mem_pct,power_w")

    try:
        while True:
            timestamp = time.time()
            dt = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            cpu_stats = get_cpu_stats()
            gpu_stats = get_gpu_stats()
            mem_stats = get_memory_stats()
            power_stats = get_power_stats()

            # Print CSV row
            print(f"{timestamp:.2f},"
                  f"{dt},"
                  f"{cpu_stats['cpu_util_pct']:.1f},"
                  f"{cpu_stats['cpu_temp_c']:.1f},"
                  f"{gpu_stats['gpu_util_pct']},"
                  f"{mem_stats['mem_used_mb']},"
                  f"{mem_stats['mem_used_pct']:.1f},"
                  f"{power_stats['power_w']:.2f}")

            time.sleep(interval)

    except KeyboardInterrupt:
        print("\n# Monitoring stopped by user", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(
        description='Monitor Jetson Orin resource utilization',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  python3 monitor_resources.py                    # Print to stdout
  python3 monitor_resources.py > metrics.csv      # Save to file
  python3 monitor_resources.py --interval 0.5     # Sample every 500ms
        '''
    )

    parser.add_argument(
        '--interval',
        type=float,
        default=1.0,
        help='Sampling interval in seconds (default: 1.0)'
    )

    args = parser.parse_args()

    monitor_loop(interval=args.interval)


if __name__ == '__main__':
    import sys
    main()
