#!/usr/bin/env python3
"""
ros2_bag_stats - A CLI tool for analysing ROS2 bag files.

Provides detailed statistics on any ROS2 .db3 bag including:
- Duration, size, start/end times
- Per-topic message counts, frequencies, and gap detection
- Sensor health checks (expected vs actual rates)
- Export to JSON, CSV, or Markdown

Usage:
  ros2_bag_stats path/to/bag/
  ros2_bag_stats path/to/bag/ --format markdown
  ros2_bag_stats path/to/bag/ --export stats.json
  ros2_bag_stats path/to/bag/ --check-rates
"""

import os
import sys
import json
import math
import sqlite3
import argparse
import struct
from datetime import datetime
from collections import defaultdict
from pathlib import Path


# -- Colours ------------------------------------------------------------------
class C:
    GREEN  = "\033[92m"
    YELLOW = "\033[93m"
    RED    = "\033[91m"
    BLUE   = "\033[94m"
    BOLD   = "\033[1m"
    RESET  = "\033[0m"
    CYAN   = "\033[96m"
    GREY   = "\033[90m"

def ok(s):    return f"{C.GREEN}OK {C.RESET} {s}"
def warn(s):  return f"{C.YELLOW}OK {C.RESET} {s}"
def err(s):   return f"{C.RED}OK {C.RESET} {s}"
def info(s):  return f"{C.BLUE}OK {C.RESET} {s}"
def bold(s):  return f"{C.BOLD}{s}{C.RESET}"


# -- SQLite bag reader ---------------------------------------------------------
def find_db3(bag_path):
    """Find the .db3 file in a bag directory."""
    bag_path = Path(bag_path)
    if bag_path.suffix == ".db3":
        return bag_path
    db3s = list(bag_path.glob("*.db3"))
    if not db3s:
        raise FileNotFoundError(f"No .db3 file found in {bag_path}")
    return db3s[0]


def read_bag(db3_path):
    """
    Read a ROS2 SQLite3 bag and return raw data.
    Returns:
        topics: dict {topic_id: {name, type, ...}}
        messages: list of (topic_id, timestamp_ns, data_size)
    """
    conn = sqlite3.connect(str(db3_path))
    cur = conn.cursor()

    # Get topics
    cur.execute("SELECT id, name, type, serialization_format FROM topics")
    topics = {}
    for row in cur.fetchall():
        topics[row[0]] = {
            "name": row[1],
            "type": row[2],
            "serialization_format": row[3],
        }

    # Get messages (timestamp + size only, no deserialization needed)
    cur.execute("SELECT topic_id, timestamp, length(data) FROM messages ORDER BY timestamp")
    messages = cur.fetchall()

    conn.close()
    return topics, messages


# -- Statistics computation ----------------------------------------------------
def compute_stats(topics, messages, db3_path):
    """Compute full statistics from raw bag data."""

    if not messages:
        return None

    # Basic bag info
    start_ns = messages[0][1]
    end_ns   = messages[-1][1]
    duration_s = (end_ns - start_ns) / 1e9

    bag_size_bytes = os.path.getsize(db3_path)

    # Per-topic stats
    topic_msgs = defaultdict(list)  # topic_id -> list of timestamps
    topic_sizes = defaultdict(int)  # topic_id -> total bytes

    for topic_id, timestamp_ns, data_size in messages:
        topic_msgs[topic_id].append(timestamp_ns)
        topic_sizes[topic_id] += data_size

    topic_stats = {}
    for topic_id, timestamps in topic_msgs.items():
        topic_info = topics.get(topic_id, {})
        name = topic_info.get("name", f"unknown_{topic_id}")
        msg_type = topic_info.get("type", "unknown")

        count = len(timestamps)
        avg_freq = count / duration_s if duration_s > 0 else 0

        # Gap analysis
        gaps = []
        if len(timestamps) > 1:
            diffs = [(timestamps[i+1] - timestamps[i]) / 1e9
                     for i in range(len(timestamps)-1)]
            expected_dt = 1.0 / avg_freq if avg_freq > 0 else 1.0
            gaps = [d for d in diffs if d > expected_dt * 3]  # gaps > 3x expected

        # Frequency stability (std dev of inter-message intervals)
        freq_std = 0.0
        if len(timestamps) > 2:
            diffs_s = [(timestamps[i+1] - timestamps[i]) / 1e9
                       for i in range(len(timestamps)-1)]
            mean_dt = sum(diffs_s) / len(diffs_s)
            freq_std = math.sqrt(sum((d - mean_dt)**2 for d in diffs_s) / len(diffs_s))
            freq_std = freq_std / mean_dt if mean_dt > 0 else 0  # coefficient of variation

        topic_stats[name] = {
            "type":       msg_type,
            "count":      count,
            "freq_hz":    round(avg_freq, 2),
            "freq_std_cv": round(freq_std, 3),  # coefficient of variation
            "size_bytes": topic_sizes[topic_id],
            "size_mb":    round(topic_sizes[topic_id] / 1e6, 2),
            "gap_count":  len(gaps),
            "max_gap_s":  round(max(gaps), 3) if gaps else 0.0,
            "first_msg_s": round((timestamps[0] - start_ns) / 1e9, 3),
            "last_msg_s":  round((timestamps[-1] - start_ns) / 1e9, 3),
        }

    return {
        "bag_path":    str(db3_path),
        "start_time":  start_ns,
        "end_time":    end_ns,
        "duration_s":  round(duration_s, 3),
        "start_datetime": datetime.fromtimestamp(start_ns / 1e9).strftime("%Y-%m-%d %H:%M:%S"),
        "total_messages": len(messages),
        "bag_size_bytes": bag_size_bytes,
        "bag_size_mb":    round(bag_size_bytes / 1e6, 1),
        "topics":         topic_stats,
        "topic_count":    len(topic_stats),
    }


# -- Rate checker --------------------------------------------------------------
# Common ROS2 sensor expected rates
EXPECTED_RATES = {
    "/livox/lidar":           {"min": 8,   "max": 12,  "label": "Livox MID-360 LiDAR"},
    "/livox/imu":             {"min": 180, "max": 220, "label": "Livox IMU"},
    "/odom":                  {"min": 40,  "max": 60,  "label": "Wheel Odometry"},
    "/sensors/core":          {"min": 40,  "max": 60,  "label": "VESC Motor Controller"},
    "/vrpn_mocap/Car2/pose":  {"min": 100, "max": 400, "label": "OptiTrack MoCap"},
    "/camera/image_raw":      {"min": 25,  "max": 35,  "label": "Camera"},
    "/scan":                  {"min": 8,   "max": 12,  "label": "2D LiDAR Scan"},
    "/imu/data":              {"min": 90,  "max": 110, "label": "IMU"},
    "/tf":                    {"min": 1,   "max": 1000,"label": "TF"},
    "/tf_static":             {"min": 1,   "max": 100, "label": "Static TF"},
}


def check_rates(stats):
    """Check topic frequencies against expected rates."""
    results = []
    for topic_name, topic_stat in stats["topics"].items():
        actual = topic_stat["freq_hz"]

        if topic_name in EXPECTED_RATES:
            expected = EXPECTED_RATES[topic_name]
            if actual < expected["min"]:
                status = "LOW"
            elif actual > expected["max"]:
                status = "HIGH"
            else:
                status = "OK"
            results.append({
                "topic": topic_name,
                "label": expected["label"],
                "actual_hz": actual,
                "expected": f"{expected['min']}-{expected['max']} Hz",
                "status": status,
            })
        else:
            results.append({
                "topic": topic_name,
                "label": "Unknown",
                "actual_hz": actual,
                "expected": "N/A",
                "status": "UNKNOWN",
            })
    return results


# -- Formatters ----------------------------------------------------------------
def format_terminal(stats, show_rates=False, no_color=False):
    """Format stats for terminal output with colours."""

    lines = []
    sep = "-" * 70

    def c(text, colour):
        if no_color:
            return text
        return colour + text + C.RESET

    lines.append(c(sep, C.GREY))
    lines.append(c("  ROS2 Bag Statistics", C.BOLD))
    lines.append(c(sep, C.GREY))
    lines.append(f"  Bag:       {stats['bag_path']}")
    lines.append(f"  Recorded:  {stats['start_datetime']}")
    dur = f"{stats['duration_s']:.1f}s"
    lines.append(f"  Duration:  {c(dur, C.CYAN)}")
    lines.append(f"  Size:      {stats['bag_size_mb']} MB")
    lines.append(f"  Messages:  {stats['total_messages']:,}")
    lines.append(f"  Topics:    {stats['topic_count']}")
    lines.append(c(sep, C.GREY))
    lines.append(c("  Topics", C.BOLD))
    lines.append(c(sep, C.GREY))

    # Column header
    lines.append(f"  {'Topic':<40} {'Hz':>7} {'Count':>8} {'MB':>6} {'Gaps':>5}")
    lines.append(c("  " + "-"*66, C.GREY))

    for topic_name, t in sorted(stats["topics"].items()):
        gap_str = str(t["gap_count"]) if t["gap_count"] == 0 else c(str(t["gap_count"]), C.YELLOW)
        freq_str = f"{t['freq_hz']:>7.1f}"
        lines.append(
            f"  {topic_name:<40} {freq_str} {t['count']:>8,} {t['size_mb']:>6.1f} {gap_str:>5}"
        )

    if show_rates:
        lines.append(c(sep, C.GREY))
        lines.append(c("  Rate Check", C.BOLD))
        lines.append(c(sep, C.GREY))
        rate_results = check_rates(stats)
        for r in rate_results:
            if r["status"] == "OK":
                symbol = c("OK ", C.GREEN)
            elif r["status"] in ("LOW", "HIGH"):
                symbol = c("OK ", C.YELLOW)
            else:
                symbol = c("OK ", C.GREY)
            lines.append(
                f"  {symbol} {r['topic']:<38} "
                f"{r['actual_hz']:>7.1f} Hz  (expected {r['expected']})"
            )

    lines.append(c(sep, C.GREY))
    return "\n".join(lines)


def format_markdown(stats):
    """Format stats as Markdown table."""
    lines = []
    lines.append(f"# ROS2 Bag Statistics")
    lines.append(f"")
    lines.append(f"| Property | Value |")
    lines.append(f"|---|---|")
    lines.append(f"| Bag | `{stats['bag_path']}` |")
    lines.append(f"| Recorded | {stats['start_datetime']} |")
    lines.append(f"| Duration | {stats['duration_s']:.1f}s |")
    lines.append(f"| Size | {stats['bag_size_mb']} MB |")
    lines.append(f"| Total Messages | {stats['total_messages']:,} |")
    lines.append(f"| Topics | {stats['topic_count']} |")
    lines.append(f"")
    lines.append(f"## Topics")
    lines.append(f"")
    lines.append(f"| Topic | Type | Hz | Count | Size (MB) | Gaps |")
    lines.append(f"|---|---|---|---|---|---|")
    for name, t in sorted(stats["topics"].items()):
        lines.append(
            f"| `{name}` | `{t['type']}` | {t['freq_hz']} "
            f"| {t['count']:,} | {t['size_mb']} | {t['gap_count']} |"
        )
    return "\n".join(lines)


def format_csv(stats):
    """Format stats as CSV."""
    lines = ["topic,type,freq_hz,count,size_mb,gap_count,max_gap_s"]
    for name, t in sorted(stats["topics"].items()):
        lines.append(
            f"{name},{t['type']},{t['freq_hz']},{t['count']},"
            f"{t['size_mb']},{t['gap_count']},{t['max_gap_s']}"
        )
    return "\n".join(lines)


# -- Main ----------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="ros2_bag_stats � Analyse ROS2 bag files",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  ros2_bag_stats bags/raw/layout_00_slow_run_01/
  ros2_bag_stats my_bag/ --format markdown
  ros2_bag_stats my_bag/ --check-rates
  ros2_bag_stats my_bag/ --export stats.json
  ros2_bag_stats my_bag/ --no-color > report.txt
        """
    )
    parser.add_argument("bag",
                        help="Path to ROS2 bag directory or .db3 file")
    parser.add_argument("--format", "-f",
                        choices=["terminal", "markdown", "csv", "json"],
                        default="terminal",
                        help="Output format (default: terminal)")
    parser.add_argument("--export", "-e",
                        help="Export stats to file (format inferred from extension)")
    parser.add_argument("--check-rates", "-r",
                        action="store_true",
                        help="Check topic frequencies against expected rates")
    parser.add_argument("--no-color",
                        action="store_true",
                        help="Disable colour output")
    parser.add_argument("--topics", "-t",
                        nargs="+",
                        help="Filter to specific topics only")

    args = parser.parse_args()

    # Find and read bag
    try:
        db3_path = find_db3(args.bag)
    except FileNotFoundError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

    topics, messages = read_bag(db3_path)

    if not messages:
        print("Error: Bag contains no messages.", file=sys.stderr)
        sys.exit(1)

    stats = compute_stats(topics, messages, db3_path)

    # Filter topics if requested
    if args.topics:
        stats["topics"] = {
            k: v for k, v in stats["topics"].items()
            if any(f in k for f in args.topics)
        }

    # Output
    if args.format == "terminal":
        print(format_terminal(stats, args.check_rates, args.no_color))
    elif args.format == "markdown":
        print(format_markdown(stats))
    elif args.format == "csv":
        print(format_csv(stats))
    elif args.format == "json":
        print(json.dumps(stats, indent=2))

    # Export
    if args.export:
        export_path = Path(args.export)
        ext = export_path.suffix.lower()
        if ext == ".json":
            content = json.dumps(stats, indent=2)
        elif ext == ".md":
            content = format_markdown(stats)
        elif ext == ".csv":
            content = format_csv(stats)
        else:
            content = json.dumps(stats, indent=2)

        export_path.write_text(content)
        print(f"\nExported to: {export_path}")


if __name__ == "__main__":
    main()