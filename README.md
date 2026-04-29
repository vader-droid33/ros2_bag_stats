# ros2_bag_stats

A fast, zero-dependency CLI tool for analysing ROS2 `.db3` bag files. Get detailed statistics on topic frequencies, message counts, data sizes, and timing gaps — without needing a ROS2 installation.


## Why?

When working with ROS2 bags — especially large sensor datasets — it's useful to quickly answer questions like:

- Is my IMU actually publishing at 200 Hz or did it drop messages?
- How many messages does each topic have and how much disk space does it use?
- Are there timing gaps that could affect my SLAM algorithm?
- Did the MoCap system publish consistently throughout the recording?

Standard ROS2 tools like `ros2 bag info` give basic information but miss per-topic gap detection, frequency validation, and export options. `ros2_bag_stats` fills that gap with a single command, and works without sourcing a ROS2 workspace.

## Features

- **Zero dependencies** — uses only Python stdlib and SQLite3
- **No ROS2 required** — reads `.db3` files directly
- **Gap detection** — finds missing messages and timing anomalies
- **Rate checking** — validates sensor frequencies against expected rates
- **Multiple output formats** — terminal, Markdown, CSV, JSON
- **Export** — save results to file for reporting

## Installation

From source:

    git clone https://github.com/vader-droid33/ros2_bag_stats.git
    cd ros2_bag_stats
    pip install -e .

## Usage

    # Basic analysis
    ros2_bag_stats path/to/bag/

    # Check sensor rates against expected values
    ros2_bag_stats path/to/bag/ --check-rates

    # Output as Markdown
    ros2_bag_stats path/to/bag/ --format markdown

    # Export to JSON
    ros2_bag_stats path/to/bag/ --export stats.json

    # Filter specific topics
    ros2_bag_stats path/to/bag/ --topics /livox/lidar /odom

    # No colour output
    ros2_bag_stats path/to/bag/ --no-color > report.txt

## Example Output

    ----------------------------------------------------------------------
      ROS2 Bag Statistics
    ----------------------------------------------------------------------
      Bag:       bags/raw/layout_00_slow_run_01/run_20260325_141516_0.db3
      Recorded:  2026-03-25 14:15:22
      Duration:  95.3s
      Size:      514.3 MB
      Messages:  68,229
      Topics:    7
    ----------------------------------------------------------------------
      Topics
    ----------------------------------------------------------------------
      Topic                                         Hz    Count     MB  Gaps
      ------------------------------------------------------------------
      /livox/imu                                 199.8   19,038    6.2  311
      /livox/lidar                                10.0      952  495.2    0
      /odom                                       50.0    4,765    3.5    1
      /sensors/core                               50.0    4,765    0.9    1
      /tf                                         50.0    4,766    0.5    1
      /tf_static                                   0.0        2    0.0    0
      /vrpn_mocap/Car2/pose                      356.1   33,941    2.9 8458
    ----------------------------------------------------------------------
      Rate Check
    ----------------------------------------------------------------------
      OK  /livox/lidar                              10.0 Hz  (expected 8-12 Hz)
      OK  /livox/imu                               199.8 Hz  (expected 180-220 Hz)
      OK  /odom                                     50.0 Hz  (expected 40-60 Hz)
      OK  /vrpn_mocap/Car2/pose                    356.1 Hz  (expected 100-400 Hz)
      OK  /sensors/core                             50.0 Hz  (expected 40-60 Hz)
    ----------------------------------------------------------------------

## Python API

    from ros2_bag_stats import find_db3, read_bag, compute_stats, check_rates

    db3 = find_db3("path/to/bag/")
    topics, messages = read_bag(db3)
    stats = compute_stats(topics, messages, db3)

    print(f"Duration: {stats['duration_s']:.1f}s")
    for topic_name, t in stats['topics'].items():
        print(f"{topic_name}: {t['freq_hz']} Hz, {t['count']} messages")

## Compatibility

- Python 3.8+
- ROS2 Foxy, Galactic, Humble, Iron, Jazzy (SQLite3 storage)
- Ubuntu 20.04 / 22.04 / 24.04
- Works without ROS2 installed

## License

MIT
