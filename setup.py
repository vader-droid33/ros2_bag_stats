from setuptools import setup, find_packages

setup(
    name="ros2_bag_stats",
    version="0.1.0",
    description="CLI tool for analysing ROS2 bag files",
    packages=find_packages(),
    python_requires=">=3.8",
    entry_points={
        "console_scripts": [
            "ros2_bag_stats=ros2_bag_stats.stats:main",
        ],
    },
)
