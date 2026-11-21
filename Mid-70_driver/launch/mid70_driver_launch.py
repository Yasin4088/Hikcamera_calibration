import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node


def load_config(config_path):
    with open(config_path, "r", encoding="utf-8") as config_file:
        return yaml.safe_load(config_file) or {}


def load_node_params(config, node_name):
    return config.get(node_name, {}).get("ros__parameters", {})


def to_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.lower() in ("true", "1", "yes", "on")
    return bool(value)


def generate_launch_description():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(base_dir, "..", "config", "mid70_default.yaml")
    config = load_config(config_path)
    launch_params = load_node_params(config, "/mid70_driver_launch")
    enable_lidar = to_bool(launch_params.get("enable_lidar", True))

    livox_node = Node(
        package="livox_lidar",
        executable="livox_lidar_node",
        name="livox_lidar",
        parameters=[config_path],
        output="both",
    )

    entities = []
    if enable_lidar:
        entities.append(livox_node)

    return LaunchDescription(entities)
