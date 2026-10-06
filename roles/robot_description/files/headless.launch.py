"""Publish authored X950 geometry and explicit nominal native-camera aliases."""

from pathlib import Path
import xml.etree.ElementTree as ET
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro
import yaml


def add_nominal_d555_aliases(robot):
    """Alias native DDS frame names; retain nominal names as visible parents."""
    suffixes = ["camera_link"]
    suffixes += [f"{stream}_{kind}" for stream in ("depth", "color", "infra1", "infra2")
                 for kind in ("frame", "optical_frame")]
    links = {element.get("name") for element in robot.findall("link")}
    for suffix in suffixes:
        parent = f"d555_nominal_{suffix}"
        child = "camera_link" if suffix == "camera_link" else f"camera_{suffix}"
        if parent not in links or child in links:
            raise ValueError(f"Cannot create nominal native frame alias {parent} -> {child}")
        ET.SubElement(robot, "link", name=child)
        joint = ET.SubElement(robot, "joint", name=f"nominal_native_{child}_joint", type="fixed")
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=child)
        ET.SubElement(joint, "origin", xyz="0 0 0", rpy="0 0 0")
        links.add(child)


def add_nominal_avia_frame(robot, config):
    """Manual optical datum, provisionally located against the existing CAD mount."""
    parent, child = "avia_link", "avia_nominal_lidar_frame"
    links = {element.get("name") for element in robot.findall("link")}
    if parent not in links or child in links:
        raise ValueError(f"Cannot create Avia frame {parent} -> {child}")
    pose = {}
    for key, default in (("xyz", [0.0525, 0.0, 0.0324]), ("rpy", [0.0, 0.0, 0.0])):
        values = config.get(f"avia_lidar_{key}", default)
        if not isinstance(values, list) or len(values) != 3 or not all(math.isfinite(float(v)) for v in values):
            raise ValueError(f"Invalid Avia nominal {key}: {values}")
        pose[key] = " ".join(str(float(v)) for v in values)
    ET.SubElement(robot, "link", name=child)
    joint = ET.SubElement(robot, "joint", name="avia_nominal_lidar_joint", type="fixed")
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=child)
    ET.SubElement(joint, "origin", **pose)


def add_avia_imu(robot, config):
    """The Avia's built-in IMU under its LiDAR frame: Livox's factory offset, axes aligned
    (FAST-LIO's avia.yaml gives the LiDAR in the IMU frame, (0.04165, 0.02326, -0.0284)).
    Named as the avia driver stamps /avia/imu (avia_imu_frame_id). FAST-LIO's IMU-to-LiDAR
    extrinsic and every consumer of its poses read this frame."""
    parent, child = "avia_nominal_lidar_frame", "avia_imu"
    links = {element.get("name") for element in robot.findall("link")}
    if parent not in links or child in links:
        raise ValueError(f"Cannot create Avia frame {parent} -> {child}")
    pose = {}
    for key, default in (("xyz", [-0.04165, -0.02326, 0.0284]), ("rpy", [0.0, 0.0, 0.0])):
        values = config.get(f"avia_imu_{key}", default)
        if not isinstance(values, list) or len(values) != 3 or not all(math.isfinite(float(v)) for v in values):
            raise ValueError(f"Invalid Avia IMU {key}: {values}")
        pose[key] = " ".join(str(float(v)) for v in values)
    ET.SubElement(robot, "link", name=child)
    joint = ET.SubElement(robot, "joint", name="avia_imu_joint", type="fixed")
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=child)
    ET.SubElement(joint, "origin", **pose)


def _launch_setup(context):
    config_path = Path(LaunchConfiguration("config_file").perform(context))
    config = yaml.safe_load(config_path.read_text())
    if not isinstance(config, dict):
        raise ValueError(f"Invalid description configuration: {config_path}")
    mappings = config.get("xacro_mappings", {})
    if not isinstance(mappings, dict):
        raise ValueError("xacro_mappings must be a mapping")
    package_share = Path(get_package_share_directory("x950_description"))
    xacro_path = package_share / "urdf" / "x950.urdf.xacro"
    declared = {element.get("name") for element in ET.parse(xacro_path).getroot().findall(
        "{http://www.ros.org/wiki/xacro}arg")}
    unknown = set(mappings) - declared
    if unknown:
        raise ValueError(f"Unknown X950 pose overrides: {sorted(unknown)}")
    robot = ET.fromstring(xacro.process_file(
        str(xacro_path), mappings={name: str(value) for name, value in mappings.items()}
    ).toxml())
    if config.get("native_d555_aliases", True):
        add_nominal_d555_aliases(robot)
    add_nominal_avia_frame(robot, config)
    add_avia_imu(robot, config)
    return [Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        on_exit=[EmitEvent(event=Shutdown(reason="robot_state_publisher exited"))],
        parameters=[{
            "robot_description": ParameterValue(ET.tostring(robot, encoding="unicode"), value_type=str),
            "use_sim_time": False,
        }],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value="/etc/uav/ros/description.yaml"),
        OpaqueFunction(function=_launch_setup),
    ])
