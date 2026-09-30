"""Display the X950 mechanical model and publish its fixed transforms."""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def _launch_setup(context, package_share, xacro_path, argument_names):
    mappings = {
        name: LaunchConfiguration(name).perform(context)
        for name in argument_names
    }
    description = xacro.process_file(
        str(xacro_path), mappings=mappings
    ).toxml()

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": ParameterValue(description, value_type=str),
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", str(package_share / "rviz" / "x950.rviz")],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory("x950_description"))
    xacro_path = package_share / "urdf" / "x950.urdf.xacro"
    # Read defaults from the exported model so launch never duplicates CAD poses.
    xacro_arguments = ET.parse(xacro_path).getroot().findall(
        "{http://www.ros.org/wiki/xacro}arg"
    )
    declarations = []
    argument_names = []
    for argument in xacro_arguments:
        name = argument.attrib["name"]
        units = "metres" if name.endswith("_xyz") else "radians, roll pitch yaw"
        declarations.append(DeclareLaunchArgument(
            name,
            default_value=argument.attrib["default"],
            description=f"Joint pose relative to its parent ({units}); three values.",
        ))
        argument_names.append(name)

    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Open RViz with the X950 model and frame axes.",
        ),
        *declarations,
        OpaqueFunction(
            function=_launch_setup,
            kwargs={
                "package_share": package_share,
                "xacro_path": xacro_path,
                "argument_names": argument_names,
            },
        ),
    ])
