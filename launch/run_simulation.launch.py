from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    ur_robot_driver_pkg = FindPackageShare("ur_robot_driver")
    ur_moveit_pkg = FindPackageShare("ur_moveit_config")
    assessment_pkg = FindPackageShare("devrob_assessment")

    tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0.8", "0", "0", "0", "world", "base_link"]
    )

    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ur_robot_driver_pkg, "/launch/ur_control.launch.py"]
        ),
        launch_arguments={
            "ur_type": "ur10",
            "robot_ip": "0.0.0.0",
            "use_fake_hardware": "true",
            "launch_rviz": "false",
            "initial_joint_controller": "scaled_joint_trajectory_controller"
        }.items()
    )

    ur_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ur_moveit_pkg, "/launch/ur_moveit.launch.py"]
        ),
        launch_arguments={
            "ur_type": "ur10",
            "launch_rviz": "true",
            "use_fake_hardware": "true" 
        }.items()
    )

    config_file = os.path.join(
        assessment_pkg.find("devrob_assessment"),
        "config",
        "waypoints.yaml"
    )
    
    waypoint_node = Node(
        package="devrob_assessment",
        executable="waypoint_mover",
        output="screen",
        parameters=[config_file]
    )

    delayed_waypoint_node = TimerAction(
        period=10.0,
        actions=[waypoint_node]
    )

    return LaunchDescription([
        tf_node,
        ur_control_launch,
        ur_moveit_launch,
        delayed_waypoint_node
    ])