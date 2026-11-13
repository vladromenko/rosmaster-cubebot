import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params = os.environ["CUBEBOT_NAV2_PARAMS"]
    nodes = ["controller_server", "planner_server", "behavior_server", "bt_navigator"]
    out = "screen"
    return LaunchDescription([
        Node(package="nav2_controller", executable="controller_server", output=out, parameters=[params],
             remappings=[("cmd_vel", "nav_cmd_vel")]),
        Node(package="nav2_planner", executable="planner_server", name="planner_server", output=out, parameters=[params]),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server", output=out,
             parameters=[params], remappings=[("cmd_vel", "nav_cmd_vel")]),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator", output=out, parameters=[params]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             output=out, parameters=[{"autostart": True}, {"node_names": nodes}, {"bond_timeout": 6.0}]),
    ])
