import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    planner_config = LaunchConfiguration("planner_config")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false"
    )

    planner_config_arg = DeclareLaunchArgument(
        "planner_config",
        default_value=os.path.join(
            get_package_share_directory("bumperbot_navigation"),
            "config",
            "planner_server.yaml"
        ),
        description="Full path to the planner_server yaml file to load"
    )

    costmap_config = LaunchConfiguration("costmap_config")
    costmap_config_arg = DeclareLaunchArgument(
        "costmap_config",
        default_value=os.path.join(
            get_package_share_directory("bumperbot_navigation"),
            "config",
            "costmap.yaml"
        ),
        description="Full path to the local costmap yaml file to load"
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            planner_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    local_costmap_node = Node(
        package="bumperbot_navigation",
        executable="local_costmap_node",
        name="local_costmap",
        output="screen",
        parameters=[
            costmap_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"node_names": ["planner_server", "local_costmap"]},
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            # This Pi has repeatedly shown slow lifecycle service responses under
            # startup CPU load (see slam_toolbox/map_server history) -- give the
            # bond more room than the 4s default before treating it as dead.
            {"bond_timeout": 10.0},
        ],
    )

    pure_pursuit = Node(
        package="bumperbot_motion",
        executable="pure_pursuit",
        name="pure_pursuit_planner_node",
        output="screen",
        parameters=[
            {"path_subscriber": "/plan"},
            {"costmap_topic": "/local_costmap/costmap"},
            {"use_sim_time": use_sim_time},
        ],
    )

    waypoint_commander = Node(
        package="bumperbot_navigation",
        executable="waypoint_commander",
        name="waypoint_commander",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        planner_config_arg,
        costmap_config_arg,
        planner_server,
        local_costmap_node,
        lifecycle_manager,
        pure_pursuit,
        waypoint_commander,
    ])
