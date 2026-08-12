import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")
    map_name = LaunchConfiguration("map_name")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="bedroom",
        description="Name of the saved map (folder under bumperbot_mapping/maps) to localize against when use_slam is false"
    )

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )

    laser_driver = Node(
            package="rplidar_ros",
            executable="rplidar_composition",
            name="rplidar_node",
            parameters=[os.path.join(
                get_package_share_directory("bumperbot_bringup"),
                "config",
                "rplidar_a1.yaml"
            )],
            output="screen"
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False",
            "use_sim_time": "False"
        }.items(),
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )

    imu_driver_node = Node(
        package="bumperbot_firmware",
        executable="mpu6050_driver.py"
    )

    global_localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        launch_arguments={
            "map_name": map_name,
            "use_sim_time": "False"
        }.items(),
        condition=UnlessCondition(use_slam)
    )
    local_localization = IncludeLaunchDescription(
        os.path.join(get_package_share_directory("bumperbot_localization"), "launch", "local_localization.launch.py"),
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
        launch_arguments={
            "map_name": map_name,
            "use_sim_time": "False"
        }.items(),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        launch_arguments={
            "map_name": map_name,
            "use_sim_time": "False"
        }.items(),
        condition=IfCondition(use_slam)
    )

    # --- FOXGLOVE BRIDGE SECTION ---
    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[{
            "port": 8765,                  # Default WebSocket port for Foxglove Studio
            "address": "0.0.0.0",         # Bind to all interfaces so external devices can connect
            "send_buffer_limit": 10000000 # Buffer limit to prevent drops on high-bandwidth data like lasers/images
        }],
        output="screen"
    )
    
    return LaunchDescription([
        use_slam_arg,
        map_name_arg,
        hardware_interface,
        laser_driver,
        controller,
        joystick,
        imu_driver_node,
        local_localization,
        global_localization,
        navigation,
        slam,
        foxglove_bridge # Added here
    ])