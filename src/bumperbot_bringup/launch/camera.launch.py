from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    video_device = LaunchConfiguration("video_device")
    framerate = LaunchConfiguration("framerate")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    marker_size = LaunchConfiguration("marker_size")
    aruco_dictionary_id = LaunchConfiguration("aruco_dictionary_id")

    video_device_arg = DeclareLaunchArgument("video_device", default_value="/dev/video0")
    # Continuous usb_cam + aruco_node CPU usage is a major contributor to the Pi being
    # too overloaded to keep up with its own nav2 lifecycle heartbeats (see costmap.yaml
    # and navigation.launch.py notes) -- dropped from 5.0 to ease that load.
    framerate_arg = DeclareLaunchArgument("framerate", default_value="3.0")
    image_width_arg = DeclareLaunchArgument("image_width", default_value="640")
    image_height_arg = DeclareLaunchArgument("image_height", default_value="480")
    marker_size_arg = DeclareLaunchArgument("marker_size", default_value="0.150")
    aruco_dictionary_id_arg = DeclareLaunchArgument("aruco_dictionary_id", default_value="DICT_4X4_1000")

    usb_cam_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam_node",
        output="screen",
        parameters=[{
            "video_device": video_device,
            "framerate": framerate,
            "image_width": image_width,
            "image_height": image_height,
        }],
    )

    aruco_node = Node(
        package="ros2_aruco",
        executable="aruco_node",
        name="aruco_node",
        output="screen",
        parameters=[{
            "marker_size": marker_size,
            "aruco_dictionary_id": aruco_dictionary_id,
            "image_topic": "/image_raw",
            "camera_info_topic": "/camera_info",
        }],
    )

    return LaunchDescription([
        video_device_arg,
        framerate_arg,
        image_width_arg,
        image_height_arg,
        marker_size_arg,
        aruco_dictionary_id_arg,
        usb_cam_node,
        aruco_node,
    ])
