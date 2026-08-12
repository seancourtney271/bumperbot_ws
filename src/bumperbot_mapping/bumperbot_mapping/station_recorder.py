#!/usr/bin/env python3

import os
import yaml

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from ros2_aruco_interfaces.msg import ArucoMarkers
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_pose_stamped
from tf_transformations import euler_from_quaternion


# Watches for ArUco markers while SLAM is running and remembers where each one
# sits in the map frame, so a saved map can be reloaded later already knowing
# where every station is -- no need to drive past each marker again to
# rediscover them under localization.
class StationRecorder(Node):
    def __init__(self):
        super().__init__("station_recorder_node")

        self.declare_parameter("map_name", "bedroom")
        self.declare_parameter("known_marker_ids", [0, 1])
        self.map_name = self.get_parameter("map_name").value
        # ArUco detection on a 640x480 webcam occasionally misreads background
        # clutter as a plausible-looking marker and reports a bogus ID -- only
        # record IDs that are actually real, known station markers.
        self.known_marker_ids = set(self.get_parameter("known_marker_ids").value)
        self._warned_ids = set()

        # marker_id -> {"x", "y", "yaw"} in the map frame. Later sightings of
        # the same marker simply overwrite the earlier estimate.
        self.stations = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_subscriber = self.create_subscription(
            ArucoMarkers, "/aruco_markers", self.marker_callback, 10)

        self.marker_publisher = self.create_publisher(MarkerArray, "/stations/found", 10)

        self.save_service = self.create_service(Trigger, "save_stations", self.save_stations_callback)

        self.get_logger().info(
            f"Station recorder ready for map '{self.map_name}'. Drive past each station's "
            "marker, then call the 'save_stations' service to write stations.yaml.")

    def marker_callback(self, msg: ArucoMarkers):
        for marker_id, pose in zip(msg.marker_ids, msg.poses):
            marker_id = int(marker_id)
            if marker_id not in self.known_marker_ids:
                if marker_id not in self._warned_ids:
                    self._warned_ids.add(marker_id)
                    self.get_logger().warn(
                        f"Ignoring detection of marker ID {marker_id} -- not in known_marker_ids "
                        f"{sorted(self.known_marker_ids)} (likely a false-positive misread).")
                continue

            marker_pose = PoseStamped()
            marker_pose.header = msg.header
            marker_pose.pose = pose

            try:
                transform = self.tf_buffer.lookup_transform(
                    "map", msg.header.frame_id, rclpy.time.Time())
            except (LookupException, ConnectivityException, ExtrapolationException) as ex:
                self.get_logger().warn(f"Could not transform marker pose into map frame: {ex}")
                continue

            map_pose = do_transform_pose_stamped(marker_pose, transform)
            _, _, yaw = euler_from_quaternion([
                map_pose.pose.orientation.x,
                map_pose.pose.orientation.y,
                map_pose.pose.orientation.z,
                map_pose.pose.orientation.w,
            ])

            # euler_from_quaternion returns numpy floats, which PyYAML's safe_dump
            # can't represent -- cast everything to plain Python types before storing.
            self.stations[marker_id] = {
                "x": float(map_pose.pose.position.x),
                "y": float(map_pose.pose.position.y),
                "yaw": float(yaw),
            }

        self.publish_markers()

    def publish_markers(self):
        marker_array = MarkerArray()
        for marker_id, station in self.stations.items():
            sphere = Marker()
            sphere.header.frame_id = "map"
            sphere.ns = "stations"
            sphere.id = marker_id * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = station["x"]
            sphere.pose.position.y = station["y"]
            sphere.pose.position.z = 0.1
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.15
            sphere.color.r = 0.1
            sphere.color.g = 0.9
            sphere.color.b = 0.1
            sphere.color.a = 1.0
            marker_array.markers.append(sphere)

            text = Marker()
            text.header.frame_id = "map"
            text.ns = "stations"
            text.id = marker_id * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = station["x"]
            text.pose.position.y = station["y"]
            text.pose.position.z = 0.4
            text.scale.z = 0.2
            text.color.r = text.color.g = text.color.b = 1.0
            text.color.a = 1.0
            text.text = f"Station {marker_id}"
            marker_array.markers.append(text)

        self.marker_publisher.publish(marker_array)

    def save_stations_callback(self, request, response):
        if not self.stations:
            response.success = False
            response.message = "No stations recorded yet -- drive past each marker before saving."
            return response

        maps_dir = os.path.join(
            get_package_share_directory("bumperbot_mapping"), "maps", self.map_name)
        os.makedirs(maps_dir, exist_ok=True)
        stations_path = os.path.join(maps_dir, "stations.yaml")

        with open(stations_path, "w") as f:
            yaml.safe_dump({"stations": self.stations}, f)

        response.success = True
        response.message = f"Saved {len(self.stations)} station(s) to {stations_path}"
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = StationRecorder()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
