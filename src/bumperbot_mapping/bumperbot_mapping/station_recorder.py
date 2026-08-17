#!/usr/bin/env python3

import os
import math
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
from tf_transformations import quaternion_matrix


# Watches for ArUco markers while SLAM is running and remembers where each one
# sits in the map frame, so a saved map can be reloaded later already knowing
# where every station is -- no need to drive past each marker again to
# rediscover them under localization.
class StationRecorder(Node):
    def __init__(self):
        super().__init__("station_recorder_node")

        self.declare_parameter("map_name", "demo")
        self.declare_parameter("known_marker_ids", [0, 1])
        self.declare_parameter("standoff_distance", 0.5)
        # Empty by default -- if set, this exact path is used for save_stations instead
        # of deriving one from map_name.
        self.declare_parameter("stations_path", "")
        self.map_name = self.get_parameter("map_name").value
        self.stations_path_override = self.get_parameter("stations_path").value
        # Must match docking_controller's default -- the whole point of recording this
        # standoff point (instead of the marker's own pose) is so mission_commander sends
        # the robot to the same reachable spot docking_controller would otherwise compute
        # live, rather than to the marker itself (which sits on/at the wall and is
        # unreachable -- inside the costmap's occupied/inflated region).
        self.standoff_distance = self.get_parameter("standoff_distance").value
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
                # Use the marker message's own timestamp, not "latest" -- the pose was
                # computed from an image captured slightly earlier (capture + detection
                # latency), and while the robot is turning, "latest" TF and "TF at capture
                # time" disagree enough to visibly shift the recorded station position.
                transform = self.tf_buffer.lookup_transform(
                    "map", msg.header.frame_id, msg.header.stamp)
            except (LookupException, ConnectivityException, ExtrapolationException) as ex:
                self.get_logger().warn(f"Could not transform marker pose into map frame: {ex}")
                continue

            map_pose = do_transform_pose_stamped(marker_pose, transform)

            # The marker's local Z axis points out of its face -- toward whoever is
            # looking at it. quaternion_matrix's third column is exactly that axis
            # expressed in the map frame (equivalent to rotating the unit Z vector by
            # the marker's orientation), matching docking_controller's C++ tf2::quatRotate.
            rot = quaternion_matrix([
                map_pose.pose.orientation.x,
                map_pose.pose.orientation.y,
                map_pose.pose.orientation.z,
                map_pose.pose.orientation.w,
            ])
            normal_x, normal_y = rot[0, 2], rot[1, 2]

            standoff_x = map_pose.pose.position.x + self.standoff_distance * normal_x
            standoff_y = map_pose.pose.position.y + self.standoff_distance * normal_y
            # Face back toward the marker from the standoff point.
            yaw = math.atan2(-normal_y, -normal_x)

            # quaternion_matrix returns numpy floats, which PyYAML's safe_dump can't
            # represent -- cast everything to plain Python types before storing.
            self.stations[marker_id] = {
                "x": float(standoff_x),
                "y": float(standoff_y),
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

        if self.stations_path_override:
            stations_path = self.stations_path_override
            os.makedirs(os.path.dirname(stations_path), exist_ok=True)
        else:
            maps_dir = os.path.join(self._source_maps_dir(), self.map_name)
            os.makedirs(maps_dir, exist_ok=True)
            stations_path = os.path.join(maps_dir, "stations.yaml")

        with open(stations_path, "w") as f:
            yaml.safe_dump({"stations": self.stations}, f)

        response.success = True
        response.message = (
            f"Saved {len(self.stations)} station(s) to {stations_path}. "
            "Rebuild bumperbot_mapping before localizing so this reaches the install space too."
        )
        self.get_logger().info(response.message)
        return response

    def _source_maps_dir(self):
        # get_package_share_directory always points at the install space (by design --
        # install space is meant to work standalone, without src/ present). Maps are kept
        # in git under src/ though, so walk back from ".../install/bumperbot_mapping/..."
        # to the workspace root and target src/bumperbot_mapping/maps instead. This only
        # works for a standard (non-merged) colcon workspace layout, which is what this
        # robot uses.
        share_dir = get_package_share_directory("bumperbot_mapping")
        workspace_root, sep, _ = share_dir.partition(os.sep + "install" + os.sep)
        if not sep:
            return os.path.join(share_dir, "maps")
        return os.path.join(workspace_root, "src", "bumperbot_mapping", "maps")


def main(args=None):
    rclpy.init(args=args)
    node = StationRecorder()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
