#!/usr/bin/env python3

import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import quaternion_from_euler


# Loads the station positions recorded by station_recorder for the current map,
# shows them in Foxglove, sends the robot to a default station on startup, and
# re-sends it to a different station whenever a marker ID is published on
# /mission/goto_station -- reuses the existing /goal_pose -> waypoint_commander
# -> planner_server -> pure_pursuit pipeline for the coarse navigation. Once
# pure_pursuit reports the coarse goal reached, hands off to docking_controller
# (via /docking/target_marker_id) for the precise, marker-based final approach --
# station IDs and marker IDs are the same number by convention, so no lookup
# table is needed.
class MissionCommander(Node):
    def __init__(self):
        super().__init__("mission_commander_node")

        self.declare_parameter("map_name", "bedroom")
        self.declare_parameter("default_station_id", 0)

        self.map_name = self.get_parameter("map_name").value
        self.default_station_id = self.get_parameter("default_station_id").value

        self.stations = self.load_stations()
        # Station id currently being navigated to (coarse leg), or None once docking
        # has been handed off / no mission is active. Used to know which marker ID to
        # dock to once pure_pursuit reports arrival.
        self.pending_station_id = None

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.goal_publisher = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.marker_publisher = self.create_publisher(MarkerArray, "/stations/markers", latched_qos)
        self.dock_target_publisher = self.create_publisher(Int32, "/docking/target_marker_id", 10)
        self.goto_subscriber = self.create_subscription(
            Int32, "/mission/goto_station", self.goto_station_callback, 10)
        self.goal_reached_subscriber = self.create_subscription(
            Bool, "/purepursuit/goal_reached", self.goal_reached_callback, 10)

        self.publish_markers()

        if self.default_station_id in self.stations:
            # Give TF/localization a moment to settle before sending the first goal.
            self.startup_timer = self.create_timer(2.0, self.send_default_goal)
        else:
            self.get_logger().warn(
                f"Default station {self.default_station_id} not found in stations.yaml -- "
                "waiting for a /mission/goto_station command instead.")

    def load_stations(self):
        stations_path = os.path.join(
            get_package_share_directory("bumperbot_mapping"), "maps", self.map_name, "stations.yaml")

        if not os.path.exists(stations_path):
            self.get_logger().warn(
                f"No stations.yaml found at {stations_path} -- run station_recorder and its "
                "save_stations service while mapping first.")
            return {}

        with open(stations_path, "r") as f:
            data = yaml.safe_load(f) or {}

        return {int(k): v for k, v in (data.get("stations") or {}).items()}

    def send_default_goal(self):
        self.startup_timer.cancel()
        self.go_to_station(self.default_station_id)

    def goto_station_callback(self, msg: Int32):
        self.go_to_station(msg.data)

    def go_to_station(self, station_id):
        station = self.stations.get(station_id)
        if station is None:
            self.get_logger().warn(
                f"Unknown station id {station_id} -- known stations: {list(self.stations.keys())}")
            return

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = station["x"]
        goal.pose.position.y = station["y"]

        # quaternion_from_euler returns numpy floats; geometry_msgs fields expect
        # plain Python floats.
        q = quaternion_from_euler(0, 0, station["yaw"])
        goal.pose.orientation.x = float(q[0])
        goal.pose.orientation.y = float(q[1])
        goal.pose.orientation.z = float(q[2])
        goal.pose.orientation.w = float(q[3])

        self.pending_station_id = station_id
        self.goal_publisher.publish(goal)
        self.get_logger().info(f"Sending goal to station {station_id}")

    def goal_reached_callback(self, msg: Bool):
        if not msg.data or self.pending_station_id is None:
            return

        station_id = self.pending_station_id
        self.pending_station_id = None

        dock_msg = Int32()
        dock_msg.data = station_id
        self.dock_target_publisher.publish(dock_msg)
        self.get_logger().info(f"Coarse goal reached -- handing off to docking_controller for marker {station_id}")

    def publish_markers(self):
        marker_array = MarkerArray()
        for station_id, station in self.stations.items():
            sphere = Marker()
            sphere.header.frame_id = "map"
            sphere.ns = "stations"
            sphere.id = station_id * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = station["x"]
            sphere.pose.position.y = station["y"]
            sphere.pose.position.z = 0.1
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.15
            sphere.color.r = 0.1
            sphere.color.g = 0.4
            sphere.color.b = 0.9
            sphere.color.a = 1.0
            marker_array.markers.append(sphere)

            text = Marker()
            text.header.frame_id = "map"
            text.ns = "stations"
            text.id = station_id * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = station["x"]
            text.pose.position.y = station["y"]
            text.pose.position.z = 0.4
            text.scale.z = 0.2
            text.color.r = text.color.g = text.color.b = 1.0
            text.color.a = 1.0
            text.text = f"Station {station_id}"
            marker_array.markers.append(text)

        self.marker_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = MissionCommander()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
