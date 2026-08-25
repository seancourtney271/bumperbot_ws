#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace bumperbot_motion
{
    class DockingController : public rclcpp::Node
    {
        public:
            DockingController();
        private:
            // Latest ArUco detections, each with a marker ID and a pose relative to the camera.
            rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr marker_subscriber_;
            // Lets the mission sequencer pick which station (marker ID) to dock to.
            rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr target_marker_subscriber_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_publisher_;
            // Latches true once the robot reaches the standoff pose facing the target marker.
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr docked_publisher_;

            std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

            rclcpp::TimerBase::SharedPtr control_loop_;

            // How far in front of the marker (along its face normal) the robot should stop.
            double standoff_distance_;
            double k_prop_linear_;
            double k_prop_angular_;
            double maximum_linear_velocity_;
            double maximum_angular_velocity_;
            // Above this heading error (radians) to the dock pose, rotate in place instead
            // of driving forward -- same reasoning as pure_pursuit's large-heading-error handling.
            double heading_error_threshold_;
            double position_tolerance_;
            double goal_yaw_tolerance_;
            // How long (seconds) a marker can go unseen before its dock pose is considered stale.
            double marker_timeout_;
            // Rotation speed while searching for a marker that hasn't been spotted yet --
            // deliberately much slower than maximum_angular_velocity_. The camera framerate
            // is low (3 Hz), so sweeping at full speed can rotate right past a marker between
            // frames without ever getting a detection while pointed at it.
            double search_angular_velocity_;
            // Give up searching (and just hold still) after sweeping for this long without
            // ever finding the marker, rather than spinning forever.
            double search_timeout_;
            // Give up on the approach/alignment phase after this long (seconds) since the
            // marker was first found, accepting wherever the robot currently is as "docked"
            // rather than fighting forever to hit position_tolerance_/goal_yaw_tolerance_
            // exactly. Without this, an approach that never quite settles (e.g. oscillating
            // around goal_yaw_tolerance_) never sets docked_ true, which leaves the same
            // marker "live" -- so driving past it again later re-triggers docking instead of
            // it staying finished.
            double docking_timeout_;

            // Which marker ID to dock to; negative means no active docking target.
            int64_t target_marker_id_;
            // Rotation direction (+1/-1) locked in while rotating in place, 0 when not
            // rotating -- prevents oscillation near the +-pi heading-error boundary.
            int locked_rotation_sign_;
            bool has_target_pose_;
            bool docked_;
            // Standoff pose to drive to, expressed in the odom frame.
            geometry_msgs::msg::PoseStamped target_pose_;
            rclcpp::Time last_marker_seen_time_;
            // When the current search for target_marker_id_ started -- reset whenever the
            // target changes. Used to enforce search_timeout_.
            rclcpp::Time search_start_time_;
            // When the marker was first found (has_target_pose_ false -> true transition).
            // Used to enforce docking_timeout_.
            rclcpp::Time approach_start_time_;

            void markerCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

            void targetMarkerCallback(const std_msgs::msg::Int32::SharedPtr msg);

            void controlLoop();
    };
}
