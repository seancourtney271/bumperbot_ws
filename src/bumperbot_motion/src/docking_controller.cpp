#include <algorithm>
#include <cmath>
#include "bumperbot_motion/docking_controller.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace bumperbot_motion
{
    // Drives the robot to a standoff pose in front of a chosen ArUco marker (a station's
    // fiducial), facing it square-on, and reports when docked. Which marker to dock to is
    // selected externally (via the target_marker_id parameter or the /docking/target_marker_id
    // topic), so a mission sequencer can reuse this node for every station.
    DockingController::DockingController() : Node("docking_controller_node"),
        standoff_distance_(0.3), k_prop_linear_(1.0), k_prop_angular_(2.0),
        maximum_linear_velocity_(0.15), maximum_angular_velocity_(0.5),
        heading_error_threshold_(0.5), position_tolerance_(0.05), goal_yaw_tolerance_(0.1),
        marker_timeout_(1.0), search_angular_velocity_(0.15), search_timeout_(60.0),
        target_marker_id_(-1), locked_rotation_sign_(0),
        has_target_pose_(false), docked_(false)
    {
        declare_parameter<double>("standoff_distance", standoff_distance_);
        declare_parameter<double>("kp_linear", k_prop_linear_);
        declare_parameter<double>("kp_angular", k_prop_angular_);
        declare_parameter<double>("maximum_linear_velocity", maximum_linear_velocity_);
        declare_parameter<double>("maximum_angular_velocity", maximum_angular_velocity_);
        declare_parameter<double>("heading_error_threshold", heading_error_threshold_);
        declare_parameter<double>("position_tolerance", position_tolerance_);
        declare_parameter<double>("goal_yaw_tolerance", goal_yaw_tolerance_);
        declare_parameter<double>("marker_timeout", marker_timeout_);
        declare_parameter<double>("search_angular_velocity", search_angular_velocity_);
        declare_parameter<double>("search_timeout", search_timeout_);
        declare_parameter<int64_t>("target_marker_id", target_marker_id_);

        standoff_distance_ = get_parameter("standoff_distance").as_double();
        k_prop_linear_ = get_parameter("kp_linear").as_double();
        k_prop_angular_ = get_parameter("kp_angular").as_double();
        maximum_linear_velocity_ = get_parameter("maximum_linear_velocity").as_double();
        maximum_angular_velocity_ = get_parameter("maximum_angular_velocity").as_double();
        heading_error_threshold_ = get_parameter("heading_error_threshold").as_double();
        position_tolerance_ = get_parameter("position_tolerance").as_double();
        goal_yaw_tolerance_ = get_parameter("goal_yaw_tolerance").as_double();
        marker_timeout_ = get_parameter("marker_timeout").as_double();
        search_angular_velocity_ = get_parameter("search_angular_velocity").as_double();
        search_timeout_ = get_parameter("search_timeout").as_double();
        target_marker_id_ = get_parameter("target_marker_id").as_int();

        marker_subscriber_ = create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "/aruco_markers", 10, std::bind(&DockingController::markerCallback, this, std::placeholders::_1));

        target_marker_subscriber_ = create_subscription<std_msgs::msg::Int32>(
            "/docking/target_marker_id", 10, std::bind(&DockingController::targetMarkerCallback, this, std::placeholders::_1));

        command_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        dock_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/docking/target_pose", 10);
        docked_publisher_ = create_publisher<std_msgs::msg::Bool>("/docking/docked", rclcpp::QoS(1).transient_local());

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        control_loop_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&DockingController::controlLoop, this));

        last_marker_seen_time_ = get_clock()->now();
        search_start_time_ = get_clock()->now();

        RCLCPP_INFO(get_logger(), "Docking controller ready (target marker: %ld)", target_marker_id_);
    }

    // Stores the standoff pose for the currently selected target marker whenever it's seen.
    void DockingController::markerCallback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
    {
        if(target_marker_id_ < 0)
        {
            return;
        }

        for(std::size_t i = 0; i < msg->marker_ids.size(); ++i)
        {
            if(msg->marker_ids[i] != target_marker_id_)
            {
                continue;
            }

            geometry_msgs::msg::PoseStamped marker_pose;
            marker_pose.header = msg->header;
            marker_pose.pose = msg->poses[i];

            geometry_msgs::msg::PoseStamped marker_pose_odom;
            try
            {
                auto transform = tf_buffer_->lookupTransform("odom", marker_pose.header.frame_id, tf2::TimePointZero);
                tf2::doTransform(marker_pose, marker_pose_odom, transform);
            }
            catch(const tf2::TransformException & ex)
            {
                RCLCPP_WARN(get_logger(), "Could not transform marker pose into odom frame: %s", ex.what());
                return;
            }

            // The marker's local Z axis points out of its face -- toward whoever is looking
            // at it. Standing off along that axis (instead of straight-line from the robot)
            // keeps the approach square to the marker regardless of the angle it was spotted from.
            tf2::Quaternion marker_q;
            tf2::fromMsg(marker_pose_odom.pose.orientation, marker_q);
            tf2::Vector3 marker_normal = tf2::quatRotate(marker_q, tf2::Vector3(0, 0, 1));

            target_pose_.header.frame_id = "odom";
            target_pose_.pose.position.x = marker_pose_odom.pose.position.x + standoff_distance_ * marker_normal.x();
            target_pose_.pose.position.y = marker_pose_odom.pose.position.y + standoff_distance_ * marker_normal.y();
            target_pose_.pose.position.z = 0.0;

            // Face back toward the marker from the standoff point.
            double yaw = std::atan2(-marker_normal.y(), -marker_normal.x());
            tf2::Quaternion target_q;
            target_q.setRPY(0, 0, yaw);
            target_pose_.pose.orientation = tf2::toMsg(target_q);

            if(!has_target_pose_)
            {
                // Just found it after searching -- discard the search rotation's lock so
                // the approach/alignment phases below pick their own direction fresh.
                locked_rotation_sign_ = 0;
            }
            has_target_pose_ = true;
            last_marker_seen_time_ = get_clock()->now();
            dock_pose_publisher_->publish(target_pose_);
            return;
        }
    }

    // Lets an external mission sequencer switch which station's marker to dock to.
    void DockingController::targetMarkerCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        if(msg->data != target_marker_id_)
        {
            RCLCPP_INFO(get_logger(), "New docking target: marker ID %d", msg->data);
            target_marker_id_ = msg->data;
            has_target_pose_ = false;
            docked_ = false;
            locked_rotation_sign_ = 0;
            search_start_time_ = get_clock()->now();
        }
    }

    void DockingController::controlLoop()
    {
        if(target_marker_id_ < 0 || docked_)
        {
            return;
        }

        if(!has_target_pose_)
        {
            // Never seen the target marker yet -- sweep slowly in place to give the
            // (low-framerate) camera a chance to actually catch it in a frame. Rotating
            // at full speed can carry the marker across the camera's narrow field of view
            // between frames without ever producing a detection while pointed at it.
            if((get_clock()->now() - search_start_time_).seconds() > search_timeout_)
            {
                RCLCPP_WARN(get_logger(), "Gave up searching for marker %ld after %.0fs -- holding still.",
                    target_marker_id_, search_timeout_);
                command_publisher_->publish(geometry_msgs::msg::Twist());
                return;
            }

            if(locked_rotation_sign_ == 0)
            {
                locked_rotation_sign_ = 1;
            }

            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.angular.z = locked_rotation_sign_ * search_angular_velocity_;
            command_publisher_->publish(cmd_vel);
            return;
        }

        geometry_msgs::msg::TransformStamped robot_transform;
        try
        {
            robot_transform = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
        }
        catch(const tf2::TransformException & ex)
        {
            RCLCPP_WARN(get_logger(), "Could not transform: %s", ex.what());
            return;
        }

        geometry_msgs::msg::PoseStamped robot_pose;
        robot_pose.header.frame_id = robot_transform.header.frame_id;
        robot_pose.pose.position.x = robot_transform.transform.translation.x;
        robot_pose.pose.position.y = robot_transform.transform.translation.y;
        robot_pose.pose.orientation = robot_transform.transform.rotation;

        double dx = target_pose_.pose.position.x - robot_pose.pose.position.x;
        double dy = target_pose_.pose.position.y - robot_pose.pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        if(distance <= position_tolerance_)
        {
            // Position reached -- now square up to the marker's facing before declaring docked.
            double goal_yaw = tf2::getYaw(target_pose_.pose.orientation);
            double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
            double yaw_error = std::atan2(std::sin(goal_yaw - robot_yaw), std::cos(goal_yaw - robot_yaw));

            if(std::abs(yaw_error) <= goal_yaw_tolerance_)
            {
                RCLCPP_INFO(get_logger(), "Docked at marker %ld", target_marker_id_);
                docked_ = true;
                locked_rotation_sign_ = 0;
                command_publisher_->publish(geometry_msgs::msg::Twist());

                std_msgs::msg::Bool docked_msg;
                docked_msg.data = true;
                docked_publisher_->publish(docked_msg);
                return;
            }

            if(locked_rotation_sign_ == 0)
            {
                locked_rotation_sign_ = (yaw_error > 0) ? 1 : -1;
            }

            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.angular.z = locked_rotation_sign_ * maximum_angular_velocity_;
            command_publisher_->publish(cmd_vel);
            return;
        }

        tf2::Transform robot_tf, target_tf, target_robot_tf;
        tf2::fromMsg(robot_pose.pose, robot_tf);
        tf2::fromMsg(target_pose_.pose, target_tf);
        target_robot_tf = robot_tf.inverse() * target_tf;

        double heading_error = std::atan2(target_robot_tf.getOrigin().getY(), target_robot_tf.getOrigin().getX());

        geometry_msgs::msg::Twist cmd_vel;
        if(std::abs(heading_error) > heading_error_threshold_)
        {
            // Target is well off to the side or behind the robot -- rotate in place toward
            // it first, locking the direction so noise near the +-pi boundary can't flip
            // it back and forth (same fix pure_pursuit needed for the same reason). No
            // freshness check here, same reasoning as the final alignment phase above:
            // this rotation itself is very likely to point the narrow-FOV camera away from
            // the marker mid-turn, and it doesn't need a fresh sighting to keep turning
            // toward a target it already locked in -- requiring one here previously left
            // the robot frozen mid-turn instead of ever finishing it and driving forward.
            if(locked_rotation_sign_ == 0)
            {
                locked_rotation_sign_ = (heading_error > 0) ? 1 : -1;
            }
            cmd_vel.angular.z = locked_rotation_sign_ * maximum_angular_velocity_;
            command_publisher_->publish(cmd_vel);
            return;
        }

        locked_rotation_sign_ = 0;

        // Only the actual forward-driving leg requires a fresh sighting -- blindly
        // advancing on a stale/possibly-wrong reading is the one part of this that could
        // actually run the robot into something.
        if((get_clock()->now() - last_marker_seen_time_).seconds() > marker_timeout_)
        {
            command_publisher_->publish(geometry_msgs::msg::Twist());
            return;
        }

        cmd_vel.linear.x = std::clamp(k_prop_linear_ * distance, 0.0, maximum_linear_velocity_);
        cmd_vel.angular.z = std::clamp(k_prop_angular_ * heading_error, -maximum_angular_velocity_, maximum_angular_velocity_);
        command_publisher_->publish(cmd_vel);
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<bumperbot_motion::DockingController>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
