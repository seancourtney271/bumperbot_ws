// CTRL SHIFT P > ROS2: Update C++ Properties
#include <algorithm>
#include "bumperbot_motion/pure_pursuit.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace bumperbot_motion
{
    // Constructor for the PD motion planner node.
    // The node receives a global path, transforms it into the robot's current frame,
    // and will eventually produce velocity commands based on a PD control law.
    PurePursuit::PurePursuit() : Node("pure_pursuit_planner_node"),
        look_ahead_distance(0.5), maximum_linear_velocity(0.3), maximum_angular_velocity(1.0), path_planner_node_name("/astar/path"), current_plan_index(0)
    {
        // Declare configurable ROS2 parameters with default values.
        declare_parameter<double>("look_ahead_distance", look_ahead_distance);  // the distance to plan path to ahead
        declare_parameter<double>("maximum_linear_velocity", maximum_linear_velocity);  // max forward speed
        declare_parameter<double>("maximum_angular_velocity", maximum_angular_velocity);  // max rotational speed

        // Topic used to subscribe to the path planner output.
        declare_parameter<std::string>("path_subscriber", path_planner_node_name);

        // Load parameter values after declaration so runtime overrides take effect.
        look_ahead_distance = get_parameter("look_ahead_distance").as_double();
        maximum_linear_velocity = get_parameter("maximum_linear_velocity").as_double();
        maximum_angular_velocity = get_parameter("maximum_angular_velocity").as_double();

        // Subscribe to the planned path. The callback saves the latest path for use by the control loop.
        path_subscriber = create_subscription<nav_msgs::msg::Path>(path_planner_node_name, 10, std::bind(&PurePursuit::pathCallback, this, std::placeholders::_1));

        // Publish velocity commands and the next selected target pose.
        command_publisher = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        carrot_pose_publisher = create_publisher<geometry_msgs::msg::PoseStamped>("/purepursuit/carrot", 10);

        // Initialize TF2 utilities for frame transforms.
        tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        // Create a periodic timer that calls the control loop at 10 Hz.
        control_loop = create_wall_timer(std::chrono::milliseconds(100), std::bind(&PurePursuit::controlLoop, this));

        RCLCPP_INFO(get_logger(), "Setup Complete");
    }

    // Store the latest path received from the planner.
    // This path is treated as the global reference trajectory until a new one arrives.
    void PurePursuit::pathCallback(const nav_msgs::msg::Path::SharedPtr path)
    {
        RCLCPP_INFO(get_logger(), "Path Recieved");
        global_plan = *path;
        current_plan_index = 0;
    }

    // Control loop executed on a fixed timer.
    // It verifies path availability, queries the current robot pose, transforms the plan
    // into the robot's current reference frame, and prepares for PD control computation.
    void PurePursuit::controlLoop()
    {
        // If no path has been received yet, skip this cycle.
        if(global_plan.poses.empty())
        {
            // RCLCPP_INFO(get_logger(), "Path Not Recieved Yet");
            return;
        }

        geometry_msgs::msg::TransformStamped robot_pose;
        try
        {
            // Obtain the current transform from the odom frame to the robot base.
            robot_pose = tf_buffer->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
        }
        catch(tf2::TransformException & ex)
        {
            RCLCPP_WARN(get_logger(), "Could not transform: %s", ex.what());
            return;
        }

        // Convert the received global plan into the robot's current frame so a controller
        // can compute commands relative to the robot's position and orientation.
        if(!transformPlan(robot_pose.header.frame_id))
        {
            RCLCPP_ERROR(get_logger(), "Unable to transform Plan into robot's frame.");
            return;
        }

        // Currently have robot pose and global plan wrt to the same reference frame


        geometry_msgs::msg::PoseStamped robot_pose_stamped;
        // Tie robot pose stamped to robot pose frame
        robot_pose_stamped.header.frame_id = robot_pose.header.frame_id;
        robot_pose_stamped.pose.position.x = robot_pose.transform.translation.x;
        robot_pose_stamped.pose.position.y = robot_pose.transform.translation.y;
        robot_pose_stamped.pose.orientation = robot_pose.transform.rotation;

        // Takes in as input current pose of the robot
        auto look_ahead_pose = getLookAheadPose(robot_pose_stamped);

        // Check if we reached the goal
        double dx = look_ahead_pose.pose.position.x - robot_pose_stamped.pose.position.x;
        double dy = look_ahead_pose.pose.position.y - robot_pose_stamped.pose.position.y;
        // Calculate distance of each pose
        double distance = std::sqrt(dx * dx + dy * dy);
        // Did we reach goal
        if(distance <= 0.1)
        {
            // Reached Goal Pose
            RCLCPP_INFO(get_logger(), "Goal Reached!");
            global_plan.poses.clear();
            return;
        }

        // Publish look ahead pose
        carrot_pose_publisher->publish(look_ahead_pose);

        // Get the error of look ahead pose and robot pose
        tf2::Transform robot_tf, look_ahead_pose_tf, look_ahead_pose_robot_tf;
        tf2::fromMsg(robot_pose_stamped.pose, robot_tf);
        tf2::fromMsg(look_ahead_pose.pose, look_ahead_pose_tf);
        // Gets the error of the pose of the robot and the look ahead pose we want to reach
        look_ahead_pose_robot_tf = robot_tf.inverse() * look_ahead_pose_tf;

        // Get the best curvature for movement to look ahead pose
        tf2::toMsg(look_ahead_pose_robot_tf, look_ahead_pose.pose);
        double curvature = getCurvature(look_ahead_pose.pose);

        // Create command vel
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = maximum_linear_velocity;
        cmd_vel.angular.z = curvature * maximum_angular_velocity;

        // Publish cmd_Vel
        command_publisher->publish(cmd_vel);
    }

    // Transform the stored plan into the requested coordinate frame.
    // This rewrites each pose so the global path is expressed relative to the robot's current frame.
    bool PurePursuit::transformPlan(const std::string & frame)
    {
        if(global_plan.header.frame_id == frame)
        {
            return true;
        }

        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer->lookupTransform(frame, global_plan.header.frame_id, tf2::TimePointZero);
        }
        catch(tf2::LookupException & ex)
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Couldn't transform plan from frame " << global_plan.header.frame_id << " to " << frame);
            return false;
        }

        for(auto & pose : global_plan.poses)
        {
            // Transform each path pose into the target frame.
            tf2::doTransform(pose, pose, transform);
        }

        global_plan.header.frame_id = frame;
        return true;
    }

    geometry_msgs::msg::PoseStamped PurePursuit::getLookAheadPose(const geometry_msgs::msg::PoseStamped & robot_pose)
    {
        // Start with a fallback target: the last pose in the plan.
        // This guarantees we always return a valid pose even if no lookahead point is found.
        geometry_msgs::msg::PoseStamped look_ahead_pose = global_plan.poses.back();
        look_ahead_pose.header = global_plan.header;

        // If the saved search index is outside the plan, reset it to the beginning.
        // This can happen when a new plan is received or the current index reached the end.
        if(current_plan_index >= global_plan.poses.size())
        {
            current_plan_index = 0;
        }

        // Search forward from the current plan index for the next waypoint that is
        // farther than the configured lookahead distance from the robot.
        // This avoids selecting a target behind the robot or one it has already passed.
        for(std::size_t i = current_plan_index; i < global_plan.poses.size(); ++i)
        {
            const auto & pose = global_plan.poses[i];
            double dx = pose.pose.position.x - robot_pose.pose.position.x;
            double dy = pose.pose.position.y - robot_pose.pose.position.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            if(distance > look_ahead_distance)
            {
                // Found the next lookahead pose.
                look_ahead_pose = pose;
                current_plan_index = i;
                break;
            }
        }

        // Compute orientation for the look ahead pose to point toward the next waypoint
        if(current_plan_index + 1 < global_plan.poses.size())
        {
            const auto & next_pose = global_plan.poses[current_plan_index + 1];
            double dx = next_pose.pose.position.x - look_ahead_pose.pose.position.x;
            double dy = next_pose.pose.position.y - look_ahead_pose.pose.position.y;
            double yaw = std::atan2(dy, dx);
            
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            look_ahead_pose.pose.orientation = tf2::toMsg(q);
        }

        return look_ahead_pose;
    }
    double PurePursuit::getCurvature(const geometry_msgs::msg::Pose & look_ahead_pose)
    {
        const double L = (look_ahead_pose.position.x * look_ahead_pose.position.x) + (look_ahead_pose.position.y * look_ahead_pose.position.y);
        if (L > 0.001)
        {
            return 2 * look_ahead_pose.position.y / L;
        }
        else
        {
            return 0.0;
        }
    }
}

int main(int argc, char **argv)
{
    // Initialize the ROS 2 client library before creating the node.
    rclcpp::init(argc, argv);

    // Create the PD motion planner node and keep it alive while ROS is running.
    auto node = std::make_shared<bumperbot_motion::PurePursuit>();
    rclcpp::spin(node);

    // Cleanly shutdown ROS2 resources after node exit.
    rclcpp::shutdown();
    return 0;
}