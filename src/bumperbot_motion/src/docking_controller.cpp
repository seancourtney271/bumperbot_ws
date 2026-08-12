// CTRL SHIFT P > ROS2: Update C++ Properties
#include <algorithm>
#include "bumperbot_motion/pd_motion_planner.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace bumperbot_motion
{
    // Constructor for the PD motion planner node.
    // The node receives a global path, transforms it into the robot's current frame,
    // and will eventually produce velocity commands based on a PD control law.
    PDMotionPlanner::PDMotionPlanner() : Node("pd_motion_planner_node"),
        k_prop(2.0), k_diff(1.0), step_size(0.2), maximum_linear_velocity(0.3), maximum_angular_velocity(1.0), path_planner_node_name("/astar/path"), previous_linear_error(0.0), previous_angular_error(0.0)
    {
        // Declare configurable ROS2 parameters with default values.
        declare_parameter<double>("kp", k_prop);  // proportional gain
        declare_parameter<double>("kd", k_diff);  // differential gain
        declare_parameter<double>("step_size", step_size);  // distance step for path point selection
        declare_parameter<double>("maximum_linear_velocity", maximum_linear_velocity);  // max forward speed
        declare_parameter<double>("maximum_angular_velocity", maximum_angular_velocity);  // max rotational speed

        // Topic used to subscribe to the path planner output.
        declare_parameter<std::string>("path_subscriber", path_planner_node_name);

        // Load parameter values after declaration so runtime overrides take effect.
        k_prop = get_parameter("kp").as_double();
        k_diff = get_parameter("kd").as_double();
        step_size = get_parameter("step_size").as_double();
        maximum_linear_velocity = get_parameter("maximum_linear_velocity").as_double();
        maximum_angular_velocity = get_parameter("maximum_angular_velocity").as_double();

        // Subscribe to the planned path. The callback saves the latest path for use by the control loop.
        path_subscriber = create_subscription<nav_msgs::msg::Path>(path_planner_node_name, 10, std::bind(&PDMotionPlanner::pathCallback, this, std::placeholders::_1));

        // Publish velocity commands and the next selected target pose.
        command_publisher = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        next_pose_publisher = create_publisher<geometry_msgs::msg::PoseStamped>("/pd/next_pose", 10);

        // Initialize TF2 utilities for frame transforms.
        tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        // Create a periodic timer that calls the control loop at 10 Hz.
        control_loop = create_wall_timer(std::chrono::milliseconds(100), std::bind(&PDMotionPlanner::controlLoop, this));

        // Last time the control loop was executed
        last_cycle_time = get_clock()->now();
        RCLCPP_INFO(get_logger(), "Setup Complete");
    }

    // Store the latest path received from the planner.
    // This path is treated as the global reference trajectory until a new one arrives.
    void PDMotionPlanner::pathCallback(const nav_msgs::msg::Path::SharedPtr path)
    {
        RCLCPP_INFO(get_logger(), "Path Recieved");
        global_plan = *path;
    }

    // Control loop executed on a fixed timer.
    // It verifies path availability, queries the current robot pose, transforms the plan
    // into the robot's current reference frame, and prepares for PD control computation.
    void PDMotionPlanner::controlLoop()
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

        geometry_msgs::msg::PoseStamped robot_pose_stamped;
        // Tie robot pose stamped to robot pose frame
        robot_pose_stamped.header.frame_id = robot_pose.header.frame_id;
        robot_pose_stamped.pose.position.x = robot_pose.transform.translation.x;
        robot_pose_stamped.pose.position.y = robot_pose.transform.translation.y;
        robot_pose_stamped.pose.orientation = robot_pose.transform.rotation;

        // Takes in as input current pose of the robot
        auto next_pose = getNextPose(robot_pose_stamped);

        // Check if we reached the goal
        double dx = next_pose.pose.position.x - robot_pose_stamped.pose.position.x;
        double dy = next_pose.pose.position.y - robot_pose_stamped.pose.position.y;
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

        // Publish next pose
        next_pose_publisher->publish(next_pose);

        // Get the error of next pose and robot pose
        tf2::Transform robot_tf, next_pose_tf, next_pose_robot_tf;
        tf2::fromMsg(robot_pose_stamped.pose, robot_tf);
        tf2::fromMsg(next_pose.pose, next_pose_tf);
        // Gets the error of the pose of the robot and the next pose we want to reach
        next_pose_robot_tf = robot_tf.inverse() * next_pose_tf;
        double linear_error = next_pose_robot_tf.getOrigin().getX();
        double angular_error = next_pose_robot_tf.getOrigin().getY();

        // grab cycle time for error 
        double dt = (get_clock()->now() - last_cycle_time).seconds();
        double linear_error_derivative = (linear_error - previous_linear_error) / dt;
        double angular_error_derivative = (angular_error - previous_angular_error) / dt;

        // Calculate Proportional and Differential Control
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = std::clamp(k_prop * linear_error + k_diff * linear_error_derivative, -maximum_linear_velocity, maximum_linear_velocity);
        cmd_vel.angular.z = std::clamp(k_prop * angular_error + k_diff * angular_error_derivative, -maximum_angular_velocity, maximum_angular_velocity);

        // Publish vel cmd
        command_publisher->publish(cmd_vel);
        // RCLCPP_INFO(get_logger(), "Published CMD VEL going next loop");
        last_cycle_time = get_clock()->now();
        previous_linear_error = linear_error;
        previous_angular_error = angular_error;
    }

    // Transform the stored plan into the requested coordinate frame.
    // This rewrites each pose so the global path is expressed relative to the robot's current frame.
    bool PDMotionPlanner::transformPlan(const std::string & frame)
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

    geometry_msgs::msg::PoseStamped PDMotionPlanner::getNextPose(const geometry_msgs::msg::PoseStamped & robot_pose)
    {
        // Init next pose we want to reach (IE Last Pose on the global plan)
        auto next_pose = global_plan.poses.back();
        // Iterate throug the poses and find the pose that less than the step size (in reverse)
        for(auto pose_it = global_plan.poses.rbegin(); pose_it != global_plan.poses.rend(); ++pose_it)
        {
            double dx = pose_it->pose.position.x - robot_pose.pose.position.x;
            double dy = pose_it->pose.position.y - robot_pose.pose.position.y;
            // Calculate distance of each pose
            double distance = std::sqrt(dx * dx + dy * dy);
            // Find pose less than step size
            if(distance > step_size)
            {
                // If bigger than step size
                next_pose = *pose_it;
                // Continue
            }
            else
            {
                // Found the pose
                break;
            }
        }
        return next_pose;
    }
}

int main(int argc, char **argv)
{
    // Initialize the ROS 2 client library before creating the node.
    rclcpp::init(argc, argv);

    // Create the PD motion planner node and keep it alive while ROS is running.
    auto node = std::make_shared<bumperbot_motion::PDMotionPlanner>();
    rclcpp::spin(node);

    // Cleanly shutdown ROS2 resources after node exit.
    rclcpp::shutdown();
    return 0;
}