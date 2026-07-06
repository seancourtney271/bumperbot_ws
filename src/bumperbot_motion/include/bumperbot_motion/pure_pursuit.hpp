#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace bumperbot_motion
{
    class PurePursuit : public rclcpp::Node
    {
        public:
            PurePursuit();
        private:
            // Grabs the global path from the path planner
            rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber;
            // Publishes the command to the wheels
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr carrot_pose_publisher;

            std::shared_ptr<tf2_ros::Buffer> tf_buffer;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener;

            rclcpp::TimerBase::SharedPtr control_loop;

            // How far the look ahead distance will plan for
            double look_ahead_distance;
            double maximum_linear_velocity;
            double maximum_angular_velocity;
            // Holds path 
            nav_msgs::msg::Path global_plan;

            // Path planner node name
            std::string path_planner_node_name;
            // Index of the first path point still ahead of the robot.
            std::size_t current_plan_index;

            void controlLoop();

            void pathCallback(const nav_msgs::msg::Path::SharedPtr path);

            bool transformPlan(const std::string & frame);

            geometry_msgs::msg::PoseStamped getLookAheadPose(const geometry_msgs::msg::PoseStamped & robot_pose);

            double getCurvature(const geometry_msgs::msg::Pose & look_ahead_pose);
    };
}