#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace bumperbot_motion
{
    class PDMotionPlanner : public rclcpp::Node
    {
        public:
            PDMotionPlanner();
        private:
            // Grabs the global path from the path planner
            rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber;
            // Publishes the command to the wheels
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr next_pose_publisher;

            std::shared_ptr<tf2_ros::Buffer> tf_buffer;
            std::shared_ptr<tf2_ros::TransformListener> tf_listener;

            rclcpp::TimerBase::SharedPtr control_loop;

            // Proportional Gain
            double k_prop;
            // Differential Gain
            double k_diff;
            double step_size;
            double maximum_linear_velocity;
            double maximum_angular_velocity;
            // Holds path 
            nav_msgs::msg::Path global_plan;

            double previous_angular_error;
            double previous_linear_error;
            rclcpp::Time last_cycle_time;

            // Path planner node name
            std::string path_planner_node_name;

            void controlLoop();

            void pathCallback(const nav_msgs::msg::Path::SharedPtr path);

            bool transformPlan(const std::string & frame);

            geometry_msgs::msg::PoseStamped getNextPose(const geometry_msgs::msg::PoseStamped & robot_pose);

    };
}