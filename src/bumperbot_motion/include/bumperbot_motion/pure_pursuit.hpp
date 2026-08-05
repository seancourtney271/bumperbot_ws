#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
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
            // Live obstacle data used to stop before driving into something not
            // accounted for when the path was originally planned.
            rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_subscriber;
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

            // Latest local costmap, used to check the look-ahead point for obstacles.
            nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap;
            // Occupancy value (0-100 scale) at/above which a cell is treated as blocked.
            int occupied_threshold;

            void controlLoop();

            void pathCallback(const nav_msgs::msg::Path::SharedPtr path);

            void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap);

            bool transformPlan(const std::string & frame);

            geometry_msgs::msg::PoseStamped getLookAheadPose(const geometry_msgs::msg::PoseStamped & robot_pose);

            double getCurvature(const geometry_msgs::msg::Pose & look_ahead_pose);

            // Checks whether the given pose falls on an occupied/unknown-beyond-threshold
            // cell of the latest costmap, transforming it into the costmap's frame first
            // if needed. Returns false (assume clear) if no costmap has arrived yet or the
            // pose falls outside the costmap's bounds, rather than blocking the robot on
            // missing data.
            bool isPoseInCollision(const geometry_msgs::msg::PoseStamped & pose);
    };
}