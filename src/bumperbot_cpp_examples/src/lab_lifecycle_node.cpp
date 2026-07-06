#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>
#include <thread>

using std::placeholders::_1;
using namespace std::chrono_literals;

class LabLifeCycleNode : public rclcpp_lifecycle::LifecycleNode
{
    public:
        explicit LabLifeCycleNode(const std::string & node_name, bool instra_process_comms = false)
        : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(instra_process_comms))
        {
            // Bring Lifecycle node to the Initial State
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
        {
            // Brings the Lifecycle Node into the Unconfigured State 
            sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&LabLifeCycleNode::msgCallback, this, _1));
            RCLCPP_INFO(get_logger(), "Lifecycle Node on_configure() called.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
        {
            // Brings the Lifecycle Node into the Finalized State before destruction of Node 
            sub_.reset();
            RCLCPP_INFO(get_logger(), "Lifecycle Node on_shutdown() called.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
        {
            // Brings the Lifecycle Node into the Unconfigured State from 
            sub_.reset();
            RCLCPP_INFO(get_logger(), "Lifecycle Node on_cleanup() called.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state)
        {
            // Brings the Lifecycle Node into the Active State (Normal Processing)
            LifecycleNode::on_activate(state);
            RCLCPP_INFO(get_logger(), "Lifecycle Node on_activate() called.");
            std::this_thread::sleep_for(2s);
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state)
        {
            // Brings the Lifecycle Node into the Inactive State from the Active State
            LifecycleNode::on_deactivate(state);
            RCLCPP_INFO(get_logger(), "Lifecycle Node on_deactivate() called.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        void msgCallback(const std_msgs::msg::String & msg)
        {
            auto state = get_current_state();
            if(state.label() == "active")
            {
                RCLCPP_INFO_STREAM(get_logger(), "Lifecycle Node heard: " << msg.data.c_str());
            }
        }
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

};

int main(int argc, char * argv[])
{
    // Pass argc and argv to rclcpp
    rclcpp::init(argc, argv);
    // Create a single thread executor which handles the node
    rclcpp::executors::SingleThreadedExecutor ste;
    // Create our Lifecycle node
    std::shared_ptr<LabLifeCycleNode> lab_lifecycle_node = std::make_shared<LabLifeCycleNode>("Lab_Lifecycle_Node");
    // Add the node to the single thread executor but convert it to a node base interface
    ste.add_node(lab_lifecycle_node->get_node_base_interface());
    // Run until shutdown
    ste.spin();
    // Handle Shutdown
    rclcpp::shutdown();
    return 0;
}