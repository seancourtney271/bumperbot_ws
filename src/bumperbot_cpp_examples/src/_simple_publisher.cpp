#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>

using namespace std::chrono_literals;

class LabQoSPublisher : public rclcpp::Node
{
    public:
        LabQoSPublisher() : Node("first_publisher"), counter_(0)
        {
            pub_ = create_publisher<std_msgs::msg::String>("First_Publish", 10);
            timer_ = create_wall_timer(1s, std::bind(&LabQoSPublisher::timerCallback, this));
            RCLCPP_INFO(get_logger(), "Publishing at 1Hz.");
        }

        void timerCallback()
        {
            std_msgs::msg::String message = std_msgs::msg::String();
            message.data = "Counter: " + std::to_string(counter_++);
            pub_->publish(message);
        }
    private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    unsigned int counter_;

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LabQoSPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}