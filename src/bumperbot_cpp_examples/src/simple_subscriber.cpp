#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


using std::placeholders::_1;

class LabQosSubscriber : public rclcpp::Node
{
public:
  LabQosSubscriber() : Node("simple_subscriber")
  {
    sub_ = create_subscription<std_msgs::msg::String>(
        "chatter", 10, std::bind(&LabQosSubscriber::msgCallback, this, _1));
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

  void msgCallback(const std_msgs::msg::String &msg) const
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: " << msg.data.c_str());
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LabQosSubscriber>());
  rclcpp::shutdown();
  return 0;
}