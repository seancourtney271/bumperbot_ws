#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


using std::placeholders::_1;

class LabQosSubscriber : public rclcpp::Node
{
public:
  LabQosSubscriber() : Node("simple_qos_subscriber"), qos_profile_sub_(10)
  {
    // Declare 2 types of QoS nodes
    declare_parameter<std::string>("reliability", "system_default");
    declare_parameter<std::string>("durability", "system_default");

    // Grab the QoS parameters for setting
    const auto reliability = get_parameter("reliability").as_string();
    const auto durability = get_parameter("durability").as_string();
    // Handle all reliability policies
    if(reliability == "best_effort")
    {
        qos_profile_sub_.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        RCLCPP_INFO(get_logger(), "[Reliability]: Best Effort");
    }
    else if (reliability == "reliable")
    {
        qos_profile_sub_.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        RCLCPP_INFO(get_logger(), "[Reliability]: Reliable");
    }
    else if (reliability == "system_default")
    {
        qos_profile_sub_.reliability(RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT);
        RCLCPP_INFO(get_logger(), "[Reliability]: System Default");
    }
    else
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Selected Reliability QoS: " << reliability << " doesn't exist.");
        return;
    }

    //Handle all durability policies
    if(durability == "volitile")
    {
        qos_profile_sub_.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        RCLCPP_INFO(get_logger(), "[Durability]: Volitile");
    }
    else if (durability == "transient local")
    {
        qos_profile_sub_.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        RCLCPP_INFO(get_logger(), "[Durability]: Transient Local");
    }
    else if (durability == "best available")
    {
        qos_profile_sub_.durability(RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE);
        RCLCPP_INFO(get_logger(), "[Durability]: Best Available");
    }
    else if (durability == "system_default")
    {
        qos_profile_sub_.durability(RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT);
        RCLCPP_INFO(get_logger(), "[Durability]: System Default");
    }
    else
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Selected Durability QoS: " << durability << " doesn't exist.");
        return;
    }
    sub_ = create_subscription<std_msgs::msg::String>(
        "chatter", qos_profile_sub_, std::bind(&LabQosSubscriber::msgCallback, this, _1));
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  rclcpp::QoS qos_profile_sub_;

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