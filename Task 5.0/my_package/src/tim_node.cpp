#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ListenerNode : public rclcpp::Node {
public:
    ListenerNode() : Node("tim_node") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "interact", 10, 
            std::bind(&ListenerNode::message_callback, this, std::placeholders::_1)
        ); 
        // This node listens to "chatter" topic

        publisher_ = this->create_publisher<std_msgs::msg::String>("response", 10);
        // This node publishes responses on "response" topic
    }

private:
    void message_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Getting colour: '%s'", msg->data.c_str());
        
        auto response_msg = std_msgs::msg::String();
        response_msg.data = "Acknowledged! Which colour next? \n";
        publisher_->publish(response_msg);

        RCLCPP_INFO(this->get_logger(), "->'%s'", response_msg.data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ListenerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
