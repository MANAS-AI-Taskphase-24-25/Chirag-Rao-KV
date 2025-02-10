#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>
#include <string>
#include <thread>

class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("bob_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("interact", 10);
        input_thread_ = std::thread(&MyNode::publish_user_message, this);
    }

    ~MyNode() {
        if (input_thread_.joinable()) {
            input_thread_.join();  
        }
    }

private:
    void publish_user_message() {
        std::string user_input;
        while (rclcpp::ok()) {  
            std::cout << "Enter message to be sent to TIM (colour): ";
            std::getline(std::cin, user_input);

            if (user_input.empty()) continue;  

            auto message = std_msgs::msg::String();
            message.data = user_input;

            RCLCPP_INFO(this->get_logger(), "SENT TO TIM: '%s'", message.data.c_str());
            publisher_->publish(message);
        }
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::thread input_thread_;  
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();

    rclcpp::spin(node);  

    rclcpp::shutdown();
    return 0;
}
