#include <chrono>    // chrono_literals
#include <functional>// bind
#include <memory>    // make_shared
#include <string>    // string

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Subscriber : public rclcpp::Node {
public:
    Subscriber() : Node("subscriber") {
        subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&Subscriber::callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Subscriber has been started");
    }

private:
    void callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), ("Message received: " + msg->data).c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto subscriber = std::make_shared<Subscriber>();
    rclcpp::spin(subscriber);
    rclcpp::shutdown();
    return 0;
}
