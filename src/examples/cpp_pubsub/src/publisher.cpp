#include <chrono>    // chrono_literals
#include <functional>// bind
#include <memory>    // make_shared
#include <string>    // to_string

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node {
public:
    Publisher() : Node("publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&Publisher::timer_callback, this));
        count_ = 0;
        RCLCPP_INFO(this->get_logger(), "Publisher has been started");
    }

private:
    void timer_callback() {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello World number " + std::to_string(count_++);
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), ("Message has been published " + msg.data).c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto publisher = std::make_shared<Publisher>();
    rclcpp::spin(publisher);
    rclcpp::shutdown();
    return 0;
}
