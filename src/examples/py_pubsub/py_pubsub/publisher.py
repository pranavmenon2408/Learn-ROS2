#!/usr/bin/env python3
import typing as t

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Publisher(Node):  # type: ignore
    def __init__(self) -> None:
        super().__init__("publisher")
        self.publisher_ = self.create_publisher(String, "topic", 10)
        self.timer_ = self.create_timer(0.5, self.timer_callback)
        self.count_ = 0
        self.get_logger().info("Publisher has been started")

    def timer_callback(self) -> None:
        msg = String()
        msg.data = f"Hello World number {self.count_}"
        self.publisher_.publish(msg)
        self.count_ += 1
        self.get_logger().info(f"Message has been published {msg.data}")


def main(arguments: t.List[str] | None = None) -> None:
    rclpy.init(args=arguments)
    publisher = Publisher()
    rclpy.spin(publisher)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
