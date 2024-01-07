#!/usr/bin/env python3

import typing as t

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Subscriber(Node):  # type: ignore
    def __init__(self) -> None:
        super().__init__("subscriber")
        self.subscription_ = self.create_subscription(String, "topic", self.callback, 10)
        self.get_logger().info("Subscriber has been started")

    def callback(self, msg: String) -> None:
        self.get_logger().info(f"Message received: {msg.data}")


def main(arguments: t.List[str] | None = None) -> None:
    rclpy.init(args=arguments)
    subscriber = Subscriber()
    rclpy.spin(subscriber)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
