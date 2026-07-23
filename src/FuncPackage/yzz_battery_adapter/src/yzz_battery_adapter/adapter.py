#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from yzz_bunker_mini_base.msg import BunkerStatus
from yzz_msgs.msg import Battery


class BatteryAdapter(Node):
    """Convert BUNKER voltage feedback into the legacy Battery message."""

    def __init__(self):
        super().__init__("battery_adapter")
        self._declare_parameters()
        self._setup_interfaces()

    def _declare_parameters(self):
        for name, value in (("status_topic", "/bunker_status"), ("battery_topic", "/battery_state"), ("empty_voltage", 24.0), ("low_voltage", 25.0), ("low_percentage", 15.0), ("full_voltage", 29.4)):
            self.declare_parameter(name, value)

    def _setup_interfaces(self):
        self.publisher = self.create_publisher(Battery, self.get_parameter("battery_topic").value, 10)
        self.create_subscription(BunkerStatus, self.get_parameter("status_topic").value, self.status_callback, 10)

    def percentage(self, voltage):
        empty = self.get_parameter("empty_voltage").value
        low = self.get_parameter("low_voltage").value
        low_percent = self.get_parameter("low_percentage").value
        full = self.get_parameter("full_voltage").value
        if not empty < low < full or not 0.0 <= low_percent <= 100.0:
            self.get_logger().error("Invalid battery voltage curve parameters", throttle_duration_sec=30.0)
            return 0
        if voltage <= empty:
            return 0
        if voltage < low:
            return round((voltage - empty) * low_percent / (low - empty))
        if voltage < full:
            return round(low_percent + (voltage - low) * (100.0 - low_percent) / (full - low))
        return 100

    def status_callback(self, status):
        message = Battery()
        message.voltage = float(status.battery_voltage)
        # yzz_msgs/Battery uses ROS2 char, represented as an integer byte in rclpy.
        message.percentage = max(0, min(100, self.percentage(message.voltage)))
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
