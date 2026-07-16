#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanSanitizer(Node):

    def __init__(self):
        super().__init__('scan_sanitizer')

        self.output_points = 1980
        self.output_angle_min = -math.pi
        self.output_angle_max = math.pi
        self.output_angle_increment = (
            self.output_angle_max - self.output_angle_min
        ) / (self.output_points - 1)

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.scan_callback,
            10,
        )

        self.publisher = self.create_publisher(
            LaserScan,
            '/scan',
            10,
        )

        self.get_logger().info(
            '固定重采样已启动：/scan_raw -> /scan，1980 points'
        )

    def scan_callback(self, msg):
        if not msg.ranges or msg.angle_increment == 0.0:
            return

        output = LaserScan()

        output.header = msg.header
        output.angle_min = self.output_angle_min
        output.angle_max = self.output_angle_max
        output.angle_increment = self.output_angle_increment
        output.time_increment = (
            msg.scan_time / (self.output_points - 1)
            if self.output_points > 1 else 0.0
        )
        output.scan_time = msg.scan_time
        output.range_min = msg.range_min
        output.range_max = msg.range_max

        ranges = []

        for i in range(self.output_points):
            target_angle = (
                self.output_angle_min
                + i * self.output_angle_increment
            )

            source_index = round(
                (target_angle - msg.angle_min)
                / msg.angle_increment
            )

            if 0 <= source_index < len(msg.ranges):
                value = msg.ranges[source_index]

                if (
                    not math.isfinite(value)
                    or value <= 0.0
                    or value < msg.range_min
                    or value > msg.range_max
                ):
                    value = math.inf
            else:
                value = math.inf

            ranges.append(value)

        output.ranges = ranges
        output.intensities = []

        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = ScanSanitizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
