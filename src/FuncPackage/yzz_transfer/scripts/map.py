#!/usr/bin/env python3
"""Forward scan data and derive map metadata/statistics for YZZ transfer nodes."""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import MapMetaData, OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class MapAndScanForwarder(Node):
    """Forward the existing scan and publish derived map transfer data."""

    def __init__(self):
        super().__init__('map_and_scan_forwarder')
        self._setup_interfaces()

    def _setup_interfaces(self):
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.metadata_pub = self.create_publisher(MapMetaData, '/map_metadata', map_qos)
        self.stats_pub = self.create_publisher(Float32MultiArray, '/map_statistics', map_qos)
        self.scan_pub = self.create_publisher(LaserScan, '/forwarded_scan', scan_qos)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, scan_qos)

    def scan_callback(self, scan):
        self.scan_pub.publish(scan)

    def map_callback(self, occupancy_grid):
        metadata = MapMetaData()
        metadata.map_load_time = self.get_clock().now().to_msg()
        metadata.resolution = occupancy_grid.info.resolution
        metadata.width = occupancy_grid.info.width
        metadata.height = occupancy_grid.info.height
        metadata.origin = occupancy_grid.info.origin
        self.metadata_pub.publish(metadata)

        cells = np.asarray(occupancy_grid.data, dtype=np.int8)
        total = cells.size
        statistics = Float32MultiArray()
        if total:
            statistics.data = [
                float(np.count_nonzero(cells == -1) * 100.0 / total),
                float(np.count_nonzero(cells == 0) * 100.0 / total),
                float(np.count_nonzero(cells == 100) * 100.0 / total),
            ]
        else:
            statistics.data = [0.0, 0.0, 0.0]
        self.stats_pub.publish(statistics)


def main():
    rclpy.init()
    node = MapAndScanForwarder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
