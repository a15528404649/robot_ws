#!/usr/bin/env python3

from datetime import datetime, timezone
import json
import math
import threading
import uuid

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String
from yzz_gps.msg import Gps
from yzz_msgs.msg import Sensor


class EnvDataNode(Node):
    """Aggregate legacy environment data without changing its public interface."""

    def __init__(self):
        super().__init__("env_data")
        self._declare_parameters()
        self._initialize_state()
        self._setup_interfaces()
        self._start_timer()

    def _declare_parameters(self):
        self.declare_parameter("device.series_number", "default_robot")
        self.declare_parameter("device.register_number", "default_reg")
        self.declare_parameter("data_threshold", 10)

    def _initialize_state(self):
        self.lock = threading.Lock()
        self.location = ""
        self.temperature = ""
        self.humidity = 0
        self.date_time = ""
        self.mapping = {}
        self.th_data = []
        self.last_initialpose = self.get_clock().now().nanoseconds / 1e9

    def _setup_interfaces(self):
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(String, "/env_data", 10)
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)
        self.create_subscription(LaserScan, "/forwarded_scan", self.scan_callback, scan_qos)
        self.create_subscription(MapMetaData, "/map_metadata", self.metadata_callback, map_qos)
        self.create_subscription(Float32MultiArray, "/map_statistics", self.statistics_callback, map_qos)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.pose_callback, 10)
        self.create_subscription(Gps, "/gps", self.gps_callback, 10)
        self.create_subscription(Sensor, "/sensor_state", self.sensor_callback, 10)

    def _start_timer(self):
        # ROS1 timer is fixed at one second; freq_thdata_upload was not used there.
        self.timer = self.create_timer(1.0, self.timer_callback)

    def gps_callback(self, message):
        with self.lock:
            self.location = f"GPS定位信息: N:{message.n:.6f}  E:{message.e:.6f} "

    def pose_callback(self, message):
        now = self.get_clock().now().nanoseconds / 1e9
        # The ROS1 throttle period was never configured and therefore evaluates to zero.
        if now - self.last_initialpose < 0.0:
            return
        self.last_initialpose = now
        pose = message.pose.pose
        pose_text = (f"position:{pose.position.x:.4f} {pose.position.y:.4f} {pose.position.z:.4f} "
                     f"orientation:{pose.orientation.x:.4f} {pose.orientation.y:.4f} {pose.orientation.z:.4f} {pose.orientation.w:.4f} ")
        with self.lock:
            self.location = f"{self.location} 激光雷达地图定位信息: {pose_text}"

    def sensor_callback(self, message):
        now = self.get_clock().now().to_msg()
        value = datetime.fromtimestamp(now.sec + now.nanosec / 1e9, timezone.utc)
        with self.lock:
            self.temperature = f"{message.temperature:.2f}"
            self.humidity = int(message.humidity)
            self.date_time = value.strftime("%Y-%m-%dT%H:%M:%S.") + f"{now.nanosec // 1_000_000:03d}Z"

    def scan_callback(self, message):
        ranges = [value if math.isfinite(value) else None for value in message.ranges]
        scan = {"angle_min": message.angle_min, "angle_max": message.angle_max, "angle_increment": message.angle_increment, "range_min": message.range_min, "range_max": message.range_max, "sample_ranges": ranges}
        with self.lock:
            self.mapping["ScanData"] = scan

    def metadata_callback(self, message):
        metadata = {"resolution": message.resolution, "width": int(message.width), "height": int(message.height), "origin_x": message.origin.position.x, "origin_y": message.origin.position.y}
        with self.lock:
            self.mapping["MapMetadata"] = metadata

    def statistics_callback(self, message):
        if len(message.data) < 3:
            return
        with self.lock:
            self.mapping["MapStatistics"] = {"unknown_percent": message.data[0], "free_percent": message.data[1], "occupied_percent": message.data[2]}

    def map_callback(self, message):
        with self.lock:
            self.mapping["data"] = [int(cell) for cell in message.data]

    def timer_callback(self):
        threshold = max(1, int(self.get_parameter("data_threshold").value))
        with self.lock:
            if len(self.th_data) < threshold:
                self.th_data.append({"DateTime": self.date_time, "Location": self.location, "Temperature": float(self.temperature or 0.0), "Humidity": self.humidity})
                return
            payload = {
                "id": str(uuid.uuid4()), "sessionId": "env_data", "command": "THDataUpload",
                "params": {
                    "SeriesNumber": self.get_parameter("device.series_number").value,
                    "RegisterNumber": self.get_parameter("device.register_number").value,
                    "THData": self.th_data, "Mapping": self.mapping,
                },
            }
            self.th_data = []
        self.publisher.publish(String(data=json.dumps(payload, ensure_ascii=False, indent=2)))


def main(args=None):
    rclpy.init(args=args)
    node = EnvDataNode()
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
