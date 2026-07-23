#!/usr/bin/env python3

import base64
from datetime import datetime
import json
import math
import threading
import uuid

import cv2
from cv_bridge import CvBridge
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String
from yzz_detect.msg import ImageResults
from yzz_gps.msg import Gps


class ResultNode(Node):
    """Convert detector output into the ROS1-compatible recognition payload."""

    def __init__(self):
        super().__init__("result")
        self._declare_parameters()
        self._initialize_state()
        self._setup_interfaces()

    def _declare_parameters(self):
        self.declare_parameter("device.series_number", "default_robot")
        self.declare_parameter("device.register_number", "default_reg")

    def _initialize_state(self):
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.gps_location = None
        self.mapping = {"amcl_pose": None, "scan_data": None, "map_metadata": None, "map_statistics": None, "occupancy_grid": None}

    def _setup_interfaces(self):
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(String, "/recognition_result", 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.amcl_callback, 10)
        self.create_subscription(ImageResults, "/detect_result_out", self.result_callback, 10)
        self.create_subscription(Gps, "/gps", self.gps_callback, 10)
        self.create_subscription(LaserScan, "/forwarded_scan", self.scan_callback, scan_qos)
        self.create_subscription(MapMetaData, "/map_metadata", self.metadata_callback, map_qos)
        self.create_subscription(Float32MultiArray, "/map_statistics", self.statistics_callback, map_qos)
        self.create_subscription(OccupancyGrid, "/map", self.map_callback, map_qos)

    def gps_callback(self, message):
        with self.lock:
            # ROS1 sends [longitude(E), latitude(N)] as strings.
            self.gps_location = [str(message.e), str(message.n)]

    def amcl_callback(self, message):
        pose = message.pose.pose
        with self.lock:
            self.mapping["amcl_pose"] = {"position": [pose.position.x, pose.position.y, pose.position.z], "orientation": [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]}

    def scan_callback(self, message):
        ranges = [-1 if not math.isfinite(value) else value for value in message.ranges]
        with self.lock:
            self.mapping["scan_data"] = {"angle_min": message.angle_min, "angle_max": message.angle_max, "angle_increment": message.angle_increment, "range_min": message.range_min, "range_max": message.range_max, "ranges": ranges}

    def metadata_callback(self, message):
        with self.lock:
            self.mapping["map_metadata"] = {"resolution": message.resolution, "width": int(message.width), "height": int(message.height), "origin_x": message.origin.position.x, "origin_y": message.origin.position.y}

    def statistics_callback(self, message):
        if len(message.data) < 3:
            return
        with self.lock:
            self.mapping["map_statistics"] = {"unknown_percent": message.data[0], "free_percent": message.data[1], "occupied_percent": message.data[2]}

    def map_callback(self, message):
        with self.lock:
            self.mapping["occupancy_grid"] = {"data": [int(cell) for cell in message.data]}

    @staticmethod
    def ros1_datetime(stamp):
        seconds = stamp.sec + stamp.nanosec / 1_000_000_000.0
        value = datetime.fromtimestamp(seconds)
        return f"{value.year}-{value.month}-{value.day} {value.hour}:{value.minute}:{value.second}:0"

    def image_data(self, image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
            ok, encoded = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                return "JPG encoding failed"
            return base64.b64encode(encoded.tobytes()).decode("ascii")
        except Exception as error:
            self.get_logger().error(f"Image processing failed: {error}")
            return "Image conversion failed"

    def result_callback(self, message):
        with self.lock:
            location = self.gps_location if self.gps_location is not None else "gps lost"
            mapping = {
                "AmclPose": self.mapping["amcl_pose"] if self.mapping["amcl_pose"] is not None else "amcl pose lost",
                "ScanData": self.mapping["scan_data"] if self.mapping["scan_data"] is not None else "scan data lost",
                "MapMetadata": self.mapping["map_metadata"] if self.mapping["map_metadata"] is not None else "map metadata lost",
                "MapStatistics": self.mapping["map_statistics"] if self.mapping["map_statistics"] is not None else "map statistics lost",
                "OccupancyGrid": self.mapping["occupancy_grid"] if self.mapping["occupancy_grid"] is not None else "occupancy grid lost",
            }
        detections = [{"Position": [str(value) for value in item.xyxy], "Score": f"{item.score:.6f}", "Type": str(item.type)} for item in message.results]
        payload = {
            "id": str(uuid.uuid4()), "sessionId": "result", "command": "ResultUpload",
            "params": {
                "SeriesNumber": self.get_parameter("device.series_number").value,
                "RegisterNumber": self.get_parameter("device.register_number").value,
                "DateTime": self.ros1_datetime(message.image.header.stamp),
                "Location": location, "ImageType": 1, "ImageURL": "",
                "Result": detections, "Mapping": mapping, "ImageData": self.image_data(message.image),
            },
        }
        self.publisher.publish(String(data=json.dumps(payload, ensure_ascii=False, indent=2)))


def main(args=None):
    rclpy.init(args=args)
    node = ResultNode()
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
