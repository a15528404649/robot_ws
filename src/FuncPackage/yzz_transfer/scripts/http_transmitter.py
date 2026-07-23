#!/usr/bin/env python3

import json
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from yzz_transfer.msg import WebsocketSignal


class HttpTransmitter(Node):
    """Send ROS1-compatible transfer payloads when HTTP is explicitly enabled."""

    def __init__(self):
        super().__init__("http_transmitter")
        self._declare_parameters()
        self._setup_interfaces()

    def _declare_parameters(self):
        self.declare_parameter("ip_port", "43.138.191.89:7201")
        self.declare_parameter("http_enabled", False)
        self.declare_parameter("http_timeout", 30.0)

    def _setup_interfaces(self):
        self.websocket_publisher = self.create_publisher(WebsocketSignal, "/websocket_signal", 10)
        self.create_subscription(String, "/heartbeat", self.heartbeat_callback, 100)
        self.create_subscription(String, "/recognition_result", self.result_callback, 100)
        self.create_subscription(String, "/env_data", self.env_data_callback, 100)

    @staticmethod
    def has_null(value):
        if value is None:
            return True
        if isinstance(value, dict):
            return any(HttpTransmitter.has_null(item) for item in value.values())
        if isinstance(value, list):
            return any(HttpTransmitter.has_null(item) for item in value)
        return False

    def post(self, payload, path):
        if not self.get_parameter("http_enabled").value:
            self.get_logger().warning("HTTP transmission is disabled; message was not sent", throttle_duration_sec=30.0)
            return None
        url = f"http://{self.get_parameter(ip_port).value}{path}"
        request = urllib.request.Request(url, data=payload.data.encode("utf-8"), headers={"Content-Type": "application/json;charset=utf-8"}, method="POST")
        try:
            with urllib.request.urlopen(request, timeout=self.get_parameter("http_timeout").value) as response:
                return response.read().decode("utf-8")
        except (urllib.error.URLError, TimeoutError, OSError) as error:
            self.get_logger().warning(f"POST failed for {url}: {error}")
            return None

    def heartbeat_callback(self, payload):
        response = self.post(payload, "/external-api/device/register")
        if response is None:
            return
        try:
            data = json.loads(response)
        except json.JSONDecodeError:
            self.get_logger().warning("Invalid heartbeat HTTP response")
            return
        if self.has_null(data):
            self.get_logger().warning("Heartbeat response contains null values")
            return
        response_data = data.get("data", {})
        register_number = response_data.get("RegisterNumber")
        if register_number:
            # ROS1 rewrote a YAML file here. Do not overwrite user configuration in ROS2.
            self.get_logger().info(f"Server returned RegisterNumber: {register_number}")
        message = WebsocketSignal()
        message.signal = int(response_data.get("IfWebsocketConnect", 0))
        self.websocket_publisher.publish(message)

    def result_callback(self, payload):
        self.post(payload, "/external-api/device-recognition/create")

    def env_data_callback(self, payload):
        self.post(payload, "/external-api/device-monitor/create")


def main(args=None):
    rclpy.init(args=args)
    node = HttpTransmitter()
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
