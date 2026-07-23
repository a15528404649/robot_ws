#!/usr/bin/env python3

import json
import socket
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from yzz_gps.msg import Gps
from yzz_msgs.msg import Battery


class HeartbeatNode(Node):
    """Publish the original heartbeat payload at its configured rate."""

    def __init__(self):
        super().__init__("heartbeat")
        self._declare_parameters()
        self._initialize_state()
        self._setup_interfaces()
        self._start_timer()

    def _declare_parameters(self):
        defaults = {
            "freq_status_report": 1.0 / 3.0,
            "network_interface": "wlan0",
            "device.series_number": "未知", "device.type": "未知", 
            "device.model": "未知", "device.name": "未知", 
            "device.battery_capacity": "未知", "device.run_model": "未知", 
            "device.soft_version": "未知", "device.StatusCode": "未知",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _initialize_state(self):
        self.battery_level = 0
        self.location = []

    def _setup_interfaces(self):
        self.publisher = self.create_publisher(String, "/heartbeat", 10)
        self.create_subscription(Battery, "/battery_state", self.battery_callback, 10)
        self.create_subscription(Gps, "/gps", self.gps_callback, 10)
        self.mac, self.ip = self.network_identity(self.get_parameter("network_interface").value)

    def _start_timer(self):
        frequency = self.get_parameter("freq_status_report").value
        if frequency <= 0.0:
            self.get_logger().warning("freq_status_report must be positive; using 1/3 Hz")
            frequency = 1.0 / 3.0
        self.timer = self.create_timer(1.0 / frequency, self.publish_heartbeat)

    @staticmethod
    def network_identity(interface):
        try:
            with open(f"/sys/class/net/{interface}/address", encoding="ascii") as stream:
                mac = stream.read().strip()
        except OSError:
            mac = ""
        try:
            ip = socket.gethostbyname(socket.gethostname())
        except socket.gaierror:
            ip = ""
        return mac, ip

    def battery_callback(self, message):
        self.battery_level = ord(message.percentage) if isinstance(message.percentage, str) else int(message.percentage)

    def gps_callback(self, message):
        # ROS1 encoded Location as [longitude(E), latitude(N)] strings.
        self.location = [str(message.e), str(message.n)]

    def publish_heartbeat(self):
        parameter = lambda name: self.get_parameter(name).value
        run_model = parameter("device.run_model")
        payload = {
            "id": str(uuid.uuid4()),
            "sessionId": "heartbeat", "command": "StatusReport",
            "params": {
                "SeriesNumber": parameter("device.series_number"),
                "MAC": self.mac, "IP": self.ip,
                "DeviceType": parameter("device.type"),
                # Kept intentionally: ROS1 assigned DeviceModel from run_model.
                "DeviceModel": run_model,
                "DeviceName": parameter("device.name"),
                "BatteryCapacity": parameter("device.battery_capacity"),
                "BatteryLevel": self.battery_level, "SignalLevel": 100,
                "Location": self.location, "RunModel": run_model,
                "SoftVersion": parameter("device.soft_version"),
                "StatusCode": parameter("device.StatusCode"),
            },
        }
        self.publisher.publish(String(data=json.dumps(payload, ensure_ascii=False, indent=2)))


def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatNode()
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
