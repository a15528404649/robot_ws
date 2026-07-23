#!/usr/bin/env python3

import asyncio
import json
import subprocess
import threading

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Int32
from yzz_gps.msg import Gps
from yzz_msgs.msg import GetHolder, SetHolder
from yzz_transfer.msg import WebsocketSignal


class ControlNode(Node):
    """Expose the original control WebSocket commands through ROS2 interfaces."""

    def __init__(self):
        super().__init__("control")
        self._declare_parameters()
        self._initialize_state()
        self._setup_interfaces()

    def _declare_parameters(self):
        for name, value in (("device.server_ip_port", "43.138.191.89:7201"), ("device.register_number", "default_reg"), ("websocket_enabled", False), ("websocket_heartbeat_sec", 30.0)):
            self.declare_parameter(name, value)

    def _initialize_state(self):
        self.holder = GetHolder()
        self.ws_thread = None
        self.ws_lock = threading.Lock()

    def _setup_interfaces(self):
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 100)
        self.goal_pub = self.create_publisher(PointStamped, "/clicked_point", 10)
        self.holder_pub = self.create_publisher(SetHolder, "/SetHolder", 10)
        self.video_pub = self.create_publisher(Int32, "/videostream_push_status", 10)
        self.navigate_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.navigation_goal_handle = None
        self.create_subscription(WebsocketSignal, "/websocket_signal", self.websocket_signal_callback, 10)
        self.create_subscription(GetHolder, "/GetHolder", self.holder_callback, 10)

    def holder_callback(self, message):
        self.holder = message

    @staticmethod
    def response(request, code=0, message="执行成功", data=None):
        reply = {"id": request.get("id", ""), "code": code, "message": message, "command": request.get("command", ""), "memo": ""}
        if data is not None:
            reply["data"] = data
        return reply

    @staticmethod
    def vector(value):
        return float(value.get("x", 0.0)), float(value.get("y", 0.0)), float(value.get("z", 0.0))

    def handle_command(self, raw):
        try:
            request = json.loads(raw)
            command = request.get("command", "")
            params = request.get("params", {})
        except (TypeError, json.JSONDecodeError):
            return {"id": "", "code": 400, "message": "invalid JSON", "command": "", "memo": ""}
        if command == "Ping":
            return self.response(request, data={"command": "Pong"})
        if command in ("StartVideoStream", "StopVideoStream"):
            self.video_pub.publish(Int32(data=1 if command == "StartVideoStream" else 0))
            return self.response(request)
        if command == "SetMoveParam":
            message = Twist()
            message.linear.x, message.linear.y, message.linear.z = self.vector(params.get("linear", {}))
            message.angular.x, message.angular.y, message.angular.z = self.vector(params.get("angular", {}))
            # Preserve ROS1 behavior: publish the command 100 times immediately.
            for _ in range(100):
                self.cmd_pub.publish(message)
            return self.response(request)
        if command == "SetHolderParam":
            message = SetHolder()
            message.angular_z = float(params.get("angular_z", 0.0))
            message.angular_y = float(params.get("angular_y", 0.0))
            message.zoom = float(params.get("zoom", 0.0))
            self.holder_pub.publish(message)
            return self.response(request)
        if command == "GetHolderParam":
            holder = self.holder
            return self.response(request, data={"pan_current": holder.pan_current, "angular_z_min": holder.pan_min, "angular_z_max": holder.pan_max, "tilt_current": holder.tilt_current, "angular_y_min": holder.tilt_min, "angular_y_max": holder.tilt_max, "zoom_current": holder.zoom_current, "zoom_min": holder.zoom_min, "zoom_max": holder.zoom_max})
        if command == "beging_navigate":
            position = params.get("position", {})
            point = PointStamped()
            point.header.stamp = self.get_clock().now().to_msg()
            point.header.frame_id = "map"
            point.point.x, point.point.y, point.point.z = self.vector(position)
            self.goal_pub.publish(point)
            if not self.navigate_client.server_is_ready():
                return self.response(request, code=503, message="Nav2 navigate_to_pose action is unavailable")
            goal = NavigateToPose.Goal()
            goal.pose = PoseStamped()
            goal.pose.header = point.header
            goal.pose.pose.position = point.point
            goal.pose.pose.orientation.w = 1.0
            future = self.navigate_client.send_goal_async(goal)
            future.add_done_callback(self.navigation_goal_response)
            return self.response(request)
        if command == "end_navagate":
            if self.navigation_goal_handle is None:
                return self.response(request, code=404, message="no control-originated Nav2 goal is active")
            self.navigation_goal_handle.cancel_goal_async()
            self.navigation_goal_handle = None
            return self.response(request, message="Nav2 goal cancel requested")
        if command in ("GetParam", "SetParam", "SetDefaultParam"):
            return self.response(request, code=501, message="ROS1 move_base/DWA configuration has no safe direct Nav2 mapping")
        if command == "RebootDevice":
            return self.response(request, message="reboot requested")
        if command == "websocketclose":
            return self.response(request, message="WebSocket will close")
        return self.response(request, code=404, message=f"unknown command: {command}")

    def navigation_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(f"Nav2 goal request failed: {error}")
            return
        if not goal_handle.accepted:
            self.get_logger().warning("Nav2 rejected the control navigation goal")
            return
        self.navigation_goal_handle = goal_handle

    def websocket_signal_callback(self, message):
        if not message.signal or not self.get_parameter("websocket_enabled").value:
            return
        with self.ws_lock:
            if self.ws_thread is None or not self.ws_thread.is_alive():
                self.ws_thread = threading.Thread(target=self.websocket_worker, daemon=True)
                self.ws_thread.start()

    def websocket_worker(self):
        asyncio.run(self.websocket_loop())

    async def websocket_loop(self):
        import websockets
        address = self.get_parameter("device.server_ip_port").value
        token = self.get_parameter("device.register_number").value
        uri = f"ws://{address}/infra/ws?token={token}"
        try:
            async with websockets.connect(uri) as websocket:
                self.get_logger().info(f"WebSocket connected: {uri}")
                async def heartbeat():
                    while True:
                        await asyncio.sleep(self.get_parameter("websocket_heartbeat_sec").value)
                        await websocket.send(json.dumps({"type": "robot-message-send", "command": "Ping"}))
                task = asyncio.create_task(heartbeat())
                try:
                    async for raw in websocket:
                        response = self.handle_command(raw)
                        await websocket.send(json.dumps(response, ensure_ascii=False))
                        if response.get("command") == "RebootDevice":
                            # Match ROS1 ordering: acknowledge first, then reboot the host.
                            await asyncio.sleep(0.2)
                            process = subprocess.Popen(["sudo", "-S", "shutdown", "-r", "now"], stdin=subprocess.PIPE)
                            process.communicate(input=b"yzz001\n", timeout=3)
                            break
                        if response.get("command") == "websocketclose":
                            await websocket.close()
                            break
                finally:
                    task.cancel()
        except Exception as error:
            self.get_logger().warning(f"WebSocket connection failed: {error}")


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
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
