#!/usr/bin/env python3
"""Execute saved map-associated waypoint routes through Nav2.

The node deliberately owns patrol sequencing only.  Nav2 continues to own
planning, obstacle avoidance and velocity output; this node never publishes
``/cmd_vel`` directly.
"""

import json
import math
import random
import re
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


SAFE_NAME = re.compile(r'^[A-Za-z0-9_-]{1,64}$')


class WaypointNavNode(Node):
    """Run one saved route at a time with ROS1-compatible patrol parameters."""

    def __init__(self):
        super().__init__('waypoint_nav_node')
        self._declare_parameters()
        self._initialize_state()
        self._setup_interfaces()
        self._start_timer()
        self.get_logger().info('Waypoint navigator is ready; waiting for a web route command')

    def _declare_parameters(self):
        self.declare_parameter(
            'routes_directory', str(Path.home() / 'robot_ws' / 'data' / 'waypoints')
        )
        self.declare_parameter('command_topic', '/yzz_waypoint_nav/command')
        self.declare_parameter('status_topic', '/yzz_waypoint_nav/status')
        self.declare_parameter('rest_time', 3.0)
        self.declare_parameter('keep_patrol', False)
        self.declare_parameter('random_patrol', False)
        self.declare_parameter('patrol_type', 0)
        self.declare_parameter('patrol_loop', 1)
        self.declare_parameter('patrol_time', 5.0)
        # Keep the original misspelling for ROS1 parameter compatibility.
        # A zero value means use every point in the selected saved route.
        self.declare_parameter('potrol_points_num', 0)
        self.declare_parameter('goal_timeout', 300.0)

    def _initialize_state(self):
        self._routes_dir = Path(self.get_parameter('routes_directory').value).expanduser()
        self._route = []
        self._map_name = ''
        self._route_name = ''
        self._state = 'idle'
        self._message = 'No route loaded'
        self._step = 0
        self._cycles_completed = 0
        self._started_at = 0.0
        self._rest_until = 0.0
        self._goal_started_at = 0.0
        self._goal_request_pending = False
        self._active_goal_handle = None
        self._active_waypoint_index = None
        self._goal_generation = 0

    def _setup_interfaces(self):
        self._command_sub = self.create_subscription(
            String,
            self.get_parameter('command_topic').value,
            self._command_callback,
            10,
        )
        self._status_pub = self.create_publisher(
            String, self.get_parameter('status_topic').value, 10
        )
        self._marker_pub = self.create_publisher(MarkerArray, '/path_point', 10)
        self._navigation_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _start_timer(self):
        self.create_timer(0.20, self._tick)
        self.create_timer(0.50, self._publish_status)

    @staticmethod
    def _require_name(value, field):
        value = str(value)
        if not SAFE_NAME.fullmatch(value):
            raise ValueError(f'{field} may contain only letters, digits, _ and -')
        return value

    def _route_file(self, map_name, route_name):
        map_name = self._require_name(map_name, 'map_name')
        route_name = self._require_name(route_name, 'route_name')
        return self._routes_dir / map_name / f'{route_name}.json'

    def _load_route(self, map_name, route_name):
        route_file = self._route_file(map_name, route_name)
        try:
            payload = json.loads(route_file.read_text(encoding='utf-8'))
            waypoints = payload['waypoints']
        except (OSError, ValueError, KeyError, TypeError) as error:
            raise ValueError(f'Unable to load saved route: {error}') from error
        if not isinstance(waypoints, list) or not waypoints:
            raise ValueError('Saved route does not contain any waypoints')
        if len(waypoints) > 64:
            raise ValueError('Saved route has too many waypoints')

        checked = []
        for index, waypoint in enumerate(waypoints, start=1):
            try:
                x = float(waypoint['x'])
                y = float(waypoint['y'])
                yaw = float(waypoint.get('yaw', 0.0))
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(f'Waypoint {index} is invalid') from error
            if not all(math.isfinite(value) for value in (x, y, yaw)):
                raise ValueError(f'Waypoint {index} contains a non-finite value')
            checked.append({'name': str(waypoint.get('name', f'航点 {index}'))[:48],
                            'x': x, 'y': y, 'yaw': yaw})

        self._cancel_active_goal()
        self._route = checked
        self._map_name = self._require_name(map_name, 'map_name')
        self._route_name = self._require_name(route_name, 'route_name')
        self._step = 0
        self._cycles_completed = 0
        self._rest_until = 0.0
        self._state = 'ready'
        self._message = f'Loaded {len(checked)} waypoint(s) from {self._route_name}'
        self._publish_markers()

    def _command_callback(self, message):
        try:
            command = json.loads(message.data)
            action = str(command.get('action', ''))
            if action == 'load':
                self._load_route(command.get('map_name', ''), command.get('route_name', ''))
            elif action == 'start':
                if command.get('map_name') or command.get('route_name'):
                    self._load_route(command.get('map_name', ''), command.get('route_name', ''))
                self._start_patrol()
            elif action == 'pause':
                self._pause_patrol()
            elif action == 'resume':
                self._resume_patrol()
            elif action == 'stop':
                self._stop_patrol()
            else:
                raise ValueError('Unsupported waypoint command')
        except (ValueError, TypeError, json.JSONDecodeError) as error:
            self._state = 'error'
            self._message = str(error)
            self.get_logger().warning(f'Waypoint command rejected: {error}')
        self._publish_status()

    def _start_patrol(self):
        if not self._route:
            raise ValueError('Load a saved route before starting patrol')
        if not self._navigation_client.wait_for_server(timeout_sec=1.0):
            raise ValueError('Nav2 NavigateToPose is unavailable; load a map and set initial pose first')
        self._cancel_active_goal()
        self._state = 'running'
        self._message = 'Patrol started'
        self._step = 0
        self._cycles_completed = 0
        self._started_at = time.monotonic()
        self._rest_until = 0.0

    def _pause_patrol(self):
        if self._state != 'running':
            raise ValueError('Patrol is not running')
        self._state = 'paused'
        self._message = 'Patrol paused'
        self._cancel_active_goal()

    def _resume_patrol(self):
        if self._state != 'paused':
            raise ValueError('Patrol is not paused')
        self._state = 'running'
        self._message = 'Patrol resumed'
        self._rest_until = 0.0

    def _stop_patrol(self):
        self._cancel_active_goal()
        self._state = 'stopped'
        self._message = 'Patrol stopped'
        self._step = 0
        self._rest_until = 0.0

    def _selected_count(self):
        limit = int(self.get_parameter('potrol_points_num').value)
        return len(self._route) if limit <= 0 else min(limit, len(self._route))

    def _tick(self):
        if self._state != 'running':
            return
        now = time.monotonic()
        if self._goal_request_pending:
            return
        if self._active_goal_handle is not None:
            timeout = max(1.0, float(self.get_parameter('goal_timeout').value))
            if now - self._goal_started_at > timeout:
                self._message = f'Waypoint timed out after {timeout:.0f} seconds'
                self._cancel_active_goal()
                self._complete_waypoint(False)
            return
        if now < self._rest_until:
            return
        if self._step >= self._selected_count():
            if not self._begin_next_cycle(now):
                return
        self._send_current_goal()

    def _begin_next_cycle(self, now):
        keep_patrol = bool(self.get_parameter('keep_patrol').value)
        patrol_type = int(self.get_parameter('patrol_type').value)
        if keep_patrol:
            self._cycles_completed += 1
            self._step = 0
            return True
        if patrol_type == 1:
            patrol_minutes = max(0.0, float(self.get_parameter('patrol_time').value))
            if now - self._started_at < patrol_minutes * 60.0:
                self._cycles_completed += 1
                self._step = 0
                return True
        else:
            loops = max(1, int(self.get_parameter('patrol_loop').value))
            if self._cycles_completed + 1 < loops:
                self._cycles_completed += 1
                self._step = 0
                return True
        self._state = 'completed'
        self._message = 'Patrol completed'
        return False

    def _current_waypoint_index(self):
        count = self._selected_count()
        if bool(self.get_parameter('random_patrol').value):
            return random.randrange(count)
        return self._step

    def _send_current_goal(self):
        if not self._route:
            self._state = 'error'
            self._message = 'Route disappeared while patrol was running'
            return
        index = self._current_waypoint_index()
        waypoint = self._route[index]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = waypoint['x']
        goal.pose.pose.position.y = waypoint['y']
        goal.pose.pose.orientation.z = math.sin(waypoint['yaw'] / 2.0)
        goal.pose.pose.orientation.w = math.cos(waypoint['yaw'] / 2.0)
        self._goal_request_pending = True
        self._active_waypoint_index = index
        self._goal_generation += 1
        generation = self._goal_generation
        self._goal_started_at = time.monotonic()
        self._message = f"Navigating to {waypoint['name']} ({index + 1}/{self._selected_count()})"
        future = self._navigation_client.send_goal_async(goal)
        future.add_done_callback(
            lambda response, point_index=index, token=generation: self._goal_response(response, point_index, token)
        )

    def _goal_response(self, future, point_index, generation):
        if generation != self._goal_generation:
            try:
                future.result().cancel_goal_async()
            except Exception:  # pylint: disable=broad-except
                pass
            return
        self._goal_request_pending = False
        try:
            goal_handle = future.result()
        except Exception as error:  # pylint: disable=broad-except
            self._message = f'Nav2 goal request failed: {error}'
            self._complete_waypoint(False)
            return
        if not goal_handle.accepted:
            self._message = 'Nav2 rejected the waypoint goal'
            self._complete_waypoint(False)
            return
        if self._state != 'running':
            goal_handle.cancel_goal_async()
            return
        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result, selected_index=point_index, token=generation: self._goal_result(result, selected_index, token)
        )

    def _goal_result(self, future, point_index, generation):
        if generation != self._goal_generation:
            return
        self._active_goal_handle = None
        self._active_waypoint_index = None
        if self._state != 'running':
            return
        try:
            result = future.result()
            succeeded = result.status == GoalStatus.STATUS_SUCCEEDED
            self._message = (
                f'Waypoint {point_index + 1} reached'
                if succeeded else f'Waypoint {point_index + 1} ended with Nav2 status {result.status}'
            )
        except Exception as error:  # pylint: disable=broad-except
            succeeded = False
            self._message = f'Waypoint result failed: {error}'
        self._complete_waypoint(succeeded)

    def _complete_waypoint(self, succeeded):
        if self._state != 'running':
            return
        self._step += 1
        rest = max(0.0, float(self.get_parameter('rest_time').value))
        self._rest_until = time.monotonic() + rest
        if succeeded:
            self.get_logger().info(self._message)
        else:
            self.get_logger().warning(self._message)

    def _cancel_active_goal(self):
        self._goal_generation += 1
        self._goal_request_pending = False
        self._active_waypoint_index = None
        if self._active_goal_handle is not None:
            self._active_goal_handle.cancel_goal_async()
            self._active_goal_handle = None

    def _publish_markers(self):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        for index, waypoint in enumerate(self._route):
            arrow = Marker()
            arrow.header.frame_id = 'map'
            arrow.header.stamp = self.get_clock().now().to_msg()
            arrow.ns = 'yzz_waypoint_nav'
            arrow.id = index * 2
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = waypoint['x']
            arrow.pose.position.y = waypoint['y']
            arrow.pose.orientation.z = math.sin(waypoint['yaw'] / 2.0)
            arrow.pose.orientation.w = math.cos(waypoint['yaw'] / 2.0)
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.42, 0.12, 0.12
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 0.1, 0.75, 1.0, 0.95
            markers.markers.append(arrow)

            label = Marker()
            label.header = arrow.header
            label.ns = arrow.ns
            label.id = index * 2 + 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = Point(x=waypoint['x'], y=waypoint['y'], z=0.25)
            label.pose.orientation.w = 1.0
            label.scale.z = 0.24
            label.color.r, label.color.g, label.color.b, label.color.a = 1.0, 1.0, 1.0, 1.0
            label.text = f'{index + 1}. {waypoint["name"]}'
            markers.markers.append(label)
        self._marker_pub.publish(markers)

    def _publish_status(self):
        current_index = (self._active_waypoint_index + 1) if (
            self._state == 'running' and self._active_waypoint_index is not None
        ) else 0
        payload = {
            'state': self._state,
            'message': self._message,
            'map_name': self._map_name,
            'route_name': self._route_name,
            'waypoint_count': len(self._route),
            'current_waypoint': current_index,
            'cycles_completed': self._cycles_completed,
        }
        self._status_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
