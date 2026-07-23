"""ROS 2 bridge for the local YZZ browser mapping console."""

import base64
import hashlib
import hmac
import json
import math
import os
import re
import secrets
import signal
import subprocess
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


MAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
SAFE_MAP_NAME = re.compile(r'^[A-Za-z0-9_-]{1,64}$')


class WebMappingNode(Node):
    """Serve a LAN-only web page and bridge its allowed actions to ROS."""

    def __init__(self):
        super().__init__('web_mapping')
        self.declare_parameter('port', 8080)
        self.declare_parameter('allow_teleop', True)
        self.declare_parameter('maps_directory', '')
        self.declare_parameter('waypoints_directory', '')
        self.declare_parameter(
            'auth_file', str(Path.home() / '.config' / 'yzz_web_mapping' / 'auth.json')
        )
        self._port = int(self.get_parameter('port').value)
        self._allow_teleop = bool(self.get_parameter('allow_teleop').value)
        self._auth_file = Path(self.get_parameter('auth_file').value).expanduser()
        self._auth = self._load_auth_config()
        self._sessions = {}
        configured_maps_dir = self.get_parameter('maps_directory').value
        # Maps are user data, not package installation data. Keep them in the
        # same source-workspace directory used by the existing project tools.
        self._maps_dir = Path(configured_maps_dir) if configured_maps_dir else (
            Path.home() / 'robot_ws' / 'src' / 'FuncPackage' / 'yzz_navigation2' / 'maps'
        )
        configured_waypoints_dir = self.get_parameter('waypoints_directory').value
        self._waypoints_dir = Path(configured_waypoints_dir) if configured_waypoints_dir else (
            Path.home() / 'robot_ws' / 'data' / 'waypoints'
        )

        self._lock = threading.Lock()
        self._map = None
        self._pose = None
        self._pose_source = 'none'
        self._mode = 'idle'
        self._process = None
        self._process_kind = None
        self._last_error = ''
        self._waypoint_status = {'state': 'offline', 'message': 'Waypoint navigator has not reported yet'}
        self._web_dir = Path(get_package_share_directory('yzz_web_mapping')) / 'web'

        self.create_subscription(OccupancyGrid, '/map', self._on_map, MAP_QOS)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )
        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._navigation_goal_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._waypoint_command_pub = self.create_publisher(
            String, '/yzz_waypoint_nav/command', 10
        )
        self.create_subscription(
            String, '/yzz_waypoint_nav/status', self._on_waypoint_status, 10
        )
        self._last_teleop_motion = 0.0
        self.create_timer(0.10, self._teleop_watchdog)
        self.create_timer(0.25, self._check_process)

        self._http_server = ThreadingHTTPServer(('0.0.0.0', self._port), self._handler())
        self._http_thread = threading.Thread(
            target=self._http_server.serve_forever, name='web-mapping-http', daemon=True
        )
        self._http_thread.start()
        self.get_logger().info(
            f'Web mapping console is listening on http://0.0.0.0:{self._port} '
            f'(teleop enabled: {self._allow_teleop})'
        )

    def _load_auth_config(self):
        """Load a PBKDF2 password hash from the private runtime config."""
        try:
            config = json.loads(self._auth_file.read_text(encoding='utf-8'))
            if not all(key in config for key in ('username', 'salt', 'password_hash')):
                raise ValueError('missing username, salt or password_hash')
            return config
        except (OSError, ValueError, json.JSONDecodeError) as error:
            raise RuntimeError(
                f'Cannot load web login configuration {self._auth_file}: {error}. '
                'Run: ros2 run yzz_web_mapping configure_auth'
            ) from error

    def _verify_login(self, username, password):
        if not hmac.compare_digest(str(username), str(self._auth['username'])):
            return False
        salt = base64.b64decode(self._auth['salt'])
        expected = base64.b64decode(self._auth['password_hash'])
        actual = hashlib.pbkdf2_hmac(
            'sha256', str(password).encode('utf-8'), salt, 200000
        )
        return hmac.compare_digest(actual, expected)

    def _create_session(self):
        token = secrets.token_urlsafe(32)
        with self._lock:
            self._sessions[token] = time.monotonic() + 8 * 60 * 60
        return token

    def _authenticated(self, cookie_header):
        token = ''
        for item in (cookie_header or '').split(';'):
            key, _, value = item.strip().partition('=')
            if key == 'yzz_session':
                token = value
                break
        if not token:
            return False
        with self._lock:
            expires_at = self._sessions.get(token, 0.0)
            if expires_at <= time.monotonic():
                self._sessions.pop(token, None)
                return False
            return True

    def _remove_session(self, cookie_header):
        for item in (cookie_header or '').split(';'):
            key, _, value = item.strip().partition('=')
            if key == 'yzz_session':
                with self._lock:
                    self._sessions.pop(value, None)
                return

    def _on_map(self, message):
        with self._lock:
            self._map = message

    def _set_pose(self, pose, source):
        with self._lock:
            self._pose = pose
            self._pose_source = source

    def _on_amcl_pose(self, message):
        self._set_pose(message.pose.pose, 'amcl')

    def _on_odom(self, message):
        with self._lock:
            if self._pose_source != 'amcl':
                self._pose = message.pose.pose
                self._pose_source = 'odom'

    def _check_process(self):
        with self._lock:
            process = self._process
            kind = self._process_kind
        if process is None or process.poll() is None:
            return
        with self._lock:
            if self._process is process:
                self._last_error = f'{kind} launch exited with code {process.returncode}'
                self._process = None
                self._process_kind = None
                self._mode = 'idle'
        self.get_logger().warning(self._last_error)

    def _teleop_watchdog(self):
        with self._lock:
            active = self._last_teleop_motion
            if not active or time.monotonic() - active <= 0.4:
                return
            self._last_teleop_motion = 0.0
        self._publish_stop()
        self.get_logger().warning('Web teleop watchdog stopped the robot after command timeout')

    def _on_waypoint_status(self, message):
        try:
            status = json.loads(message.data)
            if not isinstance(status, dict):
                raise ValueError('not an object')
        except (ValueError, TypeError, json.JSONDecodeError):
            self.get_logger().warning('Ignoring malformed waypoint status message')
            return
        with self._lock:
            self._waypoint_status = status

    def _route_directory(self, map_name):
        if not SAFE_MAP_NAME.fullmatch(map_name):
            raise ValueError('Invalid map name')
        return self._waypoints_dir / map_name

    def _route_file(self, map_name, route_name):
        if not SAFE_MAP_NAME.fullmatch(route_name):
            raise ValueError('Route name may contain only letters, digits, _ and -')
        return self._route_directory(map_name) / f'{route_name}.json'

    def _routes_payload(self, map_name):
        directory = self._route_directory(map_name)
        routes = []
        if directory.is_dir():
            for route_file in sorted(directory.glob('*.json'), key=lambda item: item.stat().st_mtime, reverse=True):
                try:
                    payload = json.loads(route_file.read_text(encoding='utf-8'))
                    count = len(payload.get('waypoints', []))
                except (OSError, ValueError, TypeError):
                    count = 0
                routes.append({'name': route_file.stem, 'waypoint_count': count,
                               'modified': int(route_file.stat().st_mtime)})
        return {'map_name': map_name, 'routes': routes}

    def _route_payload(self, map_name, route_name):
        route_file = self._route_file(map_name, route_name)
        try:
            payload = json.loads(route_file.read_text(encoding='utf-8'))
        except (OSError, ValueError, json.JSONDecodeError) as error:
            raise ValueError(f'Unable to read route: {error}') from error
        if not isinstance(payload.get('waypoints'), list):
            raise ValueError('Route has no waypoint list')
        return payload

    def _save_route(self, body):
        map_name = str(body.get('map_name', ''))
        route_name = str(body.get('route_name', ''))
        if not SAFE_MAP_NAME.fullmatch(map_name):
            raise ValueError('Invalid map name')
        if not (self._maps_dir / f'{map_name}.yaml').is_file():
            raise ValueError('Save the map before saving a route for it')
        route_file = self._route_file(map_name, route_name)
        waypoints = body.get('waypoints')
        if not isinstance(waypoints, list) or not 1 <= len(waypoints) <= 64:
            raise ValueError('A route must contain 1 to 64 waypoints')
        normalized = []
        for index, waypoint in enumerate(waypoints, start=1):
            try:
                x = float(waypoint['x'])
                y = float(waypoint['y'])
                yaw = float(waypoint.get('yaw', 0.0))
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError(f'Waypoint {index} is invalid') from error
            if not all(math.isfinite(value) for value in (x, y, yaw)):
                raise ValueError(f'Waypoint {index} contains an invalid number')
            normalized.append({
                'name': str(waypoint.get('name', f'航点 {index}')).strip()[:48] or f'航点 {index}',
                'x': x, 'y': y, 'yaw': yaw,
            })
        route_file.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            'schema_version': 1,
            'map_name': map_name,
            'route_name': route_name,
            'updated_at': int(time.time()),
            'waypoints': normalized,
        }
        temporary = route_file.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
        temporary.replace(route_file)
        return f'Saved route {route_name} with {len(normalized)} waypoint(s)'

    def _delete_route(self, body):
        route_file = self._route_file(str(body.get('map_name', '')), str(body.get('route_name', '')))
        if not route_file.is_file():
            raise ValueError('Route does not exist')
        route_file.unlink()
        return f'Deleted route {route_file.stem}'

    def _waypoint_command(self, body):
        action = str(body.get('action', ''))
        if action not in {'load', 'start', 'pause', 'resume', 'stop'}:
            raise ValueError('Invalid waypoint action')
        command = {'action': action}
        if action in {'load', 'start'}:
            map_name = str(body.get('map_name', ''))
            route_name = str(body.get('route_name', ''))
            self._route_file(map_name, route_name)
            if not (self._maps_dir / f'{map_name}.yaml').is_file():
                raise ValueError('Selected map does not exist')
            command.update({'map_name': map_name, 'route_name': route_name})
        self._waypoint_command_pub.publish(String(data=json.dumps(command, ensure_ascii=False)))
        return f'Waypoint command sent: {action}'

    def _maps_payload(self):
        self._maps_dir.mkdir(parents=True, exist_ok=True)
        maps = []
        for file_path in sorted(self._maps_dir.glob('*.yaml'), key=lambda item: item.stat().st_mtime, reverse=True):
            maps.append({
                'name': file_path.stem,
                'modified': int(file_path.stat().st_mtime),
            })
        return {'maps': maps}

    def _map_payload(self):
        with self._lock:
            map_message = self._map
        if map_message is None:
            return {'available': False}
        origin = map_message.info.origin
        return {
            'available': True,
            'stamp': {
                'sec': map_message.header.stamp.sec,
                'nanosec': map_message.header.stamp.nanosec,
            },
            'frame_id': map_message.header.frame_id or 'map',
            'width': map_message.info.width,
            'height': map_message.info.height,
            'resolution': map_message.info.resolution,
            'origin': {
                'x': origin.position.x,
                'y': origin.position.y,
                'yaw': self._yaw_from_quaternion(origin.orientation),
            },
            'data': list(map_message.data),
        }

    def _status_payload(self):
        with self._lock:
            pose = self._pose
            process_running = self._process is not None and self._process.poll() is None
            waypoint_status = dict(self._waypoint_status)
            return {
                'mode': self._mode,
                'process_running': process_running,
                'process_kind': self._process_kind,
                'map_available': self._map is not None,
                'pose_source': self._pose_source,
                'pose': None if pose is None else {
                    'x': round(pose.position.x, 3),
                    'y': round(pose.position.y, 3),
                    'yaw': round(self._yaw_from_quaternion(pose.orientation), 3),
                },
                'teleop_enabled': self._allow_teleop,
                'last_error': self._last_error,
                'waypoint': waypoint_status,
            }

    @staticmethod
    def _yaw_from_quaternion(quaternion):
        sin_yaw = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cos_yaw = 1.0 - 2.0 * (quaternion.y ** 2 + quaternion.z ** 2)
        return math.atan2(sin_yaw, cos_yaw)

    @staticmethod
    def _quaternion_from_yaw(yaw):
        return math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    def _start(self, kind, command):
        with self._lock:
            if self._process is not None and self._process.poll() is None:
                raise ValueError(f'{self._process_kind} is already running')
            self._last_error = ''
            self._process = subprocess.Popen(
                command,
                start_new_session=True,
                stdout=None,
                stderr=None,
            )
            self._process_kind = kind
            self._mode = kind
        self.get_logger().info(f'Started {kind}: {" ".join(command)}')

    def _stop(self):
        with self._lock:
            process = self._process
        if process is None or process.poll() is not None:
            raise ValueError('No web-started mapping or navigation process is running')
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
        with self._lock:
            self._process = None
            self._process_kind = None
            self._mode = 'idle'
        self._publish_stop()

    def _start_mapping(self):
        self._start('mapping', ['ros2', 'launch', 'yzz_robotlaunch', 'mapping_system.launch.py'])

    def _start_navigation(self, map_name):
        if not SAFE_MAP_NAME.fullmatch(map_name):
            raise ValueError('Invalid map name')
        map_file = self._maps_dir / f'{map_name}.yaml'
        if not map_file.is_file():
            raise ValueError(f'Map does not exist: {map_name}')
        self._start(
            'navigation',
            ['ros2', 'launch', 'yzz_robotlaunch', 'navigation_system.launch.py', f'map:={map_file}'],
        )

    def _save_map(self, map_name):
        if not SAFE_MAP_NAME.fullmatch(map_name):
            raise ValueError('Map name may contain only letters, digits, _ and -')
        self._maps_dir.mkdir(parents=True, exist_ok=True)
        target = self._maps_dir / map_name
        result = subprocess.run(
            ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(target),
             '--ros-args', '-p', 'save_map_timeout:=20.0'],
            timeout=30.0,
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode != 0:
            raise ValueError((result.stderr or result.stdout or 'map save failed').strip())
        return f'Saved {map_name}.yaml and {map_name}.pgm'

    def _publish_initial_pose(self, body):
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = float(body['x'])
        message.pose.pose.position.y = float(body['y'])
        z, w = self._quaternion_from_yaw(float(body.get('yaw', 0.0)))
        message.pose.pose.orientation.z = z
        message.pose.pose.orientation.w = w
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.0685
        self._initial_pose_pub.publish(message)

    def _publish_goal(self, body):
        message = PoseStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = float(body['x'])
        message.pose.position.y = float(body['y'])
        z, w = self._quaternion_from_yaw(float(body.get('yaw', 0.0)))
        message.pose.orientation.z = z
        message.pose.orientation.w = w
        self._goal_pub.publish(message)

    def _publish_stop(self):
        self._cmd_vel_pub.publish(Twist())

    def _send_navigation_goal(self, body):
        if not self._navigation_goal_client.wait_for_server(timeout_sec=1.0):
            raise ValueError('Nav2 NavigateToPose action is not ready; set the initial pose first')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(body['x'])
        goal.pose.pose.position.y = float(body['y'])
        z, w = self._quaternion_from_yaw(float(body.get('yaw', 0.0)))
        goal.pose.pose.orientation.z = z
        goal.pose.pose.orientation.w = w
        future = self._navigation_goal_client.send_goal_async(goal)
        future.add_done_callback(self._on_navigation_goal_response)

    def _on_navigation_goal_response(self, future):
        try:
            response = future.result()
            if response.accepted:
                self.get_logger().info('Web Navigation Goal accepted by Nav2')
            else:
                self.get_logger().warning('Web Navigation Goal was rejected by Nav2')
        except Exception as error:  # pylint: disable=broad-except
            self.get_logger().error(f'Navigation Goal request failed: {error}')

    def _teleop(self, body):
        if not self._allow_teleop:
            raise ValueError('Web teleoperation is disabled on this server')
        if self._mode == 'navigation':
            raise ValueError('Web teleoperation is unavailable while Nav2 navigation is running')
        linear = float(body.get('linear', 0.0))
        angular = float(body.get('angular', 0.0))
        if abs(linear) > 0.35 or abs(angular) > 1.0:
            raise ValueError('Teleoperation speed exceeds the web safety limit')
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self._cmd_vel_pub.publish(message)
        with self._lock:
            self._last_teleop_motion = time.monotonic() if (linear or angular) else 0.0

    def _handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                node.get_logger().debug('HTTP ' + (fmt % args))

            def _json(self, status, payload, headers=None):
                data = json.dumps(payload, ensure_ascii=False).encode('utf-8')
                self.send_response(status)
                self.send_header('Content-Type', 'application/json; charset=utf-8')
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                for key, value in (headers or {}).items():
                    self.send_header(key, value)
                self.end_headers()
                self.wfile.write(data)

            def _redirect(self, location):
                self.send_response(HTTPStatus.SEE_OTHER)
                self.send_header('Location', location)
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()

            def _require_auth(self):
                if node._authenticated(self.headers.get('Cookie')):
                    return True
                self._json(HTTPStatus.UNAUTHORIZED, {'ok': False, 'error': 'Login required'})
                return False

            def _body(self):
                size = int(self.headers.get('Content-Length', '0'))
                if size > 16384:
                    raise ValueError('Request body is too large')
                return json.loads(self.rfile.read(size) or b'{}')

            def do_GET(self):
                parsed = urlparse(self.path)
                path = parsed.path
                query = parse_qs(parsed.query)
                if path == '/login':
                    if node._authenticated(self.headers.get('Cookie')):
                        self._redirect('/')
                    else:
                        self._static('login.html', 'text/html; charset=utf-8')
                    return
                if path in ('/login.js', '/style.css'):
                    file_name = path.lstrip('/')
                    content_type = 'text/css; charset=utf-8' if path.endswith('.css') else 'text/javascript; charset=utf-8'
                    self._static(file_name, content_type)
                    return
                if path in ('/', '/index.html'):
                    if not node._authenticated(self.headers.get('Cookie')):
                        self._redirect('/login')
                    else:
                        self._static('index.html', 'text/html; charset=utf-8')
                    return
                if not self._require_auth():
                    return
                if path == '/api/status':
                    self._json(HTTPStatus.OK, node._status_payload())
                elif path == '/api/map':
                    self._json(HTTPStatus.OK, node._map_payload())
                elif path == '/api/maps':
                    self._json(HTTPStatus.OK, node._maps_payload())
                elif path == '/api/routes':
                    self._json(HTTPStatus.OK, node._routes_payload(query.get('map_name', [''])[0]))
                elif path == '/api/route':
                    self._json(HTTPStatus.OK, node._route_payload(
                        query.get('map_name', [''])[0], query.get('route_name', [''])[0]
                    ))
                elif path == '/app.js':
                    self._static('app.js', 'text/javascript; charset=utf-8')
                else:
                    self._json(HTTPStatus.NOT_FOUND, {'error': 'Not found'})

            def _static(self, filename, content_type):
                data = (node._web_dir / filename).read_bytes()
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(data)

            def do_POST(self):
                try:
                    path = urlparse(self.path).path
                    body = self._body()
                    if path == '/api/auth/login':
                        if not node._verify_login(body.get('username', ''), body.get('password', '')):
                            self._json(HTTPStatus.UNAUTHORIZED, {'ok': False, 'error': 'Invalid username or password'})
                            return
                        token = node._create_session()
                        self._json(HTTPStatus.OK, {'ok': True}, {
                            'Set-Cookie': f'yzz_session={token}; HttpOnly; SameSite=Strict; Path=/; Max-Age=28800'
                        })
                        return
                    if path == '/api/auth/logout':
                        node._remove_session(self.headers.get('Cookie'))
                        self._json(HTTPStatus.OK, {'ok': True}, {
                            'Set-Cookie': 'yzz_session=; HttpOnly; SameSite=Strict; Path=/; Max-Age=0'
                        })
                        return
                    if not self._require_auth():
                        return
                    if path == '/api/mapping/start':
                        node._start_mapping()
                        message = 'Mapping launch started'
                    elif path == '/api/navigation/start':
                        node._start_navigation(str(body.get('map_name', '')))
                        message = 'Navigation launch started; set initial pose before sending a goal'
                    elif path == '/api/system/stop':
                        node._stop()
                        message = 'Web-started system stopped'
                    elif path == '/api/map/save':
                        message = node._save_map(str(body.get('map_name', '')))
                    elif path == '/api/routes/save':
                        message = node._save_route(body)
                    elif path == '/api/routes/delete':
                        message = node._delete_route(body)
                    elif path == '/api/waypoint/command':
                        message = node._waypoint_command(body)
                    elif path == '/api/initialpose':
                        node._publish_initial_pose(body)
                        message = 'Initial pose published'
                    elif path == '/api/goal':
                        node._publish_goal(body)
                        message = '2D Goal Pose published'
                    elif path == '/api/navigation_goal':
                        node._send_navigation_goal(body)
                        message = 'Navigation Goal sent to Nav2'
                    elif path == '/api/teleop':
                        node._teleop(body)
                        message = 'Velocity command published'
                    else:
                        self._json(HTTPStatus.NOT_FOUND, {'error': 'Not found'})
                        return
                    self._json(HTTPStatus.OK, {'ok': True, 'message': message})
                except (ValueError, KeyError, json.JSONDecodeError) as error:
                    self._json(HTTPStatus.BAD_REQUEST, {'ok': False, 'error': str(error)})
                except Exception as error:  # pylint: disable=broad-except
                    node.get_logger().error(f'Web request failed: {error}')
                    self._json(HTTPStatus.INTERNAL_SERVER_ERROR, {'ok': False, 'error': str(error)})

        return Handler

    def destroy_node(self):
        self._publish_stop()
        self._http_server.shutdown()
        self._http_server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebMappingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
