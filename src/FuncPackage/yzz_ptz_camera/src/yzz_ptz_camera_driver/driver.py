"""V4L2 PTZ camera driver with ROS 1-compatible topics."""

import re
import subprocess
import threading

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from yzz_msgs.msg import GetHolder, SetHolder


DEFAULT_RANGES = {
    'pan_absolute': (-594000, 594000),
    'tilt_absolute': (-97200, 324000),
    'zoom_absolute': (0, 16384),
}


class PtzCameraNode(Node):
    def __init__(self):
        super().__init__('yzz_ptz_camera')
        self.declare_parameter('device', '/dev/video1')
        self.declare_parameter('pan_step', 18000)
        self.declare_parameter('tilt_step', 7200)
        self.declare_parameter('zoom_step', 100)
        # Web remote-control values are angles in degrees.  The UVC PTZ
        # controls use device-specific raw units, so map them to the actual
        # V4L2 range before writing pan/tilt values.
        self.declare_parameter('pan_command_degrees', 165.0)
        self.declare_parameter('tilt_command_degrees', 90.0)
        # The web joystick is normalized to roughly +/-220 rather than using
        # the raw V4L2 range.  Keep movements gradual and calibratable.
        self.declare_parameter('web_command_deadband', 2.0)
        # Raw V4L2 units added for each received web joystick command.
        # At the observed ~20 Hz callback rate, 8.0 gives a gentle response.
        self.declare_parameter('web_pan_increment', 8.0)
        self.declare_parameter('web_tilt_increment', 8.0)
        self.declare_parameter('pan_direction', 1.0)
        self.declare_parameter('tilt_direction', 1.0)
        self.declare_parameter('image_width', 320)
        self.declare_parameter('image_height', 240)
        self.declare_parameter('image_fps', 10.0)

        self.device = self.get_parameter('device').value
        self.pan_step = self.get_parameter('pan_step').value
        self.tilt_step = self.get_parameter('tilt_step').value
        self.zoom_step = self.get_parameter('zoom_step').value
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.capture = None
        self.last_key = None
        self.ranges = {
            name: self.get_ctrl_range(name) for name in DEFAULT_RANGES
        }
        self.pan = self.get_ctrl_value('pan_absolute') or 0
        self.tilt = self.get_ctrl_value('tilt_absolute') or 0
        self.zoom = self.get_ctrl_value('zoom_absolute') or 0

        self.image_pub = self.create_publisher(Image, 'usb_cam/image_raw', 1)
        self.holder_pub = self.create_publisher(GetHolder, '/GetHolder', 10)
        self.create_subscription(String, '/key_pressed', self.key_callback, 10)
        self.create_subscription(SetHolder, '/SetHolder', self.set_holder_callback, 10)

        fps = max(float(self.get_parameter('image_fps').value), 1.0)
        self.create_timer(1.0 / fps, self.capture_and_publish)
        self.create_timer(1.0, self.publish_ptz_status)
        self.create_timer(2.0, self.try_open_camera)
        self.try_open_camera()

    def run_v4l2(self, *arguments):
        return subprocess.check_output(
            ['v4l2-ctl', '-d', self.device, *arguments], text=True,
            stderr=subprocess.STDOUT)

    def get_ctrl_range(self, name):
        try:
            for line in self.run_v4l2('--list-ctrls').splitlines():
                if name not in line:
                    continue
                match = re.search(r'min=(-?\d+).*max=(-?\d+)', line)
                if match:
                    return int(match.group(1)), int(match.group(2))
        except (FileNotFoundError, subprocess.CalledProcessError) as error:
            self.get_logger().debug(f'Cannot query {name}: {error}')
        return DEFAULT_RANGES[name]

    def get_ctrl_value(self, name):
        try:
            output = self.run_v4l2('--get-ctrl', name)
            match = re.search(r'[-]?\d+', output.rsplit(':', 1)[-1])
            return int(match.group(0)) if match else None
        except (FileNotFoundError, subprocess.CalledProcessError):
            return None

    def clamp(self, name, value):
        minimum, maximum = self.ranges[name]
        return max(minimum, min(int(value), maximum))

    def set_ctrl_value(self, name, value):
        value = self.clamp(name, value)
        try:
            with self.lock:
                self.run_v4l2('-c', f'{name}={value}')
            return value
        except (FileNotFoundError, subprocess.CalledProcessError) as error:
            self.get_logger().error(f'Failed to set {name}: {error}')
            return None

    def publish_ptz_status(self):
        message = GetHolder()
        message.pan_min, message.pan_max = map(float, self.ranges['pan_absolute'])
        message.pan_current = float(self.pan)
        message.tilt_min, message.tilt_max = map(float, self.ranges['tilt_absolute'])
        message.tilt_current = float(self.tilt)
        message.zoom_min, message.zoom_max = map(float, self.ranges['zoom_absolute'])
        message.zoom_current = float(self.zoom)
        self.holder_pub.publish(message)

    def apply_web_increment(self, name, command, increment, direction):
        """Move relative to the current position; zero holds the position.

        Browser joystick controls repeatedly publish a zero value on release.
        Treating zero as an absolute target made the PTZ return to its centre.
        """
        deadband = float(self.get_parameter('web_command_deadband').value)
        if abs(float(command)) <= deadband:
            return None
        current = self.pan if name == 'pan_absolute' else self.tilt
        target = current + int(float(command) * increment * direction)
        return self.set_ctrl_value(name, target)

    def set_holder_callback(self, message):
        # Platform joystick calibration: web horizontal (angular_y) controls
        # pan, web vertical (angular_z) controls tilt.  The values are relative
        # joystick inputs, so release (zero) deliberately holds the PTZ still.
        pan = self.apply_web_increment(
            'pan_absolute', message.angular_y,
            float(self.get_parameter('web_pan_increment').value),
            float(self.get_parameter('pan_direction').value))
        tilt = self.apply_web_increment(
            'tilt_absolute', message.angular_z,
            float(self.get_parameter('web_tilt_increment').value),
            float(self.get_parameter('tilt_direction').value))
        zoom = self.set_ctrl_value('zoom_absolute', message.zoom * 10)
        if pan is not None:
            self.pan = pan
        if tilt is not None:
            self.tilt = tilt
        if zoom is not None:
            self.zoom = zoom
        self.publish_ptz_status()

    def key_callback(self, message):
        self.last_key = message.data.strip()

    def process_key(self):
        key = self.last_key
        self.last_key = None
        if key == 'I':
            self.tilt = self.set_ctrl_value('tilt_absolute', self.tilt + self.tilt_step) or self.tilt
        elif key == '<':
            self.tilt = self.set_ctrl_value('tilt_absolute', self.tilt - self.tilt_step) or self.tilt
        elif key == 'J':
            self.pan = self.set_ctrl_value('pan_absolute', self.pan + self.pan_step) or self.pan
        elif key == 'L':
            self.pan = self.set_ctrl_value('pan_absolute', self.pan - self.pan_step) or self.pan
        elif key == '+':
            self.zoom = self.set_ctrl_value('zoom_absolute', self.zoom + self.zoom_step) or self.zoom
        elif key == '_':
            self.zoom = self.set_ctrl_value('zoom_absolute', self.zoom - self.zoom_step) or self.zoom
        self.publish_ptz_status()

    def try_open_camera(self):
        if self.capture is not None and self.capture.isOpened():
            return
        capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.get_parameter('image_width').value)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.get_parameter('image_height').value)
        capture.set(cv2.CAP_PROP_FPS, self.get_parameter('image_fps').value)
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        if capture.isOpened():
            self.capture = capture
            self.get_logger().info(f'Opened PTZ camera {self.device}')
        else:
            capture.release()
            self.get_logger().warning(f'Waiting for PTZ camera {self.device}')

    def capture_and_publish(self):
        if self.last_key:
            self.process_key()
        if self.capture is None or not self.capture.isOpened():
            return
        with self.lock:
            for _ in range(4):
                self.capture.grab()
            success, frame = self.capture.read()
        if success:
            image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            image.header.stamp = self.get_clock().now().to_msg()
            self.image_pub.publish(image)

    def destroy_node(self):
        if self.capture is not None:
            self.capture.release()
        return super().destroy_node()


def main():
    rclpy.init()
    node = PtzCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
