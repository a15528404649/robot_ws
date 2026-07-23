import math
import threading

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String
from yzz_gps.msg import Gps
import serial


class GPSDriverNode(Node):
    """Read NMEA GPS sentences and publish the project-compatible ROS topics."""

    def __init__(self):
        super().__init__("gps_driver")
        self._declare_parameters()
        self._load_parameters()
        self._setup_publishers()
        self._start_reader_thread()

    def _declare_parameters(self):
        for name, value in (("port", "/dev/gps_usb"), ("baud", 115200), ("frame_id", "gps_link"), ("fix_topic", "/gps/fix"), ("velocity_topic", "/gps/vel"), ("nmea_topic", "/gps/nmea"), ("compatibility_topic", "/gps")):
            self.declare_parameter(name, value)

    def _load_parameters(self):
        self.port = self.get_parameter("port").value
        self.baud = self.get_parameter("baud").value
        self.frame_id = self.get_parameter("frame_id").value

    def _setup_publishers(self):
        self.fix_pub = self.create_publisher(NavSatFix, self.get_parameter("fix_topic").value, 10)
        self.velocity_pub = self.create_publisher(TwistStamped, self.get_parameter("velocity_topic").value, 10)
        self.nmea_pub = self.create_publisher(String, self.get_parameter("nmea_topic").value, 20)
        self.compatibility_pub = self.create_publisher(Gps, self.get_parameter("compatibility_topic").value, 10)

    def _start_reader_thread(self):
        self.serial_port = None
        self.running = True
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

    @staticmethod
    def valid_checksum(sentence):
        if not sentence.startswith("$") or "*" not in sentence:
            return False
        payload, checksum = sentence[1:].split("*", 1)
        try:
            value = 0
            for char in payload:
                value ^= ord(char)
            return len(checksum) >= 2 and value == int(checksum[:2], 16)
        except ValueError:
            return False

    @staticmethod
    def coordinate(value, hemisphere):
        digits = 2 if hemisphere in ("N", "S") else 3
        result = float(value[:digits]) + float(value[digits:]) / 60.0
        return -result if hemisphere in ("S", "W") else result

    def publish_fix(self, fields):
        if len(fields) < 10:
            return
        try:
            quality = int(fields[6] or 0)
        except (ValueError, IndexError):
            return
        latitude = longitude = altitude = math.nan
        if quality > 0:
            try:
                latitude = self.coordinate(fields[2], fields[3])
                longitude = self.coordinate(fields[4], fields[5])
                altitude = float(fields[9]) if fields[9] else math.nan
            except (ValueError, IndexError):
                quality = 0
        message = NavSatFix()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.status.service = NavSatStatus.SERVICE_GPS
        message.status.status = NavSatStatus.STATUS_FIX if quality > 0 else NavSatStatus.STATUS_NO_FIX
        message.latitude, message.longitude, message.altitude = latitude, longitude, altitude
        message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.fix_pub.publish(message)

    def publish_velocity(self, fields):
        if len(fields) < 9 or fields[2] != "A":
            return
        try:
            speed = float(fields[7] or 0.0) * 0.514444
        except ValueError:
            return
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.twist.linear.x = speed
        self.velocity_pub.publish(message)

    def publish_compatibility_gps(self, fields):
        if len(fields) < 7:
            return
        try:
            message = Gps()
            message.n = self.coordinate(fields[3], fields[4])
            message.e = self.coordinate(fields[5], fields[6])
        except (ValueError, IndexError):
            return
        self.compatibility_pub.publish(message)

    def process_sentence(self, sentence):
        if not self.valid_checksum(sentence):
            return
        self.nmea_pub.publish(String(data=sentence))
        fields = sentence[1:].split("*", 1)[0].split(",")
        if fields[0][-3:] == "GGA":
            self.publish_fix(fields)
        elif fields[0][-3:] == "RMC":
            self.publish_velocity(fields)
            self.publish_compatibility_gps(fields)

    def read_loop(self):
        try:
            self.serial_port = serial.Serial(self.port, self.baud, timeout=0.5)
        except serial.SerialException as error:
            self.get_logger().error(f"无法打开 GPS 串口 {self.port}: {error}")
            return
        self.get_logger().info(f"GPS 串口已打开：{self.port}，波特率：{self.baud}")
        while self.running and rclpy.ok():
            try:
                raw = self.serial_port.readline()
            except serial.SerialException as error:
                self.get_logger().error(f"GPS 串口读取失败：{error}")
                break
            if raw:
                sentence = raw.decode("ascii", errors="ignore").strip()
                if sentence:
                    self.process_sentence(sentence)
        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()

    def destroy_node(self):
        self.running = False
        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GPSDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
