import math
import struct
import threading

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
import serial
from yzz_gps.msg import Gps


def quaternion_from_euler(roll, pitch, yaw):
    """Convert roll, pitch and yaw in radians to a quaternion."""
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy

    return qx, qy, qz, qw


class IMUDriverNode(Node):
    """Read WIT IMU frames and optionally publish embedded GPS data."""

    def __init__(self):
        super().__init__('imu_driver')
        self._declare_parameters()
        self._load_parameters()
        self._setup_publishers()
        self._initialize_state()
        self._start_driver_thread()

    def _declare_parameters(self):
        self.declare_parameter('port', '/dev/imu_usb')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('gps_enabled', False)
        self.declare_parameter('gps_frame_id', 'gps_link')
        self.declare_parameter('gps_min_satellites', 4)
        self.declare_parameter('gps_fix_topic', '/gps/fix')
        self.declare_parameter('gps_velocity_topic', '/gps/vel')
        self.declare_parameter('gps_compatibility_topic', '/gps')

    def _load_parameters(self):
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.gps_enabled = self.get_parameter('gps_enabled').value
        self.gps_frame_id = self.get_parameter('gps_frame_id').value
        self.gps_min_satellites = self.get_parameter('gps_min_satellites').value

    def _setup_publishers(self):
        self.publisher = self.create_publisher(Imu, '/imu/data', 10)
        self.gps_fix_publisher = None
        self.gps_velocity_publisher = None
        self.gps_compatibility_publisher = None
        if self.gps_enabled:
            self.gps_fix_publisher = self.create_publisher(
                NavSatFix, self.get_parameter('gps_fix_topic').value, 10
            )
            self.gps_velocity_publisher = self.create_publisher(
                TwistStamped,
                self.get_parameter('gps_velocity_topic').value,
                10,
            )
            self.gps_compatibility_publisher = self.create_publisher(
                Gps,
                self.get_parameter('gps_compatibility_topic').value,
                10,
            )

    def _initialize_state(self):
        self.linear_acceleration = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.euler_angles = [0.0, 0.0, 0.0]
        self.gps_longitude = 0.0
        self.gps_latitude = 0.0
        self.gps_altitude = math.nan
        self.gps_ground_speed = 0.0
        self.gps_satellites = 0
        self.gps_pdop = math.nan
        self.gps_hdop = math.nan
        self.gps_vdop = math.nan
        self.serial_port = None
        self.running = True
        self.receive_buffer = bytearray()

    def _start_driver_thread(self):
        self.driver_thread = threading.Thread(target=self.driver_loop, daemon=True)
        self.driver_thread.start()

    def driver_loop(self):
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0.2
            )
        except serial.SerialException as error:
            self.get_logger().error(
                f'无法打开 IMU 串口 {self.port}: {error}'
            )
            return

        self.get_logger().info(
            f'IMU 串口已打开：{self.port}，波特率：{self.baud}'
        )

        while self.running and rclpy.ok():
            try:
                data = self.serial_port.read(
                    self.serial_port.in_waiting or 1
                )

                if data:
                    self.receive_buffer.extend(data)
                    self.parse_buffer()

            except serial.SerialException as error:
                self.get_logger().error(
                    f'IMU 串口读取失败：{error}'
                )
                break

        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()

    def parse_buffer(self):
        while len(self.receive_buffer) >= 11:
            if self.receive_buffer[0] != 0x55:
                del self.receive_buffer[0]
                continue

            frame = self.receive_buffer[:11]

            if (sum(frame[:10]) & 0xFF) != frame[10]:
                del self.receive_buffer[0]
                continue

            del self.receive_buffer[:11]
            self.process_frame(frame)

    def process_frame(self, frame):
        frame_type = frame[1]
        values = struct.unpack('<hhhh', frame[2:10])

        if frame_type == 0x51:
            self.linear_acceleration = [
                values[i] / 32768.0 * 16.0 * 9.80665
                for i in range(3)
            ]

        elif frame_type == 0x52:
            self.angular_velocity = [
                values[i] / 32768.0 * 2000.0 * math.pi / 180.0
                for i in range(3)
            ]

        elif frame_type == 0x53:
            self.euler_angles = [
                values[i] / 32768.0 * math.pi
                for i in range(3)
            ]
            self.publish_imu()

        elif self.gps_enabled and frame_type == 0x57:
            self.process_gps_coordinates(frame)

        elif self.gps_enabled and frame_type == 0x58:
            self.process_gps_data(frame)

        elif self.gps_enabled and frame_type == 0x5A:
            self.process_gps_accuracy(frame)

    def publish_imu(self):
        message = Imu()

        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id

        roll, pitch, yaw = self.euler_angles
        qx, qy, qz, qw = quaternion_from_euler(
            roll,
            pitch,
            yaw
        )

        message.orientation.x = qx
        message.orientation.y = qy
        message.orientation.z = qz
        message.orientation.w = qw

        message.angular_velocity.x = self.angular_velocity[0]
        message.angular_velocity.y = self.angular_velocity[1]
        message.angular_velocity.z = self.angular_velocity[2]

        message.linear_acceleration.x = self.linear_acceleration[0]
        message.linear_acceleration.y = self.linear_acceleration[1]
        message.linear_acceleration.z = self.linear_acceleration[2]

        self.publisher.publish(message)

    @staticmethod
    def nmea_packed_coordinate_to_degrees(value):
        """Convert WIT 0x57 ddmm.mmmmm-without-dot data to degrees."""
        sign = -1.0 if value < 0 else 1.0
        value = abs(value)
        degrees = value // 10_000_000
        minutes = (value % 10_000_000) / 100_000.0
        return sign * (degrees + minutes / 60.0)

    def process_gps_coordinates(self, frame):
        longitude_raw, latitude_raw = struct.unpack('<ii', frame[2:10])
        self.gps_longitude = self.nmea_packed_coordinate_to_degrees(longitude_raw)
        self.gps_latitude = self.nmea_packed_coordinate_to_degrees(latitude_raw)

    def process_gps_data(self, frame):
        height_decimeters, _yaw_centidegrees, speed_millikph = struct.unpack(
            '<HHI', frame[2:10]
        )
        self.gps_altitude = height_decimeters / 10.0
        self.gps_ground_speed = speed_millikph / 1000.0 / 3.6

    def process_gps_accuracy(self, frame):
        satellites, pdop_raw, hdop_raw, vdop_raw = struct.unpack(
            '<HHHH', frame[2:10]
        )
        self.gps_satellites = satellites
        self.gps_pdop = pdop_raw / 100.0
        self.gps_hdop = hdop_raw / 100.0
        self.gps_vdop = vdop_raw / 100.0
        self.publish_gps()

    def publish_gps(self):
        has_fix = (
            self.gps_satellites >= self.gps_min_satellites
            and self.gps_latitude != 0.0
            and self.gps_longitude != 0.0
        )
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self.gps_frame_id
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.status.status = (
            NavSatStatus.STATUS_FIX if has_fix else NavSatStatus.STATUS_NO_FIX
        )
        if has_fix:
            fix.latitude = self.gps_latitude
            fix.longitude = self.gps_longitude
            fix.altitude = self.gps_altitude
        else:
            fix.latitude = math.nan
            fix.longitude = math.nan
            fix.altitude = math.nan
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.gps_fix_publisher.publish(fix)

        if not has_fix:
            return

        velocity = TwistStamped()
        velocity.header = fix.header
        velocity.twist.linear.x = self.gps_ground_speed
        self.gps_velocity_publisher.publish(velocity)

        compatibility = Gps()
        compatibility.n = self.gps_latitude
        compatibility.e = self.gps_longitude
        self.gps_compatibility_publisher.publish(compatibility)

    def destroy_node(self):
        self.running = False

        if self.serial_port is not None and self.serial_port.is_open:
            self.serial_port.close()

        if self.driver_thread.is_alive():
            self.driver_thread.join(timeout=1.0)

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = IMUDriverNode()

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
