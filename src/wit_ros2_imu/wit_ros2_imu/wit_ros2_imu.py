import math
import struct
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial


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
    def __init__(self):
        super().__init__('imu_driver')

        self.declare_parameter('port', '/dev/imu_usb')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('frame_id', 'imu_link')

        self.port = (
            self.get_parameter('port')
            .get_parameter_value()
            .string_value
        )
        self.baud = (
            self.get_parameter('baud')
            .get_parameter_value()
            .integer_value
        )
        self.frame_id = (
            self.get_parameter('frame_id')
            .get_parameter_value()
            .string_value
        )

        self.publisher = self.create_publisher(
            Imu,
            '/imu/data',
            10
        )

        self.linear_acceleration = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.euler_angles = [0.0, 0.0, 0.0]

        self.serial_port = None
        self.running = True
        self.receive_buffer = bytearray()

        self.driver_thread = threading.Thread(
            target=self.driver_loop,
            daemon=True
        )
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
