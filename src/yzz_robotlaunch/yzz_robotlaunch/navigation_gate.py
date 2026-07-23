"""Start the Nav2 navigation lifecycle only after AMCL establishes map TF."""

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


class NavigationGate(Node):
    """Wait for map -> base_link, then start the navigation lifecycle."""

    def __init__(self):
        super().__init__('navigation_gate')
        self._setup_interfaces()
        self._started = False
        self._start_gate_timer()
        self.get_logger().info(
            'Waiting for map -> base_link before starting Nav2 navigation'
        )

    def _setup_interfaces(self):
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._client = self.create_client(
            ManageLifecycleNodes,
            '/lifecycle_manager_navigation/manage_nodes',
        )

    def _start_gate_timer(self):
        self._timer = self.create_timer(0.5, self._try_start_navigation)

    def _try_start_navigation(self):
        if self._started:
            return

        if not self._tf_buffer.can_transform(
            'map', 'base_link', Time(), timeout=Duration(seconds=0.0)
        ):
            return

        if not self._client.service_is_ready():
            return

        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        self._started = True
        future = self._client.call_async(request)
        future.add_done_callback(self._on_startup_response)
        self.get_logger().info('Initial pose received; starting Nav2 navigation')

    def _on_startup_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Nav2 navigation is active')
            else:
                self.get_logger().error('Nav2 lifecycle startup was rejected')
                self._started = False
        except Exception as error:  # pylint: disable=broad-except
            self.get_logger().error(f'Nav2 lifecycle startup failed: {error}')
            self._started = False


def main(args=None):
    rclpy.init(args=args)
    node = NavigationGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

