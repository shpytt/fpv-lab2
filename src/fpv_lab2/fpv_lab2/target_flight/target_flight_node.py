from geometry_msgs.msg import TwistStamped

from ..flight_test.flight_state import FlightState
from ..flight_test.flight_test_node import FlightTestNode
from ..flight_test.handlers import HANDLERS
from .handlers import (
    NavigatingHandler,
    TargetHoveringHandler,
    TargetWaitingForLandingHandler,
)


class TargetFlightNode(FlightTestNode):
    """Політ до точки (x0 - dx, y0 - dy) із подальшою посадкою."""

    HOVER_DURATION = 5.0
    TAKEOFF_ALTITUDE = 4.0

    def __init__(self):
        super().__init__()

        self.declare_parameter("dx", 4.6)
        self.declare_parameter("dy", 1.1)
        self.declare_parameter("altitude", 4.0)
        self.dx = float(self.get_parameter("dx").value)
        self.TAKEOFF_ALTITUDE = float(
            self.get_parameter("altitude").value
        )
        self.dy = float(self.get_parameter("dy").value)

        self.start = None
        self.target = None

        self.cmd_vel_publisher = self.create_publisher(
            TwistStamped,
            "/ap/v1/cmd_vel",
            10,
        )

        self._handlers = {
            **HANDLERS,
            FlightState.HOVERING: TargetHoveringHandler(),
            FlightState.NAVIGATING: NavigatingHandler(),
            FlightState.WAITING_FOR_LANDING: TargetWaitingForLandingHandler(),
        }

        self.get_logger().info(
            f"Target flight: dx={self.dx:.2f} m, dy={self.dy:.2f} m"
        )

    def odometry_callback(self, message):
        super().odometry_callback(message)

        if self.start is None:
            position = message.pose.pose.position
            self.start = (position.x, position.y, position.z)
            self.target = (position.x - self.dx, position.y - self.dy)
            self.get_logger().info(
                f"Start position: x0={position.x:.3f}, y0={position.y:.3f}; "
                f"target: x={self.target[0]:.3f}, y={self.target[1]:.3f}"
            )

    def send_velocity(self, east, north, up=0.0):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.twist.linear.x = float(east)
        message.twist.linear.y = float(north)
        message.twist.linear.z = float(up)
        self.cmd_vel_publisher.publish(message)
