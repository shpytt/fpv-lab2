import math
import os
import time
from typing import TYPE_CHECKING

from ..flight_test.flight_state import FlightState
from ..flight_test.handlers.handler import FlightStateHandler
from ..flight_test.handlers.waiting_for_landing import (
    WaitingForLandingHandler,
)

if TYPE_CHECKING:
    from .target_flight_node import TargetFlightNode


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class TargetHoveringHandler(FlightStateHandler):
    """Чекає набору висоти, зависає, потім переходить до польоту до цілі."""

    ALTITUDE_MARGIN = 0.2
    CLIMB_TIMEOUT = 90.0

    def __init__(self):
        self._climb_started = None
        self._altitude_reached_at = None

    def handle(self, node: "TargetFlightNode") -> None:
        if node.status is None or not node.status.armed:
            node.fail("Drone disarmed unexpectedly during flight")
            return

        if node.odometry is None or node.start is None:
            return

        now = time.monotonic()
        if self._climb_started is None:
            self._climb_started = now

        z = node.odometry.pose.pose.position.z
        target_z = node.start[2] + node.TAKEOFF_ALTITUDE

        if self._altitude_reached_at is None:
            if z >= target_z - self.ALTITUDE_MARGIN:
                self._altitude_reached_at = now
                node.get_logger().info(
                    f"Takeoff altitude reached: {z - node.start[2]:.2f} m, "
                    f"hovering for {node.HOVER_DURATION:.1f} s"
                )
            elif now - self._climb_started > self.CLIMB_TIMEOUT:
                node.fail("Takeoff altitude was not reached in time")
            return

        if now - self._altitude_reached_at >= node.HOVER_DURATION:
            node.get_logger().info("Hover completed, flying to target")
            node.state = FlightState.NAVIGATING


class NavigatingHandler(FlightStateHandler):
    """Політ до цілі: пропорційне керування швидкістю зі сповільненням."""

    MAX_SPEED = 0.8
    MIN_SPEED = 0.08
    SPEED_GAIN = 0.7
    CLIMB_GAIN = 0.5
    MAX_CLIMB_SPEED = 0.3
    ARRIVAL_TOLERANCE = 0.15
    RESUME_TOLERANCE = 0.30
    SETTLE_TIME = 2.0
    NAV_TIMEOUT = 240.0
    PROGRESS_PERIOD = 20.0
    MIN_PROGRESS = 0.3
    LOG_PERIOD = 2.0

    def __init__(self):
        self._started_at = None
        self._settle_since = None
        self._last_check_time = None
        self._last_check_distance = None
        self._last_log = 0.0

    def handle(self, node: "TargetFlightNode") -> None:
        if node.status is None or not node.status.armed:
            node.fail("Drone disarmed unexpectedly during flight")
            return

        if node.odometry is None or node.target is None:
            return

        position = node.odometry.pose.pose.position
        error_x = node.target[0] - position.x
        error_y = node.target[1] - position.y
        distance = math.hypot(error_x, error_y)
        now = time.monotonic()

        if self._started_at is None:
            self._started_at = now
            self._last_check_time = now
            self._last_check_distance = distance
            node.get_logger().info(
                f"Navigating: distance to target {distance:.2f} m"
            )

        if now - self._started_at > self.NAV_TIMEOUT:
            self.abort(node, "Navigation timeout")
            return

        hold_z = node.start[2] + node.TAKEOFF_ALTITUDE
        climb = clamp(
            self.CLIMB_GAIN * (hold_z - position.z),
            -self.MAX_CLIMB_SPEED,
            self.MAX_CLIMB_SPEED,
        )

        if self._settle_since is None:
            if now - self._last_check_time >= self.PROGRESS_PERIOD:
                progress = self._last_check_distance - distance
                if distance > 1.0 and progress < self.MIN_PROGRESS:
                    self.abort(
                        node,
                        "No progress towards the target "
                        f"(only {progress:.2f} m in "
                        f"{self.PROGRESS_PERIOD:.0f} s)",
                    )
                    return
                self._last_check_time = now
                self._last_check_distance = distance

            if distance <= self.ARRIVAL_TOLERANCE:
                self._settle_since = now
                node.send_velocity(0.0, 0.0, climb)
                node.get_logger().info(
                    f"Target reached (distance {distance:.3f} m), stopping"
                )
                return

            speed = clamp(
                self.SPEED_GAIN * distance, self.MIN_SPEED, self.MAX_SPEED
            )
            node.send_velocity(
                speed * error_x / distance,
                speed * error_y / distance,
                climb,
            )

            if now - self._last_log >= self.LOG_PERIOD:
                self._last_log = now
                node.get_logger().info(
                    f"Position ({position.x:.2f}, {position.y:.2f}), "
                    f"distance {distance:.2f} m, speed {speed:.2f} m/s"
                )
            return

        node.send_velocity(0.0, 0.0, climb)

        if distance > self.RESUME_TOLERANCE:
            self._settle_since = None
            node.get_logger().info("Drifted from target, resuming approach")
            return

        if now - self._settle_since >= self.SETTLE_TIME:
            self.finish(node, position.x, position.y, distance)

    def abort(self, node: "TargetFlightNode", reason: str) -> None:
        node.send_velocity(0.0, 0.0, 0.0)
        node.get_logger().error(f"{reason}, starting emergency landing")
        node.state = FlightState.LANDING

    def finish(self, node, x: float, y: float, error: float) -> None:
        start = node.start
        target = node.target
        lines = [
            f"Start position: x0={start[0]:.3f}, y0={start[1]:.3f}",
            f"Target position: x={target[0]:.3f}, y={target[1]:.3f}",
            f"Actual position before landing: x={x:.3f}, y={y:.3f}",
            f"Positioning error: {error:.3f} m",
        ]
        for line in lines:
            node.get_logger().info(line)

        try:
            directory = os.path.expanduser("~/lab2_report")
            os.makedirs(directory, exist_ok=True)
            with open(
                os.path.join(directory, "target_flight_result.txt"), "w"
            ) as file:
                file.write("\n".join(lines) + "\n")
        except OSError as error_info:
            node.get_logger().warn(f"Could not save result file: {error_info}")

        node.send_velocity(0.0, 0.0, 0.0)
        node.get_logger().info("Starting landing")
        node.state = FlightState.LANDING


class TargetWaitingForLandingHandler(WaitingForLandingHandler):
    """Те саме очікування посадки, але ще й логує кінцеву позицію."""

    def handle(self, node: "TargetFlightNode") -> None:
        if (
            node.status is not None
            and not node.status.flying
            and node.odometry is not None
        ):
            position = node.odometry.pose.pose.position
            node.get_logger().info(
                f"Landing position: x={position.x:.3f}, y={position.y:.3f}"
            )
        super().handle(node)
