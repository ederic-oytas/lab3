#!/usr/bin/env python3
from typing import Any, Optional

import numpy as np
import rclpy
import rclpy.time
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException  # type: ignore
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class SafetyNode(Node):
    """Node for emergency braking."""

    def __init__(self) -> None:
        """Creates a new instance."""
        super().__init__("ederic_safety_node")

        self.declare_parameter("base_frame")
        self.declare_parameter("laser_frame")
        self.declare_parameter("ttc_thresh")

        self.base_frame: str = self.get_parameter_value_checked("base_frame", str)
        self.laser_frame: str = self.get_parameter_value_checked("laser_frame", str)
        self.ttc_thresh: str = self.get_parameter_value_checked("ttc_thresh", float)

        self.get_logger().info(
            "\n"
            "Parameters received:\n"
            f"  base_frame  = {self.base_frame!r}\n"
            f"  laser_frame = {self.laser_frame!r}\n"
            f"  ttc_thresh  = {self.ttc_thresh!r}"
        )

        # Transform data
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriptions and publisher
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)

        # Other attributes
        self.last_odom_msg: Optional[Odometry] = None
        """Last odometry message received, None if there was none."""

    def get_parameter_value_checked(
        self, name: str, expected_type: type, check_none: bool = True
    ) -> Any:
        """Helper method to get the value of a parameter, check that it's not
        None, then check that its type is given"""
        value = self.get_parameter(name).value
        if check_none and value is None:
            raise ValueError(f"No value given for parameter {name!r}")
        if not isinstance(value, expected_type):
            raise TypeError(
                f"Given value for parameter {name!r} is not of type "
                f"{expected_type.__name__!r}"
            )
        return value

    def odom_callback(self, odom_msg: Odometry) -> None:
        """Called when an odometry message is received."""
        self.last_odom_msg = odom_msg

    def scan_callback(self, scan_msg: LaserScan) -> None:
        """Called when a laser scan message is received."""
        if self.last_odom_msg is None:
            self.get_logger().info("No Odometry yet received, skipping...")
            return

        try:
            transform_laser_to_base: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.laser_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            self.get_logger().info(
                "Could not look up laser-to-base transformation, skipping..."
            )
            return

        v_x = self.last_odom_msg.twist.twist.linear.x
        if v_x <= 0.01:
            return  # Do nothing if almost stopped

        angle_min = scan_msg.angle_min
        angle_inc = scan_msg.angle_increment

        ranges = np.array(scan_msg.ranges)
        ttcs = np.zeros_like(ranges)
        for i, r_laser in enumerate(ranges):
            # TODO: Calculate values in the laser frame
            theta_laser = ...
            x_laser = ...
            y_laser = ...
            # TODO: Calculate values in the base frame
            #   (hint: use np.sqrt() and np.arctan2())
            x_base = ...
            y_base = ...
            r_base = ...
            theta_base = ...
            # TODO: Calculate r_dot and TTC
            r_dot = ...
            ttcs[i] = ...

        # TODO: Replace False with correct condition
        #   (hint: use np.min())
        is_any_ttc_below_threshold = False
        if is_any_ttc_below_threshold:
            pass
            # TODO: Send brake message


def main() -> None:
    """Main entry point function."""
    rclpy.init()
    node = SafetyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
