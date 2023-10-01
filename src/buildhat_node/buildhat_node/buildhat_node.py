import time
from buildhat import PassiveMotor
import numpy as np
from rclpy.node import Node
import rclpy

from geometry_msgs.msg import Twist, Vector3

# ^^^^^
# -B   A
#
# -D   C

motors_orientation = np.array([1, -1, 1, -1])

# A B C D
forward_m = np.array([1, 1, 1, 1])
rotate_m = np.array([1, -1, 1, -1])
strafe_m = np.array([1, -1, -1, 1])


def start_motors(motors, vels):
    for m, v in zip(motors, 100 * vels * motors_orientation):
        m.start(max(min(v, 100), -100))


def stop_motors(motors):
    for m in motors:
        m.stop()


class BuildhatNode(Node):
    def __init__(self):
        super().__init__("buildhat_node")

        self.motors = [
            PassiveMotor("A"),
            PassiveMotor("B"),
            PassiveMotor("C"),
            PassiveMotor("D"),
        ]

        self.cmd_vel_sub = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )

    def cmd_vel_callback(self, msg: Twist) -> None:
        start_motors(
            self.motors,
            forward_m * msg.linear.x
            + strafe_m * msg.linear.y
            + rotate_m * msg.angular.z,
        )


def main(args=None):
    rclpy.init(args=args)

    node = BuildhatNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
