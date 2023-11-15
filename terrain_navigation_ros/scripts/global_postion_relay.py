"""
ROS 2 node to convert from mavros_msgs/GlobalPositionTarget Message
to ardupilot_msgs/GlobalPosition
"""

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from mavros_msgs.msg import GlobalPositionTarget
from ardupilot_msgs.msg import GlobalPosition


class GlobalPositionRelay(Node):
    def __init__(self):
        super().__init__("global_position_relay")

        self.pub = self.create_publisher(
            GlobalPosition,
            "/ap/cmd_gps_pose",
            10,
        )

        self.sub = self.create_subscription(
            GlobalPositionTarget,
            "/mavros/setpoint_raw/global",
            self.global_position_cb,
            10,
        )

    def global_position_cb(self, msg):

        pos_msg = GlobalPosition()
        # header
        pos_msg.header = msg.header
        pos_msg.header.frame_id = "map"
        # coordinate frame and type mask
        pos_msg.coordinate_frame = msg.coordinate_frame
        pos_msg.type_mask = msg.type_mask
        # geodetic position (datum AMSL)
        pos_msg.latitude = msg.latitude
        pos_msg.longitude = msg.longitude
        pos_msg.altitude = msg.altitude
        # velocity
        pos_msg.velocity.linear.x = msg.velocity.x
        pos_msg.velocity.linear.y = msg.velocity.y
        pos_msg.velocity.linear.z = msg.velocity.z
        # acceleration or force
        pos_msg.acceleration_or_force.linear.x = msg.acceleration_or_force.x
        pos_msg.acceleration_or_force.linear.y = msg.acceleration_or_force.y
        pos_msg.acceleration_or_force.linear.z = msg.acceleration_or_force.z
        # yaw rate
        pos_msg.velocity.angular.z = msg.yaw_rate
        # TODO: yaw (no corresponding member in GlobalPosition)

        self.pub.publish(pos_msg)

        self.get_logger().info("recv: {}\n".format(msg))
        self.get_logger().info("send: {}\n".format(pos_msg))


def main(args=None):
    rclpy.init(args=args)

    node = GlobalPositionRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
