import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from std_srvs.srv import Empty

import threading
import time


class TurtleController(Node):

    def __init__(self):
        super().__init__("turtle_agent_controller")

        # publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            "/turtle1/cmd_vel",
            10,
        )

        # subscriber
        self.pose = None
        self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.pose_callback,
            10,
        )

        # services
        self.pen_client = self.create_client(SetPen, "/turtle1/set_pen")
        self.clear_client = self.create_client(Empty, "/clear")

    # -------------------------
    # callbacks
    # -------------------------
    def pose_callback(self, msg):
        self.pose = msg

    # -------------------------
    # API used by tools
    # -------------------------
    def get_pose(self):
        if self.pose is None:
            return None

        return {
            "x": self.pose.x,
            "y": self.pose.y,
            "theta": self.pose.theta,
        }

    def cmd_vel(self, linear_x, angular_z, duration_s):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)

        start = time.time()

        while time.time() - start < duration_s:
            self.cmd_pub.publish(twist)
            time.sleep(0.05)

        self.stop()

    def stop(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

    def set_pen(self, r, g, b, width, off):
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off

        self.pen_client.wait_for_service()
        future = self.pen_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def clear(self):
        req = Empty.Request()

        self.clear_client.wait_for_service()
        future = self.clear_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)