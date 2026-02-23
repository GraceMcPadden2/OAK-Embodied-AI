#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

#stabilizes controller, prevents high speeds
def clamp(v, lo, hi): return max(lo, min(hi, v))

#Force angle to be in range -pi to pi
def wrap(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

#Goto class built on top of Node
class Goto(Node):
    def __init__(self):
        super().__init__("goto_node")
        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.sub = self.create_subscription(Pose, "/turtle1/pose", self.on_pose, 10)
        self.pose = None
        self.k_lin, self.k_ang = 1.5, 6.0
        self.max_lin, self.max_ang = 2.0, 3.0
        self.pos_tol, self.yaw_tol = 0.05, 0.05

    #sotres position
    def on_pose(self, msg): self.pose = msg

    #sends 0 velocity command
    def stop(self): self.pub.publish(Twist())

    def goto(self, x, y, timeout_s=10.0):

        #make sure client stays in the window
        x, y = clamp(x, 0.5, 10.5), clamp(y, 0.5, 10.5)

        #remeber time it started
        t0 = time.time()

        #while ros is connected and time not expired
        while rclpy.ok() and (time.time() - t0) < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.0)

            #if none, sleep
            if self.pose is None:
                time.sleep(0.02); continue

            #compute error in position to goal
            dx, dy = x - self.pose.x, y - self.pose.y

            #computes dist to goal pythagoeran therom
            dist = math.hypot(dx, dy)

            #if distance is smaller than error, stop
            if dist < self.pos_tol:
                self.stop()
                return True

            #change direction vector into angle, compute how far to twist
            target = math.atan2(dy, dx)
            yaw_err = wrap(target - self.pose.theta)

            #if facing the wrong direction, twist. Else, move forward.
            cmd = Twist()
            if abs(yaw_err) > self.yaw_tol:
                cmd.angular.z = clamp(self.k_ang * yaw_err, -self.max_ang, self.max_ang)
            else:
                cmd.linear.x  = clamp(self.k_lin * dist, 0.0, self.max_lin)
                cmd.angular.z = clamp(self.k_ang * yaw_err, -self.max_ang, self.max_ang)

            self.pub.publish(cmd)
            time.sleep(0.02)

        self.stop()
        return False

def main():
    rclpy.init()
    node = Goto()
    try:
        ok = node.goto(5, 5)
        print("goto ok:", ok)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()