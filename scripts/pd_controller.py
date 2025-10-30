#!/usr/bin/env python3
import math
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry


def quat2rot(q):
    """Compute the rotation matrix associated to a quaternion.
    q       -- quaternion with scalar as first element [qw qx qy qz]
    """
    r11 = q[0] ** 2 + q[1] ** 2 - q[2] ** 2 - q[3] ** 2
    r21 = 2 * (q[1] * q[2] + q[0] * q[3])
    r31 = 2 * (q[1] * q[3] - q[0] * q[2])
    r12 = 2 * (q[1] * q[2] - q[0] * q[3])
    r22 = q[0] ** 2 - q[1] ** 2 + q[2] ** 2 - q[3] ** 2
    r32 = 2 * (q[2] * q[3] + q[0] * q[1])
    r13 = 2 * (q[1] * q[3] + q[0] * q[2])
    r23 = 2 * (q[2] * q[3] - q[0] * q[1])
    r33 = q[0] ** 2 - q[1] ** 2 - q[2] ** 2 + q[3] ** 2
    return np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])


class PDController(Node):
    def __init__(self):
        super().__init__('pd_controller')

        ## gains
        self.Kp_xy = 0.5
        self.Kp_z = 0.5
        self.Kd_xy = 1.
        self.Kd_z = 1.

        self.max_accel = 2.0  # m/s^2

        self.ref = None
        self.pos = None
        self.rot = None
        self.vel = None

        ## topics
        self.create_subscription(Odometry, '~/odom', self.odom_cb, 10)
        self.create_subscription(Point, '~/ref', self.ref_cb, 10)
        self.pub = self.create_publisher(Twist, '~/cmd', 10)

        ## loop 50Hz
        self.create_timer(0.02, self.control_loop)

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        self.pos = [p.x, p.y, p.z]
        self.rot = quat2rot([q.w, q.x, q.y, q.z])
        v_B = np.array([v.x, v.y, v.z])
        self.vel = self.rot @ v_B

        if self.ref is None:
            self.ref = [p.x, p.y, p.z]

    def ref_cb(self, msg: Point):
        self.ref = [msg.x, msg.y, msg.z]

    def control_loop(self):
        if self.pos is None:
            return

        ## pos error
        ex = self.ref[0] - self.pos[0]
        ey = self.ref[1] - self.pos[1]
        ez = self.ref[2] - self.pos[2]

        ## PD control
        ax = self.Kp_xy * ex - self.Kd_xy * self.vel[0]
        ay = self.Kp_xy * ey - self.Kd_xy * self.vel[1]
        az = self.Kp_z * ez - self.Kd_z * self.vel[2]

        ## clamp to max_accel
        acc_norm = (ax**2 + ay**2 + az**2) ** 0.5
        if acc_norm > self.max_accel:
            scale = self.max_accel / acc_norm
            ax *= scale
            ay *= scale
            az *= scale

        ## publish
        msg = Twist()
        a_W = np.array([ax, ay, az])
        a_B = self.rot.T @ a_W
        msg.linear.x, msg.linear.y, msg.linear.z = a_B[0], a_B[1], a_B[2]
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
