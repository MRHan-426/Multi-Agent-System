#!/usr/bin/env python
import rospy
import sys
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


class Stabilize:
    def __init__(self, x_d=1.0, y_d=1.0, controller="lpj"):
        controllers = ["lpj", "guyue"]
        assert controller in controllers, "Controller not defined."
        self.controller = controller

        # lpj control parameters
        self.kp = 0.15
        self.ka = 0.2
        self.kb = -0.15

        # guyue control parameters
        self.kv = 0.1
        self.kw = 0.7

        self.x_d = x_d
        self.y_d = y_d

        self.x = 0
        self.y = 0
        self.theta = 0

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)

        self.vel_pub = rospy.Publisher(
            "/mobile_base/commands/velocity", Twist, queue_size=1
        )

        rospy.Timer(rospy.Duration(0.1), self.timer_cb)

    def lpj_stabilize(self):
        ruo = math.sqrt((self.x_d - self.x) ** 2 + (self.y_d - self.y) ** 2)
        beta = -math.atan2(self.y_d - self.y, self.x_d - self.x)
        alpha = -self.theta - beta

        v = self.kp * ruo
        w = self.ka * alpha + self.kb * beta

        info = "ruo = {}, alpha = {}, beta = {}".format(ruo, alpha, beta)
        rospy.loginfo(info)

        return v, w

    def guyue_stabilize(self):
        if self.theta < 0:
            self.theta = self.theta + 2 * math.pi

        d_e = math.sqrt(
            math.pow((self.x_d - self.x), 2) + math.pow((self.y_d - self.y), 2)
        )

        if (self.y_d - self.y) == 0 and (self.x_d - self.x) > 0:
            theta_d = 0
        if (self.y_d - self.y) > 0 and (self.x_d - self.x) > 0:
            theta_d = math.atan((self.y_d - self.y) / (self.x_d - self.x))
        if (self.y_d - self.y) > 0 and (self.x_d - self.x) == 0:
            theta_d = 0.5 * math.pi
        if (self.y_d - self.y) > 0 and (self.x_d - self.x) < 0:
            theta_d = math.atan((self.y_d - self.y) / (self.x_d - self.x)) + math.pi
        if (self.y_d - self.y) == 0 and (self.x_d - self.x) < 0:
            theta_d = math.pi
        if (self.y_d - self.y) < 0 and (self.x_d - self.x) < 0:
            theta_d = math.atan((self.y_d - self.y) / (self.x_d - self.x)) + math.pi
        if (self.y_d - self.y) < 0 and (self.x_d - self.x) == 0:
            theta_d = 1.5 * math.pi
        if (self.y_d - self.y) < 0 and (self.x_d - self.x) > 0:
            theta_d = math.atan((self.y_d - self.y) / (self.x_d - self.x)) + 2 * math.pi

        theta_e = theta_d - self.theta
        if theta_e < -math.pi:
            theta_e = theta_e + 2 * math.pi
        if theta_e > math.pi:
            theta_e = theta_e - 2 * math.pi

        v = self.kv * d_e
        w = self.kw * theta_e
        return v, w

    def timer_cb(self, data):
        if self.controller == "lpj":
            v, w = self.lpj_stabilize()
        elif self.controller == "guyue":
            v, w = self.guyue_stabilize()

        vel = Twist()
        vel.linear.x = v
        vel.angular.z = w

        self.vel_pub.publish(vel)

    def odom_cb(self, data):
        posistion = data.pose.pose.position
        oriention = data.pose.pose.orientation

        self.x = posistion.x
        self.y = posistion.y

        _, _, self.theta = euler_from_quaternion(
            [oriention.x, oriention.y, oriention.z, oriention.w]
        )

        info = "(self.x, self.y, theta) = ({}, {}, {})".format(
            self.x, self.y, self.theta
        )
        rospy.loginfo(info)

    pass


def main(args):
    rospy.init_node("stabilize")
    stabilize = Stabilize(1.0, 1.0, "lpj")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main(sys.argv)
