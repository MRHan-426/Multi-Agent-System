#!/usr/bin/env python
import rospy
import sys
import math
import numpy as np
import matplotlib.pyplot as plt

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


class Trajectory:
    def __init__(self, duration, controller="lpj"):
        controllers = ["lpj"]
        assert controller in controllers, "Controller not defined."
        self.controller = controller

        self.duration = duration
        self.N = duration * 10

        # lpj control parameters
        self.k1 = 0.3
        self.k2 = 0.5
        self.k3 = 0.7

        self.x = 0
        self.y = 0
        self.theta = 0

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)

        self.vel_pub = rospy.Publisher(
            "/mobile_base/commands/velocity", Twist, queue_size=1
        )

        rate = rospy.Rate(10.0)
        self.x_d, self.y_d, self.theta_d, self.v_d, self.w_d = self.genTraj()
        xList = []
        yList = []

        k = 0
        while k < self.N:
            v, w = self.lpj_trajectory(k)

            vel = Twist()
            vel.linear.x = v
            vel.angular.z = w
            self.vel_pub.publish(vel)

            xList.append(self.x)
            yList.append(self.y)

            k += 1

            rate.sleep()

        plt.scatter(xList, yList, c="b")
        plt.scatter(self.x_d, self.y_d, c="r")
        plt.show()

    def genTraj(self):
        t = np.linspace(0, self.duration, num=self.N)
        w = 0.5 * np.ones(t.shape)
        r = 1.0
        v = w * r
        x, y = r * np.cos(w * t), r * np.sin(w * t)
        theta = math.pi / 2 + w * t

        return x, y, theta, v, w

    def lpj_trajectory(self, k):
        e_x = self.x_d[k] - self.x
        e_y = self.y_d[k] - self.y
        v_r = self.v_d[k]
        w_r = self.w_d[k]

        x_e = e_x * math.cos(self.theta) + e_y * math.sin(self.theta)
        y_e = -e_x * math.sin(self.theta) + e_y * math.cos(self.theta)

        theta_e = self.theta_d[k] - self.theta

        v = v_r * math.cos(theta_e) + self.k2 * x_e
        w = w_r + self.k1 * v_r * y_e + self.k3 * math.sin(theta_e)

        return v, w

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
    rospy.init_node("trajectroy")
    stabilize = Trajectory(60, "lpj")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main(sys.argv)
