#!/usr/bin/env python
from numpy.lib.function_base import angle
import rospy
import sys
import cv2
import math
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose2D

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

# import qrcode_recog
from pyzbar import pyzbar

K = np.array([[575.81, 0.0, 314.5],
              [0.0, 575.81, 235.5],
              [0.0, 0.0,    1.0]])


class VisionModule:
    def __init__(self):
        # picture vars
        self.rgb_img = None
        self.dep_img = None
        self.bridge = CvBridge()

        # location vars
        # qrcode pos
        self.u_pos = 0
        self.v_pos = 0

        # my pos
        self.x_pos = 0
        self.y_pos = 0
        self.angle = 0

        # goal pos
        self.goal_x_pos = 0
        self.goal_y_pos = 0

        self.kp = 0.15
        self.ka = 0.2
        self.kb = -0.15

        # ros publisher and subscriber
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.rgb_img_callback)
        rospy.Subscriber("/camera/depth/image_raw",
                         Image, self.depth_img_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.pos_pub = rospy.Publisher(
            '/my_pos/global_pos', Odometry, queue_size=1)

        self.vel_pub = rospy.Publisher(
            "/mobile_base/commands/velocity", Twist, queue_size=1
        )

        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def timer_callback(self, data):
        try:
            # print('in')
            frame = self.rgb_img
            # if frame != None:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            barcodes = pyzbar.decode(gray)
            (x, y, w, h) = barcodes[0].rect

            u, v = x+w/2, y+h/2
            self.u_pos, self.v_pos = u, v
            print("the u, v is:%f %f" % (u, v))

            if u or v:
                print("the goal_x_pos, goal_y_pos")

                z = self.dep_img[v][u] / 1000
                print("*****Z:%f******" % z)
                self.goal_x_pos, self.goal_y_pos, _ = self.pixel2world(u, v, z)
                print("the goal_x_pos, goal_y_pos is:%f %f" %
                      (self.goal_x_pos, self.goal_y_pos))

                pos = Odometry()
                pos.pose.pose.position.x = self.goal_x_pos
                pos.pose.pose.position.y = self.goal_y_pos
                self.pos_pub.publish(pos)

                linearSpeed, angularSpeed = self.move_control()
                vel = Twist()
                vel.linear.x = linearSpeed
                vel.angular.z = angularSpeed
                self.vel_pub.publish(vel)
        except:
            pass

    def rgb_img_callback(self, data):
        self.rgb_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        # print(self.rgb_img)

    def depth_img_callback(self, data):
        self.dep_img = self.bridge.imgmsg_to_cv2(data)

    def odom_callback(self, data):
        pos = data.pose.pose.position
        ori = data.pose.pose.orientation

        self.x_pos = pos.x
        self.y_pos = pos.y
        self.angle = euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])

    def pixel2world(self, u, v, z):
        x_cam = z * (u - K[0, 2]) / K[0, 0]
        y_cam = z * (v - K[1, 2]) / K[1, 1]
        z_cam = z

        x_cam, y_cam, z_cam = z_cam, -x_cam, -y_cam

        x_world = x_cam * np.cos(self.angle) - y_cam * \
            np.sin(self.angle) + self.x_pos
        y_world = x_cam * np.sin(self.angle) + y_cam * \
            np.cos(self.angle) + self.y_pos
        z_world = z

        return x_world, y_world, z_world

    def move_control(self):
        rou = math.sqrt((self.goal_x_pos - self.x_pos) ** 2 +
                        (self.goal_y_pos - self.y_pos) ** 2)
        beta = -math.atan2(self.goal_y_pos - self.y_pos,
                           self.goal_x_pos - self.x_pos)
        alpha = -self.angle - beta

        v = self.kp * rou
        w = self.ka * alpha + self.kb * beta

        info = "rou = {}, alpha = {}, beta = {}".format(rou, alpha, beta)
        rospy.loginfo(info)

        return v, w


def main():
    rospy.init_node("get_trans")
    vision = VisionModule()
    rospy.spin()


if __name__ == '__main__':
    main()
