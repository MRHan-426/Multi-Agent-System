#!/usr/bin/env python
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

from pyzbar import pyzbar 


class Sense:
    def __init__(self):
        self.bridge = CvBridge()
        self.rgb_image = None
        self.dep_image = None
        self.K = None

        self.info_sub = rospy.Subscriber(
            "/camera/rgb/camera_info", CameraInfo, self.info_cb, queue_size=1
        )
        self.rgb_sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.rgb_cb, queue_size=1
        )
        self.dep_sub = rospy.Subscriber(
            "/camera/depth/image_raw", Image, self.dep_cb, queue_size=1
        )

        self.x = 0
        self.y = 0
        self.theta = 0

        self.kp = 0.3
        self.ka = 0.5
        self.kb = -0.15

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)
        self.vel_pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)

        rospy.Timer(rospy.Duration(0.1), self.timer_cb)
        pass

    def info_cb(self, data):
        # get info only once
        self.info_sub.unregister()

        self.K = data.K
        self.K = np.array(self.K)
        self.K = np.reshape(self.K, (3, 3))

        print(self.K)
        pass

    def rgb_cb(self, data):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # print("rgb", self.rgb_image.shape)
            pass
        except CvBridgeError as e:
            print(e)
            pass

        # cv2.imshow("Image window", self.rgb_image)
        # cv2.waitKey(3)
        pass

    def dep_cb(self, data):
        try:
            self.dep_image = self.bridge.imgmsg_to_cv2(data)
            # print("dep", self.dep_image.shape)
            pass
        except CvBridgeError as e:
            print(e)
            pass
        pass

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
        # rospy.loginfo(info)

    def timer_cb(self, data):
        try:
            frame = self.rgb_image

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  
            barcodes = pyzbar.decode(gray)
            (x, y, w, h) = barcodes[0].rect
            u, v = x+w/2,y+h/2
            
            print("u,v:",u, v)

            if u != 0 or v != 0:
                print("aaa")
                d = self.dep_image[int(v)][int(u)] / 1000.0
                x, y, _ = self.getCoordinateInWorld(u, v, d)

                self.x_d, self.y_d = x, y
                print("target = ({}, {}), current = ({}, {}).".format(self.x_d, self.y_d, self.x, self.y))

                velocity, oumiga = self.lpj_stabilize()

                vel = Twist()
                vel.linear.x = velocity
                vel.angular.z = oumiga
                print("linear_v,angle_w:",velocity,oumiga)
                self.vel_pub.publish(vel)
        except:
            pass
        pass

    def getCoordinateInWorld(self, u, v, z):
        x_c = z * (u - self.K[0, 2]) / self.K[0, 0]
        y_c = z * (v - self.K[1, 2]) / self.K[1, 1]
        z_c = z

        x_c, y_c, z_c = z_c, -x_c, -y_c

        x_w = x_c * math.cos(self.theta) - y_c * math.sin(self.theta) + self.x
        y_w = x_c * math.sin(self.theta) + y_c * math.cos(self.theta) + self.y
        z_w = z

        return x_w, y_w, z_w
    



def main(args):
    rospy.init_node("sense")
    sense = Sense()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)
