#!/usr/bin/env python
from __future__ import print_function
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import rospy
import sys

import numpy as np

# import roslib
# roslib.load_manifest('my_package')


class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("/bridge_out", Image)

        self.bridge = CvBridge()
        # self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.callback)
        self.image_sub = rospy.Subscriber(
            "/camera/depth/image_raw", Image, self.callback
        )

    def callback(self, data):
        try:
            # cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = self.bridge.imgmsg_to_cv2(data)
            # print(type(cv_image))
            # print(cv_image.shape)

            cv_image = np.right_shift(cv_image, 3)
            cv_image = np.bitwise_and(cv_image, 2047)
            print(cv_image[240, 320] / 2048.0 * 255)
            cv_image = cv_image * 255 / 256
            cv_image = np.right_shift(cv_image, 2)
            print(cv_image.shape, bin(cv_image[240, 320]), cv_image[240, 320])
            # print(cv_image[240, 320]/2048.0 * 6)
        except CvBridgeError as e:
            print(e)

        # (rows, cols, channels) = cv_image.shape
        # if cols > 60 and rows > 60:
        #     cv2.circle(cv_image, (50, 50), 10, 255)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        try:
            # self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image))
        except CvBridgeError as e:
            print(e)


def main(args):
    ic = image_converter()
    rospy.init_node("image_converter", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)
