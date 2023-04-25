#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion


def subscriber_cb(data):
    posistion = data.pose.pose.position
    oriention = data.pose.pose.orientation

    x = posistion.x
    y = posistion.y

    euler = euler_from_quaternion([oriention.x, oriention.y, oriention.z, oriention.w])
    theta = euler[2]

    info = "(x, y, theta) = ({}, {}, {})".format(x, y, theta)
    rospy.loginfo(info)


def main():
    rospy.init_node("odom_subscriber", anonymous=True)

    rospy.Subscriber("/odom", Odometry, subscriber_cb, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
