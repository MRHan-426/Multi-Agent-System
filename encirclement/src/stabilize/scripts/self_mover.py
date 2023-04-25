#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


def publisher_cb(data):
    global pub

    velocity = Twist()
    velocity.linear.x = 0.1
    velocity.angular.z = 0.3

    pub.publish(velocity)
    rospy.loginfo("Publish Velocity.")


def main():
    global pub

    rospy.init_node("self_mover", anonymous=True)

    pub = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=1)

    rospy.Timer(rospy.Duration(0.1), publisher_cb)

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
