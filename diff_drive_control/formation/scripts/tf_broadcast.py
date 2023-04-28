#!/usr/bin/env python
# coding=utf-8

import rospy
import tf
import nav_msgs.msg


def handle_robot_pose(msg, robotname):
    br = tf.TransformBroadcaster()  # Broadcast coordinate transforms
    # Publish the message to /tf topic
    # robot's distance from the origin
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, 0),  # translation
                     # rotation quaternion_from_euler: convert Euler angles to quaternion
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),  # timestamp
                     '/%s' % robotname,  # Publish robotname's translation and rotation to "world"
                     '/world')


if name == 'main':
    rospy.init_node('tf_broadcaster', anonymous=True)
    # Get the value of robot from the parameter server
    robotname = rospy.get_param('~robotname')
    rospy.Subscriber('/%s/odom' % robotname,  # The topic to subscribe to: /robot1/odom
                     nav_msgs.msg.Odometry,  # The type of message to subscribe to
                     handle_robot_pose,  # The callback function
                     robotname)  # Arguments to be passed to the callback function
    rospy.spin()  # Keep the node running until it's stopped
