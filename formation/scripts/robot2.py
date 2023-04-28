#!/usr/bin/env python
# coding=utf-8

import rospy
import math
import tf
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion

if name == 'main':
    rospy.init_node('tf_listener', anonymous=True)
    # TransformListener starts receiving tf broadcast information as soon as it is created and can cache it up to 10 seconds.
    listener = tf.TransformListener()

    '''
    # Set the initial coordinates of robot2
    robot2_start = rospy.Publisher('robot2/odom', nav_msgs/Odometry, queue_size=1)
    msg.pose.pose.position.x = 0
    msg.pose.pose.position.y = 0
    msg.pose.pose.position.z = 0
    msg.pose.pose.orientation.x = 0
    msg.pose.pose.orientation.y = 0
    msg.pose.pose.orientation.z = 0
    msg.pose.pose.orientation.w = 0
    robot2_start.publish(msg) # Pass the requested parameters to the initial position of robot2
    '''

    # The first argument of the Publisher function is the topic name, the second argument is the data type, and the last one is the buffer size.
    turtle_vel = rospy.Publisher(
        '/ares2/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

    rate = rospy.Rate(10.0)  # Loop execution with an update frequency of 10 Hz
    while not rospy.is_shutdown():
        try:
            # Get the posture information (translation and rotation) of robot1 relative to robot2, with robot2 as the coordinate origin
            # Lookup the relative tf and return the translation and rotation. turtle2 follows turtle1 transformation.
            (trans, rot) = listener.lookupTransform(
                '/ares2', '/ares1', rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        err_msg = "Robot2 trans: " + \
            str(trans[0]) + "Robot2 rot: " + str(trans[1])
        rospy.loginfo(err_msg)

        # Translation transformation, calculating the linear velocity to reach robot1
        linear = math.sqrt((trans[0] - 0.866) ** 2 + (trans[1] + 0.5) ** 2)
        # TODO angular Calculate ERROR!
        # Angular transformation, calculating the angular velocity to reach robot1. atan2(double y, double x) returns the azimuth angle from the origin to point (x,y), that is, the angle between the x axis.
        angular = math.atan2((trans[1] + 0.5), (trans[0] - 0.866))

        info_msg = "Robot2 angular: " + \
            str(angular) + "Robot2 linear: " + str(linear)
        rospy.loginfo(info_msg)

        msg = geometry_msgs.msg.Twist()
        # msg.linear.x = linear   # Translation transformation
        # msg.angular.z = angular # Angular transformation

        if linear > 0.035:  # If robot1 does not move, but the numerical value has a slight drift, robot2 will not be allowed to move.
            msg.linear.x = linear    # Translation transformation
            msg.angular.z = angular  # Angular transformation
        else:

            msg.linear.x = 0
            msg.angular.z = 0

        # Publish new coordinates to the /robot2/cmd_vel topic (that is, robot2 moves according to the data from /robot2/cmd_vel)
        turtle_vel.publish(msg)
        rate.sleep()  # Executes at a fixed frequency.
