#!/usr/bin/env python
#coding=utf-8

import rospy
import math
import tf
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion


if __name__ == '__main__':
    rospy.init_node('tf_listener', anonymous=True)

    listener = tf.TransformListener()  # Create a TransformListener that listens to tf broadcasts, caches up to 10 seconds.

    '''
    # Set the initial coordinates of robot3
    robot3_start = rospy.Publisher('robot3/odom', nav_msgs/Odometry, queue_size=1)
    msg.pose.pose.position.x = 0
    msg.pose.pose.position.y = 0
    msg.pose.pose.position.z = 0
    msg.pose.pose.orientation.x = 0
    msg.pose.pose.orientation.y = 0
    msg.pose.pose.orientation.z = 0
    msg.pose.pose.orientation.w = 0
    robot3_start.publish(msg)  # Set the initial position of robot3 to the requested parameters.
    '''

    # The first parameter of the Publisher function is the topic name, the second parameter is the data type, which is the msg we defined, and the last parameter is the buffer size.
    turtle_vel = rospy.Publisher('/ares3/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)

    rate = rospy.Rate(10.0)  # Execute the loop at a frequency of 10hz.
    while not rospy.is_shutdown():
        try:
            # Get the pose information (translation and rotation) of robot1 relative to robot2, which uses robot2 as the origin of the coordinate system.
            (trans, rot) = listener.lookupTransform('/ares3', '/ares1', rospy.Time()) # View the relative tf, return translation and rotation, which means that turtle2 moves with turtle1.
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        # TODO angular Calculate ERROR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        angular = math.atan2((trans[1]-0.5), (trans[0]-0.866))  # Calculate the angular velocity of robot2 to move towards robot1. atan2 (double y, double x) returns the azimuth angle from the origin to the point (x, y), that is, the angle between it and the x-axis.
        linear = math.sqrt((trans[0]-0.866) ** 2 + (trans[1]-0.5) ** 2)  # Calculate the linear velocity of robot2 to move towards robot1.

        msg = geometry_msgs.msg.Twist()
        #msg.linear.x = linear  # Linear transformation
        #msg.angular.z = angular # Angular transformation
        
        if linear > 0.035:  # If robot1 does not move, but the values ​​have slight drift, do not let robot2 move.
            msg.linear.x = linear  # Linear transformation
            msg.angular.z = angular  # Angular transformation
        else:
            # err_msg = "Robot2 angular: "+ str(angular) + "Robot2 linear: " + str(linear)
            # rospy.loginfo(err_msg)
            msg.linear.x = 0
            msg.angular.z = 0

        turtle_vel.publish(msg)  # Publish new coordinates to the /robot2/cmd_vel topic (i.e., robot2 moves based on the data of /robot2/cmd_vel).
        rate.sleep()  # Execute at a fixed frequency.
