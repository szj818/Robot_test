#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.init_node('circle_control')
    pub  = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10 Hz

    while  not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x  = 1.0  # Move forward at 1.0 m/s
        twist.angular.z = 1.0  # Rotate at 1.0 rad/s
        pub.publish(twist)
        rospy.loginfo("Turtle is drawing a circle...")
        # print("gogogo")
        # rospy.loginfo("Publishing Twist: linear.x=%.2f, angular.z=%.2f", twist.linear.x, twist.angular.z)
        rate.sleep()