#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist

def move_straight(pub, linear_speed, duration):
    """直线前进指定时间"""
    twist = Twist()
    twist.linear.x = linear_speed
    twist.angular.z = 0.0

    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
        pub.publish(twist)
        rospy.Rate(1000).sleep()

def turn(pub, angular_speed, duration):
    """旋转指定时间"""
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = angular_speed

    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time).to_sec() < duration and not rospy.is_shutdown():
        pub.publish(twist)
        rospy.Rate(1000).sleep()

def stop(pub):
    """停止运动"""
    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)
    rospy.sleep(0.3)  # 等待机器人完全停止

if __name__ == '__main__':
    rospy.init_node('hexagon_control')

    # 运动参数
    linear_speed = 2.0      # 线速度 m/s
    angular_speed = 1.0     # 角速度 rad/s
    side_length = 2.0       # 每条边长度 m
    turn_angle = math.pi / 3  # 60度 = π/3 弧度

    # 计算时间
    move_time = side_length / linear_speed
    turn_time = turn_angle / angular_speed

    # 创建发布者
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.sleep(1)  # 等待发布者连接

    rospy.loginfo("Starting hexagon trajectory...")
    rospy.loginfo("Side length: %.2f m, Move time: %.2f s", side_length, move_time)
    rospy.loginfo("Turn angle: %.2f rad, Turn time: %.2f s", turn_angle, turn_time)

    # 绘制正六边形
    for i in range(6):
        rospy.loginfo("Drawing side %d/6", i + 1)

        # 直线前进
        move_straight(pub, linear_speed, move_time)
        stop(pub)

        # 转向60度
        turn(pub, angular_speed, turn_time)
        stop(pub)

    rospy.loginfo("Hexagon completed!")
    stop(pub)
