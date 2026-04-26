#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将实现一个话题发布节点pub_image，发布/soccer_image话题，消息类型为sensor_msgs::Image，发布频率为10Hz

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def main():
    # ROS 节点初始化
    rospy.init_node('pub_image')

    # 创建发布者Publisher, 发布名为/soccer_image的话题，消息类型为sensor_msgs::Image，队列长度10
    image_pub = rospy.Publisher("soccer_image", Image, queue_size=10)

    # 设置循环的频率为10Hz
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # 初始化消息类型
        bridge = CvBridge()
        image_path = "/home/sunzj/Robot/catkin_ws/assets/image.jpg" # 需要修改路径
        image = cv2.imread(image_path)

        # 发布消息
        image_pub.publish(bridge.cv2_to_imgmsg(image,'bgr8'))
        print("image published")

        # 按照循环频率延时
        rate.sleep()

if __name__ == '__main__':
    main()