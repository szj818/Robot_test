#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将实现一个话题接收节点sub_image，订阅/soccer_image话题，消息类型为sensor_msgs::Image

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def callback(data):
    # 处理接收到的消息
    print("image received")
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.imshow("soccer", image)
    cv2.waitKey(0)

def main():
    # ROS节点初始化
    rospy.init_node("sub_image")

    # 创建订阅者Subscriber，订阅名为/soccer_image的topic，消息类型为sensor_msgs::Image，注册回掉函数callback
    image_sub = rospy.Subscriber("/soccer_image", Image, callback)

    # 循环等待回调函数
    rospy.spin()

if __name__ == "__main__":
    main()