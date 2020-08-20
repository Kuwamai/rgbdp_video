#! /usr/bin/env python
# -*- coding: utf-8 -*-
 
import rospy
import cv2
import sys
import message_filters
import tf2_ros
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

class depth_converter:
    def __init__(self):
        rospy.init_node('depth_converter', anonymous=True)
        self.bridge = CvBridge()
        sub_rgb = message_filters.Subscriber("camera/color/image_raw",Image)
        sub_depth = message_filters.Subscriber("camera/aligned_depth_to_color/image_raw",Image)
        self.mf = message_filters.ApproximateTimeSynchronizer([sub_rgb, sub_depth], 100, 10.0)
        self.mf.registerCallback(self.ImageCallback)

        self.pub_depth = rospy.Publisher("converted_depth", Image, queue_size=10)
        self.pub_image = rospy.Publisher("converted_image", Image, queue_size=10)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
 
    def ImageCallback(self, rgb_data , depth_data):
        try:
            color_image = self.bridge.imgmsg_to_cv2(rgb_data, "passthrough")
            depth_image = self.bridge.imgmsg_to_cv2(depth_data, "passthrough")
        except CvBridgeError, e:
            rospy.logerr(e)

        # 映像取得時のカメラ位置計算
        try:
            trans = self.tfBuffer.lookup_transform("map", "camera_color_frame", rospy.Time(0))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("TF error")

        resize_scale = 0.5

        # Publish color_image
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB) 
        h, w, c = color_image.shape
        color_image = cv2.resize(color_image, (int(w * resize_scale), int(h * resize_scale)))
        msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
        self.pub_image.publish(msg)
 
        # Publish depth_image
        depth_array = np.array(depth_image)
        max_distance = 1300
        min_distance = 150
        depth_array[np.isnan(depth_array)] = max_distance
        depth_array = depth_array.astype(np.float32)
        cv2.imshow("Origin Image", depth_image)
        ret, depth_array = cv2.threshold(depth_array, max_distance, max_distance, cv2.THRESH_TRUNC)
        ret, depth_array = cv2.threshold(depth_array, min_distance, max_distance, cv2.THRESH_TOZERO)
        depth_array = (depth_array - min_distance) / (max_distance - min_distance) * 360
        depth_hsv = np.dstack([depth_array, np.ones_like(depth_array), np.ones_like(depth_array)])
        depth_rgb = cv2.cvtColor(depth_hsv, cv2.COLOR_HSV2RGB)

        h, w, d = depth_rgb.shape
        depth_rgb = cv2.resize(depth_rgb, (int(w * resize_scale), int(h * resize_scale)))

        depth_rgb = np.uint8(np.round(depth_rgb * 255))
        msg = self.bridge.cv2_to_imgmsg(depth_rgb, encoding = "rgb8")
        self.pub_depth.publish(msg)
 
if __name__ == '__main__':
    try:
        de = depth_converter()
        rospy.spin()
    except rospy.ROSInterruptException: pass
