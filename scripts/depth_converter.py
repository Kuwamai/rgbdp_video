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
from tf.transformations import quaternion_matrix

class depth_converter:
    def __init__(self):
        rospy.init_node('depth_converter', anonymous=True)

        self.pos_lim = rospy.get_param("/depth_converter/pos_lim")
        self.z_offset = rospy.get_param("/depth_converter/z_offset")
        self.max_distance = rospy.get_param("/depth_converter/max_distance")
        self.min_distance = rospy.get_param("/depth_converter/min_distance")

        self.bridge = CvBridge()
        sub_rgb = message_filters.Subscriber("camera/color/image_raw",Image)
        sub_depth = message_filters.Subscriber("camera/aligned_depth_to_color/image_raw",Image)
        self.mf = message_filters.ApproximateTimeSynchronizer([sub_rgb, sub_depth], 100, 10.0)
        self.mf.registerCallback(self.ImageCallback)

        self.pub_image = rospy.Publisher("rgbdp_image", Image, queue_size=10)

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
            pos_x_image = np.full((40, 40, 3), [(trans.transform.translation.x / self.pos_lim + 0.5) * 360, 1, 1])
            pos_y_image = np.full((40, 40, 3), [(trans.transform.translation.y / self.pos_lim + 0.5) * 360, 1, 1])
            pos_z_image = np.full((40, 40, 3), [((trans.transform.translation.z + self.z_offset) / self.pos_lim + 0.5) * 360, 1, 1])
            matrix = quaternion_matrix([trans.transform.rotation.w,
                                        trans.transform.rotation.x,
                                        trans.transform.rotation.y,
                                        trans.transform.rotation.z])
            pose_list = [pos_x_image, pos_y_image, pos_z_image]

            for i in range(3):
                for j in range(3):
                    pose_list.append(np.full((40, 40, 3), [(matrix[i][j] / 2 + 0.5) * 360, 1, 1]))

            pose_hsv = np.vstack(pose_list)
            pose_hsv = pose_hsv.astype(np.float32)
            pose_rgb = cv2.cvtColor(pose_hsv, cv2.COLOR_HSV2RGB)
            pose_rgb = np.uint8(np.round(pose_rgb * 255))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("TF error")
            return 0

        resize_scale = 1

        # color_image
        #color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB) 
        h, w, c = color_image.shape
 
        # depth_image
        depth_array = np.array(depth_image)
        max_distance = 3000
        min_distance = 100
        depth_array = depth_array.astype(np.float32)
        v_array = s_array = np.ones_like(depth_array)
        v_array[depth_array == 0] = 0
        cv2.imshow("Origin Image", depth_image)
        ret, depth_array = cv2.threshold(depth_array, self.max_distance, self.max_distance, cv2.THRESH_TRUNC)
        ret, depth_array = cv2.threshold(depth_array, self.min_distance, self.max_distance, cv2.THRESH_TOZERO)
        depth_array = depth_array / (self.max_distance - self.min_distance) * 360
        depth_hsv = np.dstack([depth_array, s_array, v_array])
        depth_rgb = cv2.cvtColor(depth_hsv, cv2.COLOR_HSV2RGB)
        depth_rgb = cv2.resize(depth_rgb, (int(w), int(h)))
        depth_rgb = np.uint8(np.round(depth_rgb * 255))
        side_by_side = cv2.hconcat([pose_rgb, color_image, depth_rgb])
        cv2.imshow('image', cv2.cvtColor(side_by_side, cv2.COLOR_BGR2RGB))
        cv2.waitKey(1)
        msg = self.bridge.cv2_to_imgmsg(side_by_side, encoding = "rgb8")
        self.pub_image.publish(msg)
 
if __name__ == '__main__':
    try:
        de = depth_converter()
        rospy.spin()
    except rospy.ROSInterruptException: pass
