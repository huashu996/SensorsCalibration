#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
import numpy as np
import pcl
import os
import cv2
from sensor_msgs import point_cloud2
import time
class ImagePointCloudSaverNode:
    def __init__(self):
        rospy.init_node('image_pointcloud_saver_node')
        #self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.pointcloud_sub = rospy.Subscriber('/livox/lidar', PointCloud2, self.pointcloud_callback)
        self.cv_bridge = CvBridge()
        self.save_directory = rospy.get_param('~save_directory', './data/image') 
        self.save_directory2 = rospy.get_param('~save_directory', './data/pcd') 
        self.point_clouds = []
        self.save_interval = rospy.Duration.from_sec(0.1)  # 每隔1秒保存一次点云
        self.last_save_time = rospy.Time.now()
        self.file_counter = 0
        self.points = []

    def image_callback(self, msg):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr('Failed to convert image message to CV2: {}'.format(e))
            return

        file_name = '{:05d}.png'.format(self.file_counter)
        file_path = os.path.join(self.save_directory, file_name)
        cv2.imwrite(file_path, cv_image)
        rospy.loginfo('Saved image as {}'.format(file_path))
        self.file_counter += 1

    def pointcloud_callback(self, msg):
        cloud = pc2.read_points(msg, field_names=("x", "y", "z" , "intensity"), skip_nans=True)
        for p in cloud:
            self.points.append([p[0], p[1], p[2], p[3]])
        current_time = rospy.Time.now()
        # 计算时间间隔
        time_diff = current_time - self.last_save_time

        # 保存点云的时间间隔达到指定的间隔时进行保存
        if time_diff >= self.save_interval:
            points_array = np.array(self.points, dtype=np.float32)
            pcl_cloud = pcl.PointCloud_PointXYZI()
            pcl_cloud.from_array(points_array)
            #pcl_cloud.from_list(self.points)
            self.last_save_time = current_time
            # 将点云保存为PCD文件
            file_name = '{:05d}.pcd'.format(self.file_counter)
            file_path = os.path.join(self.save_directory2, file_name)
            pcl.save(pcl_cloud, file_path)
            rospy.loginfo("Saved point cloud to %s", file_path)
            # 保存图片
            image_msg = rospy.wait_for_message('/usb_cam/image_raw', Image)
            self.image_callback(image_msg)
            self.points = []

if __name__ == '__main__':
    try:
        image_pointcloud_saver = ImagePointCloudSaverNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
