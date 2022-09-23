#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import tf2_ros
import math
import copy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from rospy.exceptions import ROSException
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped, Quaternion

class EstimatePosture():
    def __init__(self):
        try:
            self.mirror_distance   = rospy.get_param("/lidar_with_mirror/mirror_distance"  , 0.1         )
            self.mirror_roll_angle = rospy.get_param("/lidar_with_mirror/mirror_roll_angle",  math.pi  /4)
            self.scan_front_begin  = rospy.get_param("/lidar_with_mirror/scan_front_begin" , -math.pi  /3 + 0.01)
            self.scan_front_end    = rospy.get_param("/lidar_with_mirror/scan_front_end"   ,  math.pi  /3 - 0.01)
            self.scan_left_begin   = rospy.get_param("/lidar_with_mirror/scan_left_begin"  ,  math.pi  /3 + 0.01)
            self.scan_left_end     = rospy.get_param("/lidar_with_mirror/scan_left_end"    ,  math.pi*2/3       )
            self.scan_right_begin  = rospy.get_param("/lidar_with_mirror/scan_right_begin" , -math.pi*2/3       )
            self.scan_right_end    = rospy.get_param("/lidar_with_mirror/scan_right_end"   , -math.pi  /3 - 0.01)
        except ROSException:
            rospy.loginfo("param load error")

        self.sub_scan = rospy.Subscriber('/lidar_with_mirror_scan', LaserScan, self.callbackScan)
        self.pub_scan_front = rospy.Publisher('scan_front', LaserScan, queue_size=1)
        self.lp = lg.LaserProjection()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.pub_transformed_pc_front  = rospy.Publisher("transformed_point_cloud_front" , PointCloud2, queue_size=1)

    def trim_scan_data(self, data, start_angle_rad, end_angle_rad):
        trim_data = copy.deepcopy(data)
        start_angle_rad = max(start_angle_rad, data.angle_min)
        end_angle_rad   = min(end_angle_rad  , data.angle_max)
        start = int((start_angle_rad - data.angle_min) / data.angle_increment)
        end   = int((end_angle_rad   - data.angle_min) / data.angle_increment)
        trim_data.ranges = data.ranges[start:end]
        trim_data.intensities = data.intensities[start:end]
        trim_data.angle_min = start_angle_rad
        trim_data.angle_max = end_angle_rad
        return trim_data

    def find_plane(self, r):
        c = np.mean(r, axis=0)
        r0 = r - c
        u, s, v = np.linalg.svd(r0)
        nv = v[-1, :]
        ds = np.dot(r, nv)
        return np.r_[nv, -np.mean(ds)]

    def callbackScan(self, data):
        # Trimming scanned data within a specified range
        front_data = self.trim_scan_data(data, self.scan_front_begin, self.scan_front_end)
        right_data = self.trim_scan_data(data, self.scan_right_begin, self.scan_right_end)
        left_data  = self.trim_scan_data(data, self.scan_left_begin , self.scan_left_end )
        right_data.header.frame_id = "lidar_with_mirror_right_link"
        left_data.header.frame_id  = "lidar_with_mirror_left_link"

        # Converting to lidar_with_mirror_center_link coordinate
        right_pc = self.lp.projectLaser(right_data)
        left_pc  = self.lp.projectLaser(left_data)
        try:
            trans_right = self.tfBuffer.lookup_transform('lidar_with_mirror_center_link', right_data.header.frame_id, data.header.stamp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        try:
            trans_left = self.tfBuffer.lookup_transform('lidar_with_mirror_center_link', left_data.header.frame_id, data.header.stamp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        try:
            trans = self.tfBuffer.lookup_transform('odom', 'lidar_with_mirror_center_link', data.header.stamp)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        right_pc_base = do_transform_cloud(right_pc, trans_right)
        left_pc_base  = do_transform_cloud(left_pc , trans_left )

        # Calculate planes from scan data reflected by left and right mirrors
        pc = []
        for p in pc2.read_points(right_pc_base, skip_nans=True, field_names=("x", "y", "z")):
            pc.append(p)
        for p in pc2.read_points(left_pc_base, skip_nans=True, field_names=("x", "y", "z")):
            pc.append(p)
        coef = self.find_plane(pc)
        
        # Calculating LiDAR position with respect to the ground
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = data.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "lidar_with_mirror_estimated_center_link"
        t.transform.translation.x = trans.transform.translation.x
        t.transform.translation.y = trans.transform.translation.y
        height = math.fabs(coef[3])/math.sqrt(coef[0]**2+coef[1]**2+coef[2]**2)
        t.transform.translation.z = height
        roll  = math.atan(coef[1]/coef[2])
        pitch = math.atan(-coef[0]/coef[2]*math.cos(roll))
        q = tf.transformations.quaternion_from_euler(roll, pitch, 0)
        t.transform.rotation = Quaternion(q[0], q[1], q[2], q[3])
        br.sendTransform(t)

        # Calculate ground height based on estimated LiDAR position
        front_data.header.frame_id = "lidar_with_mirror_estimated_center_link"
        self.pub_scan_front.publish(front_data)

if __name__ == '__main__':
    rospy.init_node('estimate_posture')
    node = EstimatePosture()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
