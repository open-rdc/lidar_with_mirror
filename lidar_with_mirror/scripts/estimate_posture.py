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
            self.scan_front_begin  = rospy.get_param("/lidar_with_mirror/scan_front_begin" , -0.70 )
            self.scan_front_end    = rospy.get_param("/lidar_with_mirror/scan_front_end"   ,  0.70 )
            self.scan_left_begin   = rospy.get_param("/lidar_with_mirror/scan_left_begin"  , -1.80 ) #-2.00
            self.scan_left_end     = rospy.get_param("/lidar_with_mirror/scan_left_end"    , -1.05 )
            self.scan_right_begin  = rospy.get_param("/lidar_with_mirror/scan_right_begin" ,  1.14 )
            self.scan_right_end    = rospy.get_param("/lidar_with_mirror/scan_right_end"   ,  1.80 ) #2.09
        except ROSException:
            rospy.loginfo("param load error")

        self.sub_scan = rospy.Subscriber('/lidar_with_mirror_scan', LaserScan, self.callbackScan)
        self.pub_scan_front = rospy.Publisher('scan_front', LaserScan, queue_size=1)
        self.pub_pc_right = rospy.Publisher("point_cloud_right", PointCloud2, queue_size=1)
        self.pub_pc_left  = rospy.Publisher("point_cloud_left" , PointCloud2, queue_size=1)
        self.lp = lg.LaserProjection()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.height, self.roll, self.pitch = 0, 0 ,0

        # for hough conversion
        self.no_bin_the, self.no_bin_rho = 30, 30
        sin_l, cos_l = [], []
        step = 2 * math.pi / self.no_bin_the
        for t in np.arange(step/2, 2 * math.pi, step):
            sin_l.append(math.sin(t))
            cos_l.append(math.cos(t))
        self.sin_t, self.cos_t = np.array(sin_l), np.array(cos_l)
        self.max_rho = 2


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
        return np.r_[nv, -np.mean(ds)], s[-1]

    def hough(self, list_x, list_y):
        bin = np.zeros((self.no_bin_the, self.no_bin_rho))
        for x, y in zip(list_x, list_y):
            rho = x * self.cos_t + y * self.sin_t
            for t in range(self.no_bin_the):
                if (rho[t] > self.max_rho):
                    print("rho > max_rho rho: "+str(rho[t])+"max_rho: "+str(self.max_rho))
                    break
                bin[t][int((rho[t]/self.max_rho*self.no_bin_rho+self.no_bin_rho)/2)] += 1
        max_angle = bin.argmax() // self.no_bin_rho
        max_dist = bin.argmax() % self.no_bin_rho
        return -self.cos_t[max_angle]/self.sin_t[max_angle], 1/self.sin_t[max_angle]*(max_dist-self.no_bin_rho/2)/(self.no_bin_rho/2)*self.max_rho
    def dist_from_line(self, x, y, a, b):
        return abs(a * x - y + b) / math.sqrt(a ** 2 + 1)
    def line_detect(self, list_x, list_y, a, b, threshold):
        res_x, res_y = [], []
        for x, y in zip(list_x, list_y):
            if self.dist_from_line(x, y, a, b) < threshold:
                res_x.append(x)
                res_y.append(y)
        return res_x, res_y
    def delete_outliers(self, pc):
        list_x, list_y = [], []
        for p in pc2.read_points(pc, skip_nans=True, field_names=("x", "y")):
            list_x.append(p[0])
            list_y.append(p[1])
        a, b = self.hough(list_x, list_y)
        line_x, line_y = self.line_detect(list_x, list_y, a, b, 0.1)
        points = []
        for x, y in zip(line_x, line_y):
            points.append([x, y, 0, 0])
        ret_pc = pc2.create_cloud(pc.header, pc.fields, points)
        return ret_pc

    def callbackScan(self, data):
        # Trimming scanned data within a specified range
        front_data = self.trim_scan_data(data, self.scan_front_begin, self.scan_front_end)
        left_data  = self.trim_scan_data(data, self.scan_left_begin , self.scan_left_end )
        right_data = self.trim_scan_data(data, self.scan_right_begin, self.scan_right_end)
        right_data.header.frame_id = "lidar_with_mirror_right_link"
        left_data.header.frame_id  = "lidar_with_mirror_left_link"

        # Converting to lidar_with_mirror_center_link coordinate
        right_pc = self.lp.projectLaser(right_data)
        left_pc  = self.lp.projectLaser(left_data)
        #right_pc = self.delete_outliers(right_pc0)
        #left_pc = self.delete_outliers(left_pc0)
        trans_right = self.tfBuffer.lookup_transform('lidar_with_mirror_center_link', right_data.header.frame_id, rospy.Time())
        trans_left = self.tfBuffer.lookup_transform('lidar_with_mirror_center_link', left_data.header.frame_id, rospy.Time())
        trans = self.tfBuffer.lookup_transform('base_link', 'lidar_with_mirror_center_link',  rospy.Time())
        right_pc_base = do_transform_cloud(right_pc, trans_right)
        left_pc_base  = do_transform_cloud(left_pc , trans_left )

        # Calculate planes from scan data reflected by left and right mirrors
        pc = []
        for p in pc2.read_points(right_pc_base, skip_nans=True, field_names=("x", "y", "z")):
            pc.append(p)
        for p in pc2.read_points(left_pc_base, skip_nans=True, field_names=("x", "y", "z")):
            pc.append(p)
        coef, rmd = self.find_plane(pc)
        print(coef, rmd)
        if rmd < 0.15: #adjust
            self.height = math.fabs(coef[3])/math.sqrt(coef[0]**2+coef[1]**2+coef[2]**2)
            self.roll  = math.atan(coef[1]/coef[2])+math.pi
            self.pitch = math.atan(-coef[0]/coef[2]*math.cos(self.roll))

        # Calculating LiDAR position with respect to the ground
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"
        t.child_frame_id = "lidar_with_mirror_estimated_center_link"
        t.transform.translation.x = trans.transform.translation.x
        t.transform.translation.y = trans.transform.translation.y
        t.transform.translation.z = self.height
        q = tf.transformations.quaternion_from_euler(self.roll, self.pitch, 0)
        t.transform.rotation = Quaternion(q[0], q[1], q[2], q[3])
        br.sendTransform(t)

        # Calculate ground height based on estimated LiDAR position
        front_data.header.frame_id = "lidar_with_mirror_estimated_center_link"
        self.pub_scan_front.publish(front_data)
        left_pc_base.header.frame_id = "lidar_with_mirror_estimated_center_link"
        right_pc_base.header.frame_id = "lidar_with_mirror_estimated_center_link"
        self.pub_pc_left.publish(left_pc_base)
        self.pub_pc_right.publish(right_pc_base)

if __name__ == '__main__':
    rospy.init_node('estimate_posture')
    node = EstimatePosture()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
