#!/usr/bin/env python3
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
from statistics import mean, median,variance,stdev

class EstimatePosture():
    def __init__(self):
        try:
            self.mirror_distance   = rospy.get_param("/lidar_with_mirror/mirror_distance"  , 0.1         )
            self.mirror_roll_angle = rospy.get_param("/lidar_with_mirror/mirror_roll_angle",  math.pi  /4)
            self.scan_front_begin  = rospy.get_param("/lidar_with_mirror/scan_front_begin" , -math.pi  /3 + 0.01)
            self.scan_front_end    = rospy.get_param("/lidar_with_mirror/scan_front_end"   ,  math.pi  /3 - 0.01)
            self.scan_left_begin   = rospy.get_param("/lidar_with_mirror/scan_left_begin"  , -math.pi*2/3       )
            self.scan_left_end     = rospy.get_param("/lidar_with_mirror/scan_left_end"    , -math.pi  /3 - 0.01)
            self.scan_right_begin  = rospy.get_param("/lidar_with_mirror/scan_right_begin" ,  math.pi  /3 + 0.01)
            self.scan_right_end    = rospy.get_param("/lidar_with_mirror/scan_right_end"   ,  math.pi*2/3       )
        except ROSException:
            rospy.loginfo("param load error")

        self.sub_scan = rospy.Subscriber('/lidar_with_mirror_scan', LaserScan, self.callbackScan)
        self.pub_scan_front = rospy.Publisher('front_scan', LaserScan, queue_size=1)
        self.pub_right_scan = rospy.Publisher('right_scan', LaserScan, queue_size=1)
        self.pub_left_scan  = rospy.Publisher('left_scan' , LaserScan, queue_size=1)
        self.lp = lg.LaserProjection()
        self.pub_pc_right = rospy.Publisher("point_cloud_right", PointCloud2, queue_size=1)
        self.pub_pc_left  = rospy.Publisher("point_cloud_left" , PointCloud2, queue_size=1)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.pub_transformed_pc_front  = rospy.Publisher("transformed_point_cloud_front" , PointCloud2, queue_size=1)

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
        self.pub_pc_right.publish(right_pc_base)
        self.pub_pc_left.publish(left_pc_base)

        # Calculate planes from scan data reflected by left and right mirrors
        pc = []
        for p in pc2.read_points(right_pc_base, skip_nans=True, field_names=("x", "y", "z")):
            pc.append(p)
        for p in pc2.read_points(left_pc_base, skip_nans=True, field_names=("x", "y", "z")):
            pc.append(p)
        coef = self.find_plane(pc)
        
        #print(coef)
        #min_val = 0.0
        #max_val = 0.0
        #for x, y, z in pc:
        #    val = coef[0]*x+coef[1]*y+coef[2]*z+coef[3]
        #    min_val = min(min_val, val)
        #    max_val = max(max_val, val)
        #print("min: " + str(min_val) + ", max: " + str(max_val))

        # 平面からLiDARの座標を計算
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = data.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "lidar_with_mirror_estimated_link"
        t.transform.translation.x = trans.transform.translation.x
        t.transform.translation.y = trans.transform.translation.y
        height = math.fabs(coef[3])/math.sqrt(coef[0]**2+coef[1]**2+coef[2]**2)
        t.transform.translation.z = height
        roll  = math.atan(coef[1]/coef[2])+math.pi
        pitch = math.atan(-coef[0]/coef[2]*math.cos(roll))
        q = tf.transformations.quaternion_from_euler(roll, pitch, 0)
        t.transform.rotation = Quaternion(q[0], q[1], q[2], q[3])
        br.sendTransform(t)

        # 真値の計算（比較のため）
        quaternion = trans.transform.rotation
        h = trans.transform.translation.z
        e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        #print("roll_pitch_height_estimated, "+str(roll)+", "+str(pitch)+", "+str(height)+", ground truth, "+str(e[0])+", "+str(e[1])+", "+str(h))

        #LiDARの座標から正面のLiDARの位置を検出
        front_pc = self.lp.projectLaser(front_data)
        front_pc.header.frame_id = "lidar_with_mirror_estimated_link"
        self.pub_transformed_pc_front.publish(front_pc)
        pc_base  = do_transform_cloud(front_pc , t)
        pc = []
        for p in pc2.read_points(pc_base, skip_nans=True, field_names=("x", "y", "z")):
            pc.append(p[2])
        print(str(e[0])+", "+str(e[1])+", "+str(mean(pc))+", "+str(stdev(pc)))
        #right_pc_base.header.frame_id = "lidar_with_mirror_estimated_link"
        #left_pc_base.header.frame_id = "lidar_with_mirror_estimated_link"
        #self.pub_pc_right.publish(right_pc_base)
        #self.pub_pc_left.publish(left_pc_base)

#   trim_scan_data
#   スキャンデータから取得角度に基づいてターゲット範囲のスキャンデータを取り出す
#   引数：trim_scan_data(LaserScan data, float start_angle_rad, float end_angle_rad)
#       data              LiDARのスキャン生データ
#       start_angle_rad   ターゲットの開始角度[rad]
#       end_angle_rad     ターゲットの終了角度[rad]
#   返り値: LaserScan trim_data

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

if __name__ == '__main__':
    rospy.init_node('estimate_posture')
    node = EstimatePosture()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)

