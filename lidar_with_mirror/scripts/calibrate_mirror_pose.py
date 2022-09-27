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
from geometry_msgs.msg import TransformStamped, Quaternion, Point
from visualization_msgs.msg import Marker

class CalibrateMirrorPose():
    def __init__(self):
        try:
            self.mirror_distance   = rospy.get_param("/lidar_with_mirror/mirror_distance"  , 0.1         )
            self.mirror_roll_angle = rospy.get_param("/lidar_with_mirror/mirror_roll_angle",  math.pi  /4)
            self.scan_front_begin  = rospy.get_param("/lidar_with_mirror/scan_front_begin" , -0.70 ) #-math.pi  /3 + 0.01
            self.scan_front_end    = rospy.get_param("/lidar_with_mirror/scan_front_end"   ,  0.70 ) # math.pi  /3 - 0.01
            self.scan_left_begin   = rospy.get_param("/lidar_with_mirror/scan_left_begin"  , -2.00 ) # math.pi  /3 + 0.01)
            self.scan_left_end     = rospy.get_param("/lidar_with_mirror/scan_left_end"    , -1.05 ) # math.pi*2/3
            self.scan_right_begin  = rospy.get_param("/lidar_with_mirror/scan_right_begin" ,  1.14 ) #-math.pi*2/3
            self.scan_right_end    = rospy.get_param("/lidar_with_mirror/scan_right_end"   ,  2.09 ) #-math.pi  /3 - 0.01
            self.obstacle_height   = rospy.get_param("/lidar_with_mirror/obstacle_height"  ,  0.10 )
        except ROSException:
            rospy.loginfo("param load error")

        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.callbackScan)
        self.pub_scan_front = rospy.Publisher('scan_front', LaserScan, queue_size=1)
        self.pub_scan_right = rospy.Publisher('scan_right', LaserScan, queue_size=1)
        self.pub_scan_left  = rospy.Publisher('scan_left' , LaserScan, queue_size=1)
        self.maker_pub = rospy.Publisher("marker_pub", Marker, queue_size=10)
        self.marker = Marker()
        self.marker.header.frame_id = "laser"
        self.marker.ns = "basic_shapes"
        self.marker.id = 0
        self.marker.action = Marker.ADD
        self.marker.type = Marker.LINE_LIST
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0
        self.marker.scale.x = 0.01
        self.marker.scale.y = 0.01
        self.marker.scale.z = 0.01
        self.marker.pose.orientation.w = 1.0
        self.lp = lg.LaserProjection()
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

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

    def dist_from_line(self, x, y, a, b):
        return abs(a * x - y + b) / math.sqrt(a ** 2 + 1)

    def obstacle_detect(self, list_x, list_y, a, b, threshold):
        res_x, res_y = [], []
        for x, y in zip(list_x, list_y):
            if self.dist_from_line(x, y, a, b) > threshold and math.sqrt(x ** 2 + y ** 2) > 0.1:
                res_x.append(x)
                res_y.append(y)
        if len(res_x) >= 4:
            del res_x[0:1]
            del res_y[0:1]
            del res_x[-1]
            del res_x[-1]
            del res_y[-1]
            del res_y[-1]
        return res_x, res_y
    def line_detect(self, list_x, list_y, a, b, threshold):
        res_x, res_y = [], []
        for x, y in zip(list_x, list_y):
            if self.dist_from_line(x, y, a, b) < threshold:
                res_x.append(x)
                res_y.append(y)
        return res_x, res_y

    def callbackScan(self, data):
        # Trimming scanned data within a specified range
        front_data = self.trim_scan_data(data, self.scan_front_begin, self.scan_front_end)
        left_data  = self.trim_scan_data(data, self.scan_left_begin , self.scan_left_end )
        right_data = self.trim_scan_data(data, self.scan_right_begin, self.scan_right_end)
        self.pub_scan_front.publish(front_data)
        self.pub_scan_left.publish(left_data)
        self.pub_scan_right.publish(right_data)

        # Calculate posture of sensor
        na = int(0.1 / data.angle_increment)
        front_pc = self.lp.projectLaser(front_data)
        front_x, front_y = [], []
        for p in pc2.read_points(front_pc, skip_nans=True, field_names=("x", "y", "z")):
            front_x.append(p[0])
            front_y.append(p[1])
        n = len(front_x) - 1
        front_a0, front_b0 = np.polyfit(front_y[:na] + front_y[n - na:n], front_x[:na] + front_x[n - na:n], 1)
        front_a, front_b = 1.0/front_a0, -front_b0/front_a0
        front_fit_x = np.array([-3, 3])
        front_fit_y = front_a * front_fit_x + front_b

        front_obs_x, front_obs_y = self.obstacle_detect(front_x, front_y, front_a, front_b, 0.1)
        front_obs_ave_x, front_obs_ave_y = np.mean(front_obs_x), np.mean(front_obs_y)
        front_obs_b = front_obs_ave_y - front_a * front_obs_ave_x
        front_obs_fit_y = front_a * front_fit_x + front_obs_b
        self.marker.points = []
        self.marker.points.append(Point(front_fit_x[0], front_fit_y[0], 0))
        self.marker.points.append(Point(front_fit_x[1], front_fit_y[1], 0))
        self.marker.points.append(Point(front_fit_x[0], front_obs_fit_y[0], 0))
        self.marker.points.append(Point(front_fit_x[1], front_obs_fit_y[1], 0))

        pitch_angle = math.asin(-self.obstacle_height * front_a / (front_b - front_obs_b))
        roll_angle = math.asin(math.tan(pitch_angle) / front_a)
        z = - front_b * math.sin(roll_angle) * math.cos(pitch_angle)

        # 真値の計算（gazeboのみ，比較のため）
        #try:
        #    trans = self.tfBuffer.lookup_transform('odom', 'lidar_with_mirror_center_link', data.header.stamp)
        #except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #    return
        #quaternion = trans.transform.rotation
        #z_t = trans.transform.translation.z
        #e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        #roll_t, pitch_t = e[0], e[1]
        #print("measure_roll_pitch_height, "+str(roll_angle)+", "+str(pitch_angle)+", "+str(z)+", true, "+str(roll_t)+", "+str(pitch_t)+", "+str(z_t))

        # Calculate left mirror pose
        left_pc = self.lp.projectLaser(left_data)
        left_x, left_y = [], []
        for p in pc2.read_points(left_pc, skip_nans=True, field_names=("x", "y", "z")):
            left_x.append(p[0])
            left_y.append(p[1])
        n = len(left_x) - 1
        left_a0, left_b0 = np.polyfit(left_x[:na] + left_x[n - na:n], left_y[:na] + left_y[n - na:n], 1)
        left_line_x, left_line_y = self.line_detect(left_x, left_y, left_a0, left_b0, 0.01)
        left_a, left_b = np.polyfit(left_line_x, left_line_y, 1)
        left_obs_x, left_obs_y = self.obstacle_detect(left_x, left_y, left_a, left_b, 0.1)
        left_obs_ave_x, left_obs_ave_y = np.mean(left_obs_x), np.mean(left_obs_y)
        left_obs_b = left_obs_ave_y - left_a * left_obs_ave_x
        left_roll_angle = math.acos(self.obstacle_height / self.dist_from_line(left_obs_ave_x, left_obs_ave_y, left_a, left_b)*math.cos(pitch_angle))
        left_distance = math.fabs(left_b)-z/(math.cos(pitch_angle)*math.cos(left_roll_angle))
        left_pitch_angle = math.asin(-left_a * (z + left_distance * math.cos(left_roll_angle)) / left_b)

        left_fit_x = np.array([-3, 3])
        left_fit_y = left_a * left_fit_x + left_b
        left_obs_fit_y = left_a * left_fit_x + left_obs_b
        self.marker.points.append(Point(left_fit_x[0], left_fit_y[0], 0))
        self.marker.points.append(Point(left_fit_x[1], left_fit_y[1], 0))
        self.marker.points.append(Point(left_fit_x[0], left_obs_fit_y[0], 0))
        self.marker.points.append(Point(left_fit_x[1], left_obs_fit_y[1], 0))

        # Calculate right mirror pose
        right_pc = self.lp.projectLaser(right_data)
        right_x, right_y = [], []
        for p in pc2.read_points(right_pc, skip_nans=True, field_names=("x", "y", "z")):
            right_x.append(p[0])
            right_y.append(p[1])
        n = len(right_x) - 1
        right_a0, right_b0 = np.polyfit(right_x[:na] + right_x[n - na:n], right_y[:na] + right_y[n - na:n], 1)
        right_line_x, right_line_y = self.line_detect(right_x, right_y, right_a0, right_b0, 0.01)
        right_a, right_b = np.polyfit(right_line_x, right_line_y, 1)
        right_obs_x, right_obs_y = self.obstacle_detect(right_x, right_y, right_a, right_b, 0.1)
        right_obs_ave_x, right_obs_ave_y = np.mean(right_obs_x), np.mean(right_obs_y)
        right_obs_b = right_obs_ave_y - right_a * right_obs_ave_x
        self.dist_from_line(right_obs_ave_x, right_obs_ave_y, right_a, right_b)
        right_roll_angle = math.acos(self.obstacle_height / self.dist_from_line(right_obs_ave_x, right_obs_ave_y, right_a, right_b)*math.cos(pitch_angle))
        right_distance = math.fabs(right_b)-z/(math.cos(pitch_angle)*math.cos(right_roll_angle))
        right_pitch_angle = math.asin(-right_a * (z + right_distance * math.cos(right_roll_angle)) / right_b)
        print("roll(->0), "+str(roll_angle)+", left_roll_pitch_dist, "+str(left_roll_angle+roll_angle)+", "+str(left_pitch_angle-pitch_angle)+", "+str(left_distance)+", right_roll_pitch_dist, "+str(right_roll_angle+roll_angle)+", "+str(right_pitch_angle-pitch_angle)+", "+str(right_distance))

        right_fit_x = np.array([-3, 3])
        right_fit_y = right_a * right_fit_x + right_b
        right_obs_fit_y = right_a * right_fit_x + right_obs_b
        self.marker.points.append(Point(right_fit_x[0], right_fit_y[0], 0))
        self.marker.points.append(Point(right_fit_x[1], right_fit_y[1], 0))
        self.marker.points.append(Point(right_fit_x[0], right_obs_fit_y[0], 0))
        self.marker.points.append(Point(right_fit_x[1], right_obs_fit_y[1], 0))
        self.marker.header.stamp = rospy.Time.now()
        self.maker_pub.publish(self.marker)

if __name__ == '__main__':
    rospy.init_node('calibrate_mirror_angle')
    node = CalibrateMirrorPose()
    while not rospy.is_shutdown():
        rospy.sleep(0.1)

