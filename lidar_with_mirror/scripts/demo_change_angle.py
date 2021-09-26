#!/usr/bin/env python
from __future__ import print_function
import roslib
roslib.load_manifest('lidar_with_mirror')
import rospy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from std_msgs.msg import Float64
import math
import time

class move_lidar:
    def __init__(self):
        rospy.init_node('mmove_lidar_node', anonymous=True)
        self.lidar_with_mirror_mirror1_pub   = rospy.Publisher('/lidar_with_mirror_mirror1_controller/command'  , Float64, queue_size=1)
        self.lidar_with_mirror_mirror2_pub   = rospy.Publisher('/lidar_with_mirror_mirror2_controller/command'  , Float64, queue_size=1)
        self.lidar_with_mirror_prismatic_pub = rospy.Publisher('/lidar_with_mirror_prismatic_controller/command', Float64, queue_size=1)
        self.lidar_with_mirror_pitch_pub     = rospy.Publisher('/lidar_with_mirror_pitch_controller/command'    , Float64, queue_size=1)
        self.lidar_with_mirror_roll_pub      = rospy.Publisher('/lidar_with_mirror_roll_controller/command'     , Float64, queue_size=1)
        self.caster_front_pub                = rospy.Publisher('/caster_front_controller/command'               , Float64, queue_size=1)
        time.sleep(0.5)
        self.lidar_with_mirror_mirror1_pub.publish(0.0)
        self.lidar_with_mirror_mirror2_pub.publish(0.0)
        self.lidar_with_mirror_prismatic_pub.publish(0.0)
        self.lidar_with_mirror_pitch_pub.publish(0.0)
        self.lidar_with_mirror_roll_pub.publish(0.0)
        self.caster_front_pub.publish(0.0)
        self.seq_no = 0
        self.prev_seq_no = -1

    def loop(self):
        if self.seq_no != self.prev_seq_no:
            self.is_first = True
        else:
            self.is_first = False
        self.prev_seq_no = self.seq_no
        if self.seq_no == 0:
            if self.is_first:
                self.angle_deg = 0.0
            self.angle_deg -= math.pi/4/50
            self.lidar_with_mirror_mirror1_pub.publish(self.angle_deg)
            self.lidar_with_mirror_mirror2_pub.publish(self.angle_deg)
            if self.angle_deg <= -math.pi/4:
                self.seq_no += 1
        elif self.seq_no == 1:
            self.angle_deg += math.pi/4/50
            self.lidar_with_mirror_mirror1_pub.publish(self.angle_deg)
            self.lidar_with_mirror_mirror2_pub.publish(self.angle_deg)
            if self.angle_deg >= 0.0:
                self.seq_no += 1
        elif self.seq_no == 2:
            if self.is_first:
                self.angle_deg = 0.0
            self.angle_deg += math.pi/9/50
            self.lidar_with_mirror_pitch_pub.publish(self.angle_deg)
            if self.angle_deg >= math.pi/9:
                self.seq_no += 1
        elif self.seq_no == 3:
            self.angle_deg -= math.pi/9/50
            self.lidar_with_mirror_pitch_pub.publish(self.angle_deg)
            if self.angle_deg <= -math.pi/18:
                self.seq_no += 1
        elif self.seq_no == 4:
            self.angle_deg += math.pi/9/50
            self.lidar_with_mirror_pitch_pub.publish(self.angle_deg)
            if self.angle_deg >= 0.0:
                self.seq_no += 1
        elif self.seq_no == 5:
            if self.is_first:
                self.height = 0.0
            self.height += 0.1/25
            self.lidar_with_mirror_prismatic_pub.publish(self.height)
            if self.height >= 0.1:
                self.seq_no += 1
        elif self.seq_no == 6:
            self.height -= 0.1/25
            self.lidar_with_mirror_prismatic_pub.publish(self.height)
            if self.height <= 0.0:
                self.seq_no += 1
        elif self.seq_no == 7:
            if self.is_first:
                self.angle_deg = 0.0
            self.angle_deg += math.pi/10/25
            self.lidar_with_mirror_roll_pub.publish(self.angle_deg)
            if self.angle_deg >= math.pi/10:
                self.seq_no += 1
        elif self.seq_no == 8:
            self.angle_deg -= math.pi/10/25
            self.lidar_with_mirror_roll_pub.publish(self.angle_deg)
            if self.angle_deg <= -math.pi/10:
                self.seq_no += 1
        elif self.seq_no == 9:
            self.angle_deg += math.pi/10/25
            self.lidar_with_mirror_roll_pub.publish(self.angle_deg)
            if self.angle_deg >= 0:
                self.seq_no += 1
        elif self.seq_no == 10:
            if self.is_first:
                self.height = 0.0
            self.height -= 0.05/200
            self.caster_front_pub.publish(self.height)
            if self.height <= -0.05:
                self.seq_no += 1
        elif self.seq_no == 11:
            self.height += 0.05/200
            self.caster_front_pub.publish(self.height)
            if self.height >= 0.05:
                self.seq_no += 1
        elif self.seq_no == 12:
            self.height -= 0.05/200
            self.caster_front_pub.publish(self.height)
            if self.height <= 0.0:
                self.seq_no += 1
        elif self.seq_no == 13:
            return False
        return True

if __name__ == '__main__':
    ml = move_lidar()
    DURATION = 0.05
    r = rospy.Rate(1 / DURATION)
    while not rospy.is_shutdown():
        if not ml.loop():
            break
        r.sleep()


