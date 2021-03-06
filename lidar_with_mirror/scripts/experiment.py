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
        rospy.init_node('move_lidar_node', anonymous=True)
        self.lidar_with_mirror_mirror1_pub   = rospy.Publisher('/lidar_with_mirror_mirror1_controller/command'  , Float64, queue_size=1)
        self.lidar_with_mirror_mirror2_pub   = rospy.Publisher('/lidar_with_mirror_mirror2_controller/command'  , Float64, queue_size=1)
        self.lidar_with_mirror_prismatic_pub = rospy.Publisher('/lidar_with_mirror_prismatic_controller/command', Float64, queue_size=1)
        self.lidar_with_mirror_pitch_pub     = rospy.Publisher('/lidar_with_mirror_pitch_controller/command'    , Float64, queue_size=1)
        self.lidar_with_mirror_roll_pub      = rospy.Publisher('/lidar_with_mirror_roll_controller/command'     , Float64, queue_size=1)
        self.caster_front_pub                = rospy.Publisher('/caster_front_controller/command'               , Float64, queue_size=1)
        self.wheel_hinge_pub                 = rospy.Publisher('/wheel_hinge_controller/command'                , Float64, queue_size=1)
        time.sleep(0.5)
        self.lidar_with_mirror_mirror1_pub.publish(0.0)
        self.lidar_with_mirror_mirror2_pub.publish(0.0)
        self.lidar_with_mirror_prismatic_pub.publish(0.0)
        self.lidar_with_mirror_pitch_pub.publish(0.0)
        self.lidar_with_mirror_roll_pub.publish(0.0)
        self.caster_front_pub.publish(0.0)
        self.wheel_hinge_pub.publish(0.0)
        self.seq_no = 0 #roll
        #self.seq_no = 1 #pitch
        self.prev_seq_no = -1

    def loop(self):
        if self.seq_no != self.prev_seq_no:
            self.is_first = True
        else:
            self.is_first = False
        self.prev_seq_no = self.seq_no
        if self.seq_no == 0:
            if self.is_first:
                self.angle_deg = -math.pi/30 - math.pi/30/5
            self.angle_deg += math.pi/30/5
            self.lidar_with_mirror_roll_pub.publish(self.angle_deg)
            print("roll angle: "+str(self.angle_deg))
            if self.angle_deg >= math.pi/30:
                self.seq_no = 100
        elif self.seq_no == 1:
            if self.is_first:
                self.angle_deg = -math.pi/18 - math.pi/9/10
            self.angle_deg += math.pi/9/10
            self.lidar_with_mirror_pitch_pub.publish(self.angle_deg)
            print("pitch angle: "+str(self.angle_deg))
            if self.angle_deg >= math.pi/9:
                self.seq_no = 100
        elif self.seq_no == 100:
            return False
        return True

if __name__ == '__main__':
    ml = move_lidar()
    DURATION = 12
    r = rospy.Rate(1.0 / DURATION)
    while not rospy.is_shutdown():
        if not ml.loop():
            break
        r.sleep()

