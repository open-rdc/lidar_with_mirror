#!/usr/bin/env python3
from __future__ import print_function
import roslib
roslib.load_manifest('lidar_with_mirror')
import rospy
from std_msgs.msg import Float64
import math
import time

class move_lidar:
    def __init__(self):
        rospy.init_node('move_lidar_node', anonymous=True)
        self.lidar_with_mirror_mirror1_prismatic_pub  = rospy.Publisher('/lidar_with_mirror_mirror1_prismatic_controller/command' , Float64, queue_size=1)
        self.lidar_with_mirror_mirror2_prismatic_pub  = rospy.Publisher('/lidar_with_mirror_mirror2_prismatic_controller/command' , Float64, queue_size=1)
        self.lidar_with_mirror_mirror1_roll_pub = rospy.Publisher('/lidar_with_mirror_mirror1_roll_controller/command', Float64, queue_size=1)
        self.lidar_with_mirror_mirror2_roll_pub = rospy.Publisher('/lidar_with_mirror_mirror2_roll_controller/command', Float64, queue_size=1)
        self.lidar_with_mirror_mirror1_prismatic2_pub = rospy.Publisher('/lidar_with_mirror_mirror1_prismatic2_controller/command', Float64, queue_size=1)
        self.lidar_with_mirror_mirror2_prismatic2_pub = rospy.Publisher('/lidar_with_mirror_mirror2_prismatic2_controller/command', Float64, queue_size=1)
        self.lidar_with_mirror_prismatic_pub    = rospy.Publisher('/lidar_with_mirror_prismatic_controller/command'   , Float64, queue_size=1)
        self.lidar_with_mirror_pitch_pub        = rospy.Publisher('/lidar_with_mirror_pitch_controller/command'       , Float64, queue_size=1)
        self.lidar_with_mirror_roll_pub         = rospy.Publisher('/lidar_with_mirror_roll_controller/command'        , Float64, queue_size=1)
        self.caster_front_pub                   = rospy.Publisher('/caster_front_controller/command'                  , Float64, queue_size=1)
        self.wheel_hinge_pub                    = rospy.Publisher('/wheel_hinge_controller/command'                   , Float64, queue_size=1)
        time.sleep(0.5)
        self.lidar_with_mirror_mirror1_prismatic_pub.publish(0.0)
        self.lidar_with_mirror_mirror2_prismatic_pub.publish(0.0)
        self.lidar_with_mirror_mirror1_roll_pub.publish(0.0)
        self.lidar_with_mirror_mirror2_roll_pub.publish(0.0)
        self.lidar_with_mirror_mirror1_prismatic2_pub.publish(0.0)
        self.lidar_with_mirror_mirror2_prismatic2_pub.publish(0.0)
        self.lidar_with_mirror_prismatic_pub.publish(0.0)
        self.lidar_with_mirror_pitch_pub.publish(0.0)
        self.lidar_with_mirror_roll_pub.publish(0.0)
        self.caster_front_pub.publish(0.0)
        self.wheel_hinge_pub.publish(0.0)
        self.seq_no = 1
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
                self.lidar_with_mirror_roll_pub.publish(0.0)
                self.seq_no = 1
        elif self.seq_no == 1:
            if self.is_first:
                self.angle_deg = -math.pi/18 - math.pi/18/5
            self.angle_deg += math.pi/18/5
            self.lidar_with_mirror_pitch_pub.publish(self.angle_deg)
            print("pitch angle: "+str(self.angle_deg))
            if self.angle_deg >= math.pi/18:
                self.lidar_with_mirror_pitch_pub.publish(0.0)
                self.seq_no = 2
        elif self.seq_no == 2:
            if self.is_first:
                self.height = -0.05
            self.height += 0.01
            self.lidar_with_mirror_prismatic_pub.publish(self.height)
            print("height: "+str(self.height))
            if self.height >= 0.05:
                self.lidar_with_mirror_prismatic_pub.publish(0.0)
                self.seq_no = 3
        elif self.seq_no == 3:
            if self.is_first:
                self.position = 0.0
            self.position -= 0.01
            self.lidar_with_mirror_mirror1_prismatic_pub.publish(self.position)
            self.lidar_with_mirror_mirror1_prismatic2_pub.publish(self.position)
            self.lidar_with_mirror_mirror2_prismatic_pub.publish(self.position)
            self.lidar_with_mirror_mirror2_prismatic2_pub.publish(self.position)
            print("mirror distance: "+str(self.position))
            if self.position <= -0.1:
                self.lidar_with_mirror_mirror1_prismatic_pub.publish(0.0)
                self.lidar_with_mirror_mirror1_prismatic2_pub.publish(0.0)
                self.lidar_with_mirror_mirror2_prismatic_pub.publish(0.0)
                self.lidar_with_mirror_mirror2_prismatic2_pub.publish(0.0)
                self.seq_no = 4
        elif self.seq_no == 4:
            if self.is_first:
                self.angle_deg = -math.pi/16
            self.angle_deg += math.pi / 16 / 5
            self.lidar_with_mirror_mirror1_roll_pub.publish(self.angle_deg)
            self.lidar_with_mirror_mirror2_roll_pub.publish(self.angle_deg)
            print("mirror1_2_roll: "+str(self.angle_deg))
            if self.angle_deg >= math.pi/16:
                self.lidar_with_mirror_mirror1_roll_pub.publish(0.0)
                self.lidar_with_mirror_mirror2_roll_pub.publish(0.0)
                self.seq_no += 100
        elif self.seq_no == 10:
            if self.is_first:
                self.height = -0.05
            self.height += 0.01
            self.caster_front_pub.publish(self.height)
            print("caster: "+str(self.height))
            if self.height >= 0.05:
                self.seq_no = 100
        elif self.seq_no == 100:
            return False
        return True

if __name__ == '__main__':
    ml = move_lidar()
    DURATION = 2
    r = rospy.Rate(1.0 / DURATION)
    while not rospy.is_shutdown():
        if not ml.loop():
            break
        r.sleep()
