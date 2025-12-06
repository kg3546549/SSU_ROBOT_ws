#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point
from yahboomcar_msgs.msg import ArmJoint
from yahboomcar_msgs.srv import RobotArmArray, RobotArmArrayRequest


class FingerTrackController:
    def __init__(self):
        rospy.init_node("finger_track_node")
        rospy.loginfo("finger_track_node started (tracking finger position).")

        # smoothing
        self.ALPHA = 0.25
        self.sx = 0.0
        self.sy = 0.0
        self.first_detect = False

        # robot arm joints
        self.joints = [90, 145, 0, 45, 90, 30]

        rospy.Subscriber("/hand/position", Point, self.finger_callback)
        self.pub_arm = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1)

        self.load_current()

        rospy.Timer(rospy.Duration(0.1), self.control_loop)

    # ----------------------------------------
    # Load current servo positions
    # ----------------------------------------
    def load_current(self):
        try:
            rospy.wait_for_service("CurrentAngle", 2.0)
            srv = rospy.ServiceProxy("CurrentAngle", RobotArmArray)
            resp = srv(RobotArmArrayRequest())
            if len(resp.angles) == 6:
                self.joints = list(resp.angles)
                rospy.loginfo("Loaded arm angles: %s", self.joints)
        except:
            rospy.logwarn("Using default arm angles (load failed).")


    # ----------------------------------------
    # Mapping using given calibration points
    # ----------------------------------------
    def map_yaw(self, x):
        # x=0 → 136°, x=640 → 45°
        yaw_min = 136
        yaw_max = 45
        yaw = yaw_min - (float(x) / 640.0) * (yaw_min - yaw_max)
        return yaw

    def map_pitch(self, y):
        # y=480 → 31°, y=0 → 67°
        pitch_top = 67
        pitch_bottom = 31
        pitch = pitch_top - (float(y) / 480.0) * (pitch_top - pitch_bottom)
        return pitch


    # ----------------------------------------
    # Receive finger point
    # ----------------------------------------
    def finger_callback(self, msg):
        if not self.first_detect:
            self.first_detect = True
            return

        self.sx = self.ALPHA * msg.x + (1 - self.ALPHA) * self.sx
        self.sy = self.ALPHA * msg.y + (1 - self.ALPHA) * self.sy


    # ----------------------------------------
    # Control loop
    # ----------------------------------------
    def control_loop(self, event):

        if not self.first_detect:
            return

        yaw = self.map_yaw(self.sx)
        pitch = self.map_pitch(self.sy)

        self.joints[0] = yaw      # servo id 1
        self.joints[3] = pitch    # servo id 4

        cmd = ArmJoint()
        cmd.id = -1
        cmd.run_time = 80
        cmd.joints = self.joints

        self.pub_arm.publish(cmd)

        rospy.loginfo("Finger Tracking → yaw: %.1f pitch: %.1f | x=%.1f y=%.1f",
                      yaw, pitch, self.sx, self.sy)


if __name__ == "__main__":
    FingerTrackController()
    rospy.spin()
