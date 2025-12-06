#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from yahboomcar_msgs.msg import ArmJoint
from yahboomcar_msgs.srv import RobotArmArray, RobotArmArrayRequest


class FingerTrackController:
    def __init__(self):
        rospy.init_node("finger_track_node")
        rospy.loginfo("finger_track_node started, waiting for gesture node...")

        # READY FLAG
        self.ready = False
        rospy.Subscriber("/hand/ready", Bool, self.ready_callback)

        # EMA smoothing
        self.ALPHA = 0.25
        self.smooth_x = 0.0
        self.smooth_y = 0.0

        self.first_detect = False

        # yaw(idx=0), pitch(idx=3)
        self.joints_deg = [90, 145, 0, 45, 90, 30]

        # hand position subscriber
        rospy.Subscriber("/hand/position", Point, self.finger_callback)

        # arm publisher
        self.pub_arm = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1)

        # load current arm angles
        self.load_current()

        # 10Hz control loop
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)

    # --------------------------------------------
    # READY SIGNAL
    # --------------------------------------------
    def ready_callback(self, msg):
        if msg.data:
            self.ready = True
            rospy.loginfo("[finger_track_node] READY received → Tracking ENABLED ✔")

    # --------------------------------------------
    # Load current joint values
    # --------------------------------------------
    def load_current(self):
        try:
            rospy.wait_for_service("CurrentAngle", 2.0)
            srv = rospy.ServiceProxy("CurrentAngle", RobotArmArray)
            resp = srv(RobotArmArrayRequest())
            if len(resp.angles) == 6:
                self.joints_deg = list(resp.angles)
                rospy.loginfo("Loaded arm angles: %s" % self.joints_deg)
        except:
            rospy.logwarn("Using default joint values (load failed).")

    # --------------------------------------------
    # Mapping function (camera 640x480 기준)
    # --------------------------------------------
    def map_yaw(self, x):
        # 좌(0px) → angle 136  
        # 우(640px) → angle 45  
        return 136 + (x / 640.0) * (45 - 136)

    def map_pitch(self, y):
        # 하단(480px) → angle 31  
        # 상단(0px) → angle 67  
        return 31 + ((480 - y) / 480.0) * (67 - 31)

    # --------------------------------------------
    # Finger callback
    # --------------------------------------------
    def finger_callback(self, msg):
        if not self.ready:
            return

        if not self.first_detect:
            self.first_detect = True
            return

        # EMA
        self.smooth_x = self.ALPHA * msg.x + (1 - self.ALPHA) * self.smooth_x
        self.smooth_y = self.ALPHA * msg.y + (1 - self.ALPHA) * self.smooth_y

    # --------------------------------------------
    # Main loop
    # --------------------------------------------
    def control_loop(self, event):

        if not self.ready:
            return

        if not self.first_detect:
            return

        yaw = self.map_yaw(self.smooth_x)
        pitch = self.map_pitch(self.smooth_y)

        self.joints_deg[0] = yaw
        self.joints_deg[3] = pitch

        cmd = ArmJoint()
        cmd.id = -1
        cmd.run_time = 70
        cmd.joints = self.joints_deg

        self.pub_arm.publish(cmd)

        rospy.loginfo("Finger → Yaw: %.1f | Pitch: %.1f" % (yaw, pitch))


if __name__ == "__main__":
    FingerTrackController()
    rospy.spin()
