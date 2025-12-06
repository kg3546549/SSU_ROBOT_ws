#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point
from yahboomcar_msgs.msg import ArmJoint
from yahboomcar_msgs.srv import RobotArmArray, RobotArmArrayRequest


class FaceTrackController:
    def __init__(self):
        rospy.init_node("face_track_node")
        rospy.loginfo("face_track_node started with linear mapping mode.")

        # EMA smoothing
        self.ALPHA = 0.2

        self.smooth_x = 0.0
        self.smooth_y = 0.0

        self.first_face = False

        # yaw(idx=0), pitch(idx=3)
        self.joints_deg = [90, 145, 0, 45, 90, 30]

        rospy.Subscriber("/face/position", Point, self.face_callback)
        self.pub_arm = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1)

        self.load_current()

        # 10Hz
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)

    # ----------------------------
    # Load current joint values
    # ----------------------------
    def load_current(self):
        try:
            rospy.wait_for_service("CurrentAngle", 2.0)
            srv = rospy.ServiceProxy("CurrentAngle", RobotArmArray)
            resp = srv(RobotArmArrayRequest())
            if len(resp.angles) == 6:
                self.joints_deg = list(resp.angles)
                rospy.loginfo("Loaded angles: %s" % self.joints_deg)
        except:
            rospy.logwarn("Using default joint values.")

    # ----------------------------
    # Mapping functions
    # ----------------------------
    def map_yaw(self, dx):
        if dx <= -200:
            return 34.0
        if dx >= 200:
            return 120.0

        if dx < 0:
            # -200 → 34, 0 → 90
            return 34 + (dx + 200) * (90 - 34) / 200.0
        else:
            # 0 → 90, 200 → 120
            return 90 + dx * (120 - 90) / 200.0

    def map_pitch(self, dy):
        if dy <= -180:
            return 60.0
        if dy >= 170:
            return 0.0

        if dy < 0:
            # -180 → 60, 0 → 32
            return 60 + (dy + 180) * (32 - 60) / 180.0
        else:
            # 0 → 32, 170 → 0
            return 32 + dy * (0 - 32) / 170.0

    # ----------------------------
    # Face callback
    # ----------------------------
    def face_callback(self, msg):
        if not self.first_face:
            self.first_face = True
            return

        # EMA smoothing
        self.smooth_x = self.ALPHA * msg.x + (1 - self.ALPHA) * self.smooth_x
        self.smooth_y = self.ALPHA * msg.y + (1 - self.ALPHA) * self.smooth_y

    # ----------------------------
    # Main control loop
    # ----------------------------
    def control_loop(self, event):
        if not self.first_face:
            return

        yaw = self.map_yaw(self.smooth_x)
        pitch = self.map_pitch(self.smooth_y)

        # update joint array
        self.joints_deg[0] = yaw
        self.joints_deg[3] = pitch

        cmd = ArmJoint()
        cmd.id = -1
        cmd.run_time = 70
        cmd.joints = self.joints_deg

        self.pub_arm.publish(cmd)

        rospy.loginfo("Yaw: %.1f | Pitch: %.1f | dx=%.1f dy=%.1f"
                      % (yaw, pitch, self.smooth_x, self.smooth_y))


if __name__ == "__main__":
    FaceTrackController()
    rospy.spin()
