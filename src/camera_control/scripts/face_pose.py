#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Pose
from common import RobotMode, TOPICS, get_mode_by_value
import cv2


class FaceRecogNode:
    def __init__(self):
        rospy.init_node('face_pose_node', anonymous=True)

        self.face_pose_publisher = rospy.Publisher(TOPICS.FACE_POSE, Pose, queue_size=10)
        self.face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")
        self.cap = cv2.VideoCapture(0)

    def run(self) :
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)

            if len(faces) > 0:
                x, y, w, h = faces[0]  # 첫 번째 얼굴
                cx = x + w // 2
                cy = y + h // 2

                facePose = Pose()
                facePose.position.x=cx
                facePose.position.y=cy

                self.face_pose_publisher.publish(facePose)

if __name__ == "__main__":
    try:
        faceRecog = FaceRecogNode()
        faceRecog.run()
    except rospy.ROSInterruptException:
        pass
