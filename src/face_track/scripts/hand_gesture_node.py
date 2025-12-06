#!/usr/bin/env python3.9
# coding: utf-8

import rospy
import cv2 as cv
from media_library import HandDetector
from geometry_msgs.msg import Point
from std_msgs.msg import String, Bool
from time import time


class FingerGestureNode:
    def __init__(self):
        rospy.init_node("finger_gesture_node", anonymous=True)

        # Mediapipe Hand Detector
        self.hand_detector = HandDetector(detectorCon=0.7, trackCon=0.7)

        # Camera
        self.cap = cv.VideoCapture(1)
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
        rospy.loginfo("[finger_gesture_node] Camera initialized")

        # Publishers
        self.pub_pos = rospy.Publisher("/hand/position", Point, queue_size=10)
        self.pub_gesture = rospy.Publisher("/hand/gesture", String, queue_size=10)
        self.pub_ready = rospy.Publisher("/hand/ready", Bool, queue_size=1)

        # 노드 준비 완료 → hand_track_node가 tracking을 시작하게 됨
        rospy.sleep(1.0)
        self.pub_ready.publish(True)
        rospy.loginfo("[finger_gesture_node] READY signal sent ✔")

        self.pTime = 0
        self.gesture = ""

    def run(self):
        while not rospy.is_shutdown():

            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("[finger_gesture_node] Failed to read frame")
                continue

            frame = cv.flip(frame, 1)
            frame, lmList, bbox = self.hand_detector.findHands(frame)

            # ------------------------------  
            # Hand Landmark Exists  
            # ------------------------------
            if lmList:

                # (1) 손가락 위치 publish
                idx_finger_tip = lmList[8]  # [id, x, y]

                msg = Point()
                msg.x = float(idx_finger_tip[1])
                msg.y = float(idx_finger_tip[2])
                msg.z = 0.0
                self.pub_pos.publish(msg)

                # (2) 제스처 publish
                self.gesture = self.hand_detector.get_gesture(lmList)

                if self.gesture != "":
                    self.pub_gesture.publish(self.gesture)
                    rospy.loginfo(f"[finger_gesture_node] Gesture: {self.gesture}")

            # FPS 표시
            cTime = time()
            fps = 1 / (cTime - self.pTime) if self.pTime != 0 else 0
            self.pTime = cTime

            cv.putText(frame, f"FPS: {int(fps)}", (20, 30),
                       cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            cv.putText(frame, f"Gesture: {self.gesture}", (20, 70),
                       cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            

            cv.imshow("Finger Gesture Node", frame)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv.destroyAllWindows()


if __name__ == "__main__":
    node = FingerGestureNode()
    node.run()
