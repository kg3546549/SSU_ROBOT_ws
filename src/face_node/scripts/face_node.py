#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from geometry_msgs.msg import Point

def main():
    rospy.init_node("face_node")
    rospy.loginfo("face_node started (fixed camera /dev/video0)")

    pub = rospy.Publisher("/face/position", Point, queue_size=1)

    # 고정 카메라: /dev/video0
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        rospy.logerr("Cannot open /dev/video0")
        return

    # Haar Cascade 경로 (Jetson / OpenCV 버전에 맞게 수정 필요할 수 있음)
    cascade_path = "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml"
    face_cascade = cv2.CascadeClassifier(cascade_path)
    if face_cascade.empty():
        rospy.logerr("Failed to load Haar Cascade: %s" % cascade_path)
        return

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Camera read failed")
            rate.sleep()
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        cx = w // 2
        cy = h // 2

        # 얼굴 검출
        faces = face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(80, 80)
        )

        msg = Point()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = 0.0

        face_center = None

        if len(faces) > 0:
            (x, y, fw, fh) = faces[0]
            fx = x + fw // 2
            fy = y + fh // 2
            face_center = (fx, fy)

            # 화면 중심 대비 오차
            msg.x = float(fx - cx)  # 오른쪽 → 양수
            msg.y = float(fy - cy)  # 아래쪽 → 양수

            # UI: 얼굴 박스 + 중심점
            cv2.rectangle(frame, (x, y), (x + fw, y + fh), (255, 0, 0), 2)
            cv2.circle(frame, (fx, fy), 5, (0, 0, 255), -1)

        # 화면 중앙 표시
        cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)

        # 텍스트로 현재 offset 보여주기
        text = "dx: {:.1f}, dy: {:.1f}".format(msg.x, msg.y)
        cv2.putText(frame, text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # ROS 퍼블리시
        pub.publish(msg)

        # OpenCV UI 표시
        cv2.imshow("Face Node (Camera 0)", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC 누르면 종료
            break

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()
    rospy.loginfo("face_node terminated.")


if __name__ == "__main__":
    main()
