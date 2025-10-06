#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import json
from common import RobotMode, TOPICS, get_mode_by_value


class CentralNode:
    def __init__(self):
        rospy.init_node('central_node', anonymous=True)

        # 현재 모드 초기화
        self.curMode = RobotMode.IDLE
        self.curSpeed = 0.5

        # Publisher
        self.cmd_vel_publisher = rospy.Publisher(TOPICS.CMD_VEL, Twist, queue_size=10)

        # Mode Manager 구독 - 모드 변경 감지
        rospy.Subscriber(TOPICS.MODE_BROAD, String, self.modeBroadCallback)

        # 입력 소스별 구독자
        rospy.Subscriber("/keyboard", String, self.keyboardCallback)
        rospy.Subscriber(TOPICS.WEB_CMD_VEL, Twist, self.webCmdVelCallback)
        rospy.Subscriber("/joy", Joy, self.joyCallback)
        rospy.Subscriber("/patrol/cmd_vel", Twist, self.patrolCmdVelCallback)

        rospy.loginfo("[CentralNode] Started with mode: %s", self.curMode.name)

    def modeBroadCallback(self, msg):
        """Mode Manager로부터 모드 변경 수신"""
        try:
            data = json.loads(msg.data)
            mode_value = data.get('mode')
            speed = data.get('speed', self.curSpeed)

            new_mode = get_mode_by_value(mode_value)
            if new_mode:
                self.curMode = new_mode
                self.curSpeed = speed
                rospy.loginfo("[CentralNode] Mode changed to: %s (speed: %.2f)",
                            self.curMode.name, self.curSpeed)

                # EMERGENCY 모드 시 즉시 정지
                if self.curMode == RobotMode.EMERGENCY:
                    self.emergencyStop()
        except json.JSONDecodeError as e:
            rospy.logerr("[CentralNode] Invalid mode message: %s", e)

    def emergencyStop(self):
        """긴급 정지 - 모든 속도 0으로"""
        rospy.logwarn("[CentralNode] EMERGENCY STOP!")
        stop_msg = Twist()
        self.cmd_vel_publisher.publish(stop_msg)

    def keyboardCallback(self, msg):
        """키보드 입력 처리 - KEYBOARD 모드에서만 동작"""
        if self.curMode != RobotMode.KEYBOARD:
            return

        key = msg.data.lower()
        rospy.loginfo("[CentralNode] Key pressed: %s", key)

        vel_msg = Twist()

        # 전진/후진
        if key == 'i':
            vel_msg.linear.x = self.curSpeed
        elif key == ',':
            vel_msg.linear.x = -self.curSpeed

        # 좌우 이동
        elif key == 'j':
            vel_msg.linear.y = self.curSpeed
        elif key == 'l':
            vel_msg.linear.y = -self.curSpeed

        # 회전
        elif key == 'u':
            vel_msg.angular.z = self.curSpeed
        elif key == 'o':
            vel_msg.angular.z = -self.curSpeed

        # 정지
        elif key == 'k' or key == ' ':
            vel_msg = Twist()

        else:
            return  # 알 수 없는 키는 무시

        self.cmd_vel_publisher.publish(vel_msg)

    def webCmdVelCallback(self, msg):
        """웹 조이스틱 입력 처리 - WEB_JOY 모드에서만 동작"""
        if self.curMode != RobotMode.WEB_JOY:
            return

        # 웹에서 받은 Twist 메시지를 그대로 cmd_vel로 전달
        self.cmd_vel_publisher.publish(msg)

    def joyCallback(self, msg):
        """조이스틱 입력 처리 - JOY 모드에서만 동작"""
        if self.curMode != RobotMode.JOY:
            return

        vel_msg = Twist()

        # 일반적인 조이스틱 매핑 (axes[1]: 전후진, axes[0]: 좌우, axes[3]: 회전)
        # 조이스틱 종류에 따라 인덱스 조정 필요
        if len(msg.axes) >= 4:
            vel_msg.linear.x = msg.axes[1] * self.curSpeed
            vel_msg.linear.y = msg.axes[0] * self.curSpeed
            vel_msg.angular.z = msg.axes[3] * self.curSpeed

        self.cmd_vel_publisher.publish(vel_msg)

    def patrolCmdVelCallback(self, msg):
        """순찰 입력 처리 - AUTO_PATROL 모드에서만 동작"""
        if self.curMode != RobotMode.AUTO_PATROL:
            return

        # 순찰 노드에서 받은 Twist 메시지를 그대로 cmd_vel로 전달
        self.cmd_vel_publisher.publish(msg)

    def run(self):
        """메인 루프"""
        rate = rospy.Rate(10)  # 10Hz

        while not rospy.is_shutdown():
            # IDLE 모드에서는 대기
            if self.curMode == RobotMode.IDLE:
                pass

            # EMERGENCY 모드에서는 지속적으로 정지 명령 전송
            elif self.curMode == RobotMode.EMERGENCY:
                self.emergencyStop()

            rate.sleep()


if __name__ == "__main__":
    try:
        centralNode = CentralNode()
        centralNode.run()
    except rospy.ROSInterruptException:
        pass
