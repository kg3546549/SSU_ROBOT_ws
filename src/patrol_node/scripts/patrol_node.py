#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math
from enum import Enum


class PatrolState(Enum):
    """순찰 상태"""
    INIT = 0           # 초기화
    FIND_WALL = 1      # 벽 찾기
    FOLLOW_WALL = 2    # 벽 추종
    OBSTACLE_WAIT = 3  # 장애물 대기
    TURN_LEFT = 4      # 좌회전
    TURN_RIGHT = 5     # 우회전


class PIDController:
    """PID 제어기"""
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.previous_error = 0.0
        self.error_sum = 0.0
        self.last_time = rospy.Time.now()

    def calculate(self, error):
        """PID 출력 계산"""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()

        if dt <= 0:
            dt = 0.01

        # P: 비례
        p_term = self.kp * error

        # I: 적분
        self.error_sum += error * dt
        i_term = self.ki * self.error_sum

        # D: 미분
        error_change = (error - self.previous_error) / dt
        d_term = self.kd * error_change

        # 업데이트
        self.previous_error = error
        self.last_time = current_time

        return p_term + i_term + d_term

    def reset(self):
        """PID 초기화"""
        self.previous_error = 0.0
        self.error_sum = 0.0
        self.last_time = rospy.Time.now()


class PatrolNode:
    def __init__(self):
        rospy.init_node('patrol_node', anonymous=True)

        # 상태
        self.state = PatrolState.INIT
        self.state_start_time = rospy.Time.now()

        # 센서 데이터
        self.scan_data = None
        self.front_distance = float('inf')
        self.right_distance = float('inf')
        self.left_distance = float('inf')

        # 파라미터
        self.wall_distance = 0.5        # 우측 벽 유지 거리 (m)
        self.obstacle_threshold = 0.4   # 전방 장애물 감지 거리 (m)
        self.gap_threshold = 1.2        # 우측 빈 공간 감지 거리 (m)
        self.forward_speed = 0.2        # 전진 속도 (m/s)
        self.rotation_speed = 0.5       # 회전 속도 (rad/s)
        self.obstacle_wait_time = 2.0   # 장애물 대기 시간 (초)

        # PID 제어기 (벽 추종용)
        self.wall_pid = PIDController(kp=1.0, ki=0.05, kd=0.3)

        # Publisher & Subscriber
        self.cmd_vel_pub = rospy.Publisher('/patrol/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.scanCallback)

        rospy.loginfo("[PatrolNode] Started - Waiting for scan data...")

    def scanCallback(self, msg):
        """레이저 스캔 데이터 처리"""
        self.scan_data = msg

        # 각도별 거리 추출
        ranges = list(msg.ranges)
        num_readings = len(ranges)

        # inf 값을 최대 거리로 대체
        ranges = [r if not math.isinf(r) else msg.range_max for r in ranges]

        # 전방: -30도 ~ +30도 (중앙 60도)
        front_start = int(num_readings * (330 / 360))
        front_end = int(num_readings * (30 / 360))
        front_readings = ranges[front_start:] + ranges[:front_end]
        self.front_distance = min(front_readings) if front_readings else float('inf')

        # 우측: 270도 ~ 300도 (우측 30도 범위)
        right_start = int(num_readings * (270 / 360))
        right_end = int(num_readings * (300 / 360))
        right_readings = ranges[right_start:right_end]
        self.right_distance = min(right_readings) if right_readings else float('inf')

        # 좌측: 60도 ~ 120도
        left_start = int(num_readings * (60 / 360))
        left_end = int(num_readings * (120 / 360))
        left_readings = ranges[left_start:left_end]
        self.left_distance = min(left_readings) if left_readings else float('inf')

    def publishCmdVel(self, linear_x=0.0, angular_z=0.0):
        """속도 명령 발행"""
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd)

    def changeState(self, new_state):
        """상태 변경"""
        rospy.loginfo("[PatrolNode] State: %s -> %s", self.state.name, new_state.name)
        self.state = new_state
        self.state_start_time = rospy.Time.now()

        # 상태 변경 시 PID 리셋
        if new_state == PatrolState.FOLLOW_WALL:
            self.wall_pid.reset()

    def getElapsedTime(self):
        """현재 상태 경과 시간"""
        return (rospy.Time.now() - self.state_start_time).to_sec()

    def stateMachine(self):
        """상태 머신 로직"""

        if self.state == PatrolState.INIT:
            # 초기화 - 스캔 데이터 대기
            if self.scan_data is not None:
                self.changeState(PatrolState.FIND_WALL)
            else:
                self.publishCmdVel(0, 0)

        elif self.state == PatrolState.FIND_WALL:
            # 벽 찾기 - 우측에 벽이 생길 때까지 좌회전
            if self.right_distance < self.wall_distance * 1.5:
                # 우측에 벽 발견
                self.changeState(PatrolState.FOLLOW_WALL)
            else:
                # 제자리 좌회전
                self.publishCmdVel(0, self.rotation_speed)

        elif self.state == PatrolState.FOLLOW_WALL:
            # 벽 추종

            # 1. 우측에 빈 공간 발견 -> 우회전
            if self.right_distance > self.gap_threshold:
                self.changeState(PatrolState.TURN_RIGHT)
                return

            # 2. 전방 장애물 발견 -> 대기
            if self.front_distance < self.obstacle_threshold:
                self.changeState(PatrolState.OBSTACLE_WAIT)
                return

            # 3. PID로 벽 추종
            error = self.wall_distance - self.right_distance
            angular_z = self.wall_pid.calculate(error)

            # 각속도 제한
            angular_z = max(-self.rotation_speed, min(self.rotation_speed, angular_z))

            self.publishCmdVel(self.forward_speed, angular_z)

        elif self.state == PatrolState.OBSTACLE_WAIT:
            # 장애물 대기
            self.publishCmdVel(0, 0)

            if self.getElapsedTime() > self.obstacle_wait_time:
                # 2초 경과 후에도 장애물이 있으면 벽으로 판단 -> 좌회전
                if self.front_distance < self.obstacle_threshold:
                    self.changeState(PatrolState.TURN_LEFT)
                else:
                    # 장애물 사라짐 -> 다시 전진
                    self.changeState(PatrolState.FOLLOW_WALL)

        elif self.state == PatrolState.TURN_LEFT:
            # 좌회전 - 전방이 충분히 열릴 때까지 회전
            if self.front_distance > self.obstacle_threshold * 2.5:
                # 회전 완료 - 전방이 열림
                rospy.loginfo("[PatrolNode] Left turn completed (front: %.2fm)", self.front_distance)
                self.changeState(PatrolState.FOLLOW_WALL)
            else:
                self.publishCmdVel(0, self.rotation_speed)

        elif self.state == PatrolState.TURN_RIGHT:
            # 우회전 - 우측 빈 공간이 전방으로 올 때까지 회전
            # 우회전 중에는 원래 전방이 우측으로 가고, 우측 빈 공간이 전방으로 옴
            # 따라서 우측 거리가 가까워지면(벽이 보이면) 회전 완료
            if self.right_distance < self.wall_distance * 2.0:
                # 회전 완료 - 우측에 벽이 보임
                rospy.loginfo("[PatrolNode] Right turn completed (right: %.2fm)", self.right_distance)
                self.changeState(PatrolState.FOLLOW_WALL)
            else:
                self.publishCmdVel(0, -self.rotation_speed)

    def run(self):
        """메인 루프"""
        rate = rospy.Rate(10)  # 10Hz

        while not rospy.is_shutdown():
            self.stateMachine()
            rate.sleep()


if __name__ == "__main__":
    try:
        patrol_node = PatrolNode()
        patrol_node.run()
    except rospy.ROSInterruptException:
        pass
