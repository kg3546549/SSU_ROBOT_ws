#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os

sys.path.insert(0, '/home/yahboom/SSU_ROBOT_ws/devel/lib/python3/dist-packages')

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json
from mode_manager.srv import ModeRequest, ModeRequestResponse
from common import (TOPICS, SERVICES, RobotMode, create_response, get_mode_by_value, get_mode_by_name)


class ModeManager:
    def modeCmd_callback(self):
        a = 1

    def __init__(self):
        rospy.init_node('mode_manager', anonymous=True)

        self.curMode = RobotMode.IDLE
        self.curSpeed = 0.5

        self.mode_pub = rospy.Publisher(TOPICS.MODE_BROAD, String, queue_size=1, latch=True)
        self.mode_service = rospy.Service(SERVICES.MODE_SRV, ModeRequest, self.modeService)

        self.publish_current_mode()
        rospy.loginfo("Mode Manager started %s", self.curMode)

    def publish_current_mode(self):
        msg = String()
        msg.data = json.dumps({
            'mode': self.curMode.value,
            'name': self.curMode.name,
            'speed': self.curSpeed
        })
        self.mode_pub.publish(msg)

    def modeService(self, msg):
        rospy.loginfo("[ModeManager] msg received : %s", msg)

        try:
            data = json.loads(msg.request_data)
            command = data.get('command', '').lower()
            
            # 각 handle 함수는 dict를 반환
            if command == 'get':
                response = self.handle_get_mode()
            elif command == 'set':
                response = self.handle_set_mode(data)
            elif command == 'list':
                response = self.handle_list_modes()
            else:
                response = self.create_error_response(f"Unknown command: {command}")

            #rospy.loginfo("[ModeManager] response %s", response)
            
            # dict → JSON 문자열 → ModeRequestResponse
            return ModeRequestResponse(json.dumps(response))

        except json.JSONDecodeError as e:
            rospy.logerr(f"Invalid JSON: {e}")
            error_response = self.create_error_response("Invalid JSON format")
            return ModeRequestResponse(json.dumps(error_response))
        except Exception as e:
            rospy.logerr(f"Unexpected error: {e}")
            error_response = self.create_error_response(f"Error: {str(e)}")
            return ModeRequestResponse(json.dumps(error_response))

    def handle_get_mode(self):
        """현재 모드 조회 - dict 반환"""
        response = create_response(True, "Current mode retrieved")
        response["current_mode"] = self.curMode.value
        response["mode_name"] = self.curMode.name
        response["speed"] = self.curSpeed
        return response

    def handle_set_mode(self, data):
        """모드 변경 - dict 반환"""
        try:
            # 모드 값 가져오기
            getModeRaw = data.get('mode', None)
            
            if getModeRaw is None:
                return create_response(False, "Mode value is required")
            
            rospy.loginfo("[ModeManager] Requested mode: %d", getModeRaw)
            
            # 모드 검증
            getMode = get_mode_by_value(getModeRaw)
            if getMode is None:
                return create_response(False, f"Invalid mode value: {getModeRaw}")
            
            # 속도 설정 (선택)
            if 'speed' in data:
                self.curSpeed = data.get('speed', 0.5)
            
            # 모드 변경
            self.curMode = getMode
            
            # 응답 생성
            response = create_response(True, "Mode changed successfully")
            response["current_mode"] = self.curMode.value
            response["mode_name"] = self.curMode.name
            response["speed"] = self.curSpeed
            
            rospy.loginfo("[ModeManager] Mode Changed! %s", self.curMode.name)
            
            # 현재 모드 브로드캐스트
            self.publish_current_mode()
            
            return response
            
        except Exception as e:
            rospy.logerr("[ModeManager] Error in handle_set_mode: %s", str(e))
            return create_response(False, f"Error: {str(e)}")

    def handle_list_modes(self):
        """모드 목록 조회 - dict 반환"""
        modes_list = [
            {"value": mode.value, "name": mode.name}
            for mode in RobotMode
        ]
        response = create_response(True, "Modes list retrieved")
        response["modes"] = modes_list
        return response

    def create_error_response(self, error_message):
        """에러 응답 생성"""
        return {
            'success': False,
            'current_mode': self.curMode.value,
            'mode_name': self.curMode.name,
            'message': error_message
        }

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    mode_manager_node = ModeManager()
    mode_manager_node.run()