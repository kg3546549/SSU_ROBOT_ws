#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os

sys.path.insert(0, '/home/yahboom/SSU_ROBOT_ws/devel/lib/python3/dist-packages')

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import (Trigger, TriggerResponse, TriggerRequest)
import json
from mode_manager.srv import ModeRequest, ModeRequestResponse
from common import TOPICS, SERVICES, RobotMode


#from Types import Modes

# class RobotMode(Enum):
#     IDLE = 0
#     #MANUAL = 1
#     AUTO_PATROL = 1
#     AUTO_NAVIGATION = 2
#     KEYBOARD = 3
#     WEB_JOY = 4
#     JOY = 5
#     EMERGENCY = 99


class ModeManager :
    def modeCmd_callback(self) :
        a=1

    def __init__(self) :
        rospy.init_node('mode_manager', anonymous=True)

        #self.curMode = Modes.RobotMode.STANDBY
        self.curMode:RobotMode = RobotMode.IDLE
        self.curSpeed = 0.5

        self.mode_pub = rospy.Publisher(TOPICS.MODE_BROAD, String, queue_size=1, latch=True)
        self.mode_service = rospy.Service(SERVICES.MODE_SRV, ModeRequest, self.modeService)

        #self.status_pub = rospy.Publisher('/mode/status', String, queue_size=10)
        
        
        self.publish_current_mode()
        rospy.loginfo("Mode Manager started %s", self.curMode)


    def publish_current_mode(self):
        msg = String()
        msg.data = json.dumps({
            # 'mode': self.current_mode.value,
            # 'name': self.current_mode.name
        })
        self.mode_pub.publish(msg)

    def modeService(self, msg) :
        rospy.loginfo("[ModeManager] msg received : %s", msg)
        try:
            data = json.loads(msg.data)
            command = data.get('command', '').lower()
            
            if command == 'get':
                response = self.handle_get_mode()
            elif command == 'set':
                response = self.handle_set_mode(data)
            elif command == 'list':
                response = self.handle_list_modes()
            else:
                response = self.create_error_response(f"Unknown command: {command}")
            
            # 응답 발행
            response_msg = String()
            response_msg.data = json.dumps(response)
            return ModeRequestResponse(response_msg)

        except json.JSONDecodeError as e:
            rospy.logerr(f"Invalid JSON: {e}")
            error_response = self.create_error_response("Invalid JSON format")
            response_msg = String()
            response_msg.data = json.dumps(error_response)

            return ModeRequestResponse(response_msg)

    def handle_get_mode(self) :
        A=1
    
    def handle_set_mode(self, data) :
        data

    def handle_list_modes(self) :
        A=1


    def create_error_response(self, error_message):
        """에러 응답 생성"""
        return {
            'success': False,
            'current_mode': self.current_mode.value,
            'mode_name': self.current_mode.name,
            'message': error_message
        }

    def run(self) :
        rospy.spin()

if __name__ == "__main__" :
    mode_manager_node = ModeManager()
    mode_manager_node.run()