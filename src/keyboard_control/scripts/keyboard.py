#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

#from Types import Modes

import sys, select, termios, tty

class CentralNode :

    def get_key(self):
        """키 입력을 받는 함수 (논블로킹)"""
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return None

    def process_key(self, key) :
        str_key = String(key)
        rospy.loginfo("[Key Pressed!] : %s" % str_key)
        self.keyboard_pub.publish(str_key)

    def setup_terminal(self):
        """터미널을 raw 모드로 설정"""
        self.old_attr = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
    
    def restore_terminal(self):
        """터미널 설정 복원"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_attr)

    def __init__(self) :
        #self.curMode = Modes.RobotMode.STANDBY
        self.curMode = "KEYBOARD"
        self.curSpeed = 0.5

        self.setup_terminal()

        rospy.init_node('keyboard_node', anonymous=True)
        rospy.loginfo("[ROS Keyboard Node Initialzed!!]")

        # ** Publisher **
        self.keyboard_pub = rospy.Publisher('/keyboard', String, queue_size=8)
        self.rate = rospy.Rate(30)

    def run(self) :
        try:
            while not rospy.is_shutdown():
                # Initialize geometry_msgs::Twist type message
                key = self.get_key()
                if key:
                    self.process_key(key)
                
                self.rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("Ctrl+C로 종료됨")
        finally:
            # 터미널 설정 복원
            self.restore_terminal()



if __name__ == "__main__" :
    centralNode = CentralNode()
    centralNode.run()