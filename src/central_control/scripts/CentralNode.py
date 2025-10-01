#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json

#from Types import Modes

class RobotMode(Enum):
    IDLE = 0
    #MANUAL = 1
    AUTO_PATROL = 1
    AUTO_NAVIGATION = 2
    KEYBOARD = 3
    WEB_JOY = 4
    JOY = 5
    EMERGENCY = 99


class CentralNode :
    def webCmdVelCallback(self, msg:Twist) :
        if self.curMode != "WEB_MANUAL" :
            return
        
        self.cmd_vel_publisher.publish(msg)
        
    def webCmdModeCallback(self, msg:String) :
        jsonData = json.loads(msg)
        jsonData.


    def keyboardCallback(self, msg) :
        if self.curMode != RobotMode.KEYBOARD :
            return

        rospy.loginfo("[KeyPressed!] %s", msg)

        vel_msg = Twist()
        if msg == String("I") or msg == String("i") : 
            vel_msg.linear.x = 0.5
        elif msg == String(",") or msg == String("<") : 
            vel_msg.linear.x = -0.5
        elif msg == String("j") or msg == String("J") :
            vel_msg.linear.y = -0.5
        elif msg == String("l") or msg == String("L") :
            vel_msg.linear.y = 0.5
        
        elif msg == String("u") or msg == String("U") :
            vel_msg.angular.z = -0.5

        elif msg == String("p") or msg == String("P") :
            vel_msg.angular.z = 0.5
        
        else :
            vel_msg.linear.x = 0
            # vel_msg.linear = {0,0,0}
            # vel_msg.angular = {0,0,0}

        rospy.loginfo("[Twist] : %s", vel_msg)
        self.cmd_vel_publisher.publish(vel_msg)

    def __init__(self) :
        #self.curMode = Modes.RobotMode.STANDBY
        self.curMode:RobotMode = RobotMode.IDLE
        self.curSpeed = 0.5

        rospy.init_node('central_node', anonymous=True)

        # ** Publisher **
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=8)

        # ** Keyboard Subscriber **
        rospy.Subscriber("/keyboard", String, self.keyboardCallback)

        # ** Web Subscriber **
        rospy.Subscriber("/web/cmd_vel", String, self.webCmdVelCallback)
        
        # ** Service Server **
        rospy.Service("/web/cmd_mode", String, self.webCmdModeCallback)



    def run(self) :
        rospy.spin()

if __name__ == "__main__" :
    centralNode = CentralNode()
    centralNode.run()