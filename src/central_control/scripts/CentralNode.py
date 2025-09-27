#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

#from Types import Modes


class CentralNode :
    def keyboardCallback(self, msg) :

        rospy.loginfo("[KeyPressed!] %s", msg)

        vel_msg = Twist()
        if msg == "I" or msg == "i" : 
            vel_msg.linear.x = 0.5
        elif msg == "," or msg == "<" : 
            vel_msg.linear.x = -0.5
        elif msg == "j" or msg == "J" :
            vel_msg.linear.y = -0.5
        elif msg == "l" or msg == "L" :
            vel_msg.linear.y = 0.5
        else :
            vel_msg.linear.x = 0
            # vel_msg.linear = {0,0,0}
            # vel_msg.angular = {0,0,0}

        rospy.loginfo("[Twist] : %s", vel_msg)
        self.cmd_vel_publisher.publish(vel_msg)

    def __init__(self) :
        #self.curMode = Modes.RobotMode.STANDBY
        self.curMode = "KEYBOARD"
        self.curSpeed = 0.5

        rospy.init_node('central_node', anonymous=True)

        # ** Publisher **
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=8)

        # ** Subscriber **
        rospy.Subscriber("/keyboard", String, self.keyboardCallback)


    def run(self) :
        rospy.spin()

if __name__ == "__main__" :
    centralNode = CentralNode()
    centralNode.run()