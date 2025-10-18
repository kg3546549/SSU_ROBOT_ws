#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import moveit_commander
from std_msgs.msg import Float64MultiArray
from yahboomcar_msgs.msg import ArmJoint
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose

class MoveItCoordinateToAngle:
    def __init__(self):
        rospy.init_node('moveit_coordinate_to_angle_node')
        
        # MoveIt 초기화
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.loginfo("Waiting for MoveIt to be ready...")
        
        try:
            self.arm_group = moveit_commander.MoveGroupCommander(
                "arm_group",
                wait_for_servers=15.0
            )
            rospy.loginfo("✓ Successfully connected to MoveIt!")
            
        except RuntimeError as e:
            rospy.logfatal("Failed to connect to MoveIt!")
            rospy.logfatal("Error: %s", str(e))
            raise
        
        # 구독자 & 발행자
        rospy.Subscriber('/moveit_coordinate', Float64MultiArray, 
                        self.coordinate_callback, queue_size=10)
        
        self.target_angle_pub = rospy.Publisher('/TargetAngle', ArmJoint, queue_size=10)
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("MoveIt Coordinate to Angle Bridge Ready")
        rospy.loginfo("ROS Distribution: Melodic")
        rospy.loginfo("Subscribing: /moveit_coordinate [x, y, z, w(yaw)]")
        rospy.loginfo("Publishing: /TargetAngle (ArmJoint)")
        rospy.loginfo("Planning Group: %s", self.arm_group.get_name())
        rospy.loginfo("Joints: %s", self.arm_group.get_active_joints())
        rospy.loginfo("=" * 50)
    
    def coordinate_callback(self, msg):
        if len(msg.data) != 4:
            rospy.logwarn("Expected 4 values [x, y, z, w], got %d", len(msg.data))
            return
        
        x, y, z, yaw = msg.data
        rospy.loginfo("Received: x=%.3f, y=%.3f, z=%.3f, yaw=%.3f", x, y, z, yaw)
        
        # Pose 생성
        pose_goal = Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        
        q = quaternion_from_euler(0, 0, yaw)
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]
        
        # IK 계산
        self.arm_group.set_pose_target(pose_goal)
        
        rospy.loginfo("Planning with IK...")
        plan = self.arm_group.plan()
        
        # ROS Melodic 호환: plan()은 RobotTrajectory 객체 반환
        # 빈 trajectory인지 확인
        if plan and len(plan.joint_trajectory.points) > 0:
            # 성공!
            target_angles_rad = plan.joint_trajectory.points[-1].positions
            target_angles_deg = [int(angle * 180.0 / 3.14159) for angle in target_angles_rad]
            
            rospy.loginfo("✓ IK success! Angles(deg): %s", target_angles_deg)
            rospy.loginfo("  Joint angles (rad): %s", 
                         [round(a, 3) for a in target_angles_rad])
            
            self.publish_target_angle(target_angles_deg)
        else:
            # 실패
            rospy.logwarn("✗ IK failed - pose unreachable or no valid solution")
            rospy.logwarn("  Target: x=%.3f, y=%.3f, z=%.3f, yaw=%.3f", x, y, z, yaw)
            rospy.logwarn("  Tip: Try a closer position or different orientation")
    
    def publish_target_angle(self, angles_deg, run_time=1000):
        msg = ArmJoint()
        msg.joints = angles_deg
        msg.run_time = run_time
        
        self.target_angle_pub.publish(msg)
        rospy.loginfo("→ Published /TargetAngle: %s (run_time=%dms)", 
                     msg.joints, msg.run_time)
    
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = MoveItCoordinateToAngle()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Failed to start node: %s", e)
        import traceback
        traceback.print_exc()