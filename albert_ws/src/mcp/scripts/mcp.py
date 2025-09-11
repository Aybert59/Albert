#!/usr/bin/env python
# encoding: utf-8

import time
import rospy
from time import sleep
from yahboomcar_msgs.msg import *
from yahboomcar_msgs.srv import *
from sensor_msgs.msg import JointState



class MasterControl:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        
        self.armjoint = ArmJoint() 
        self.arm_joints = [90, 90, 90, 90, 90, 90]
        self.pub_Arm = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1000)
        self.sub_Arm = rospy.Subscriber("ArmAngleUpdate", ArmJoint, self.Armcallback, queue_size=1000)
        self.srv_arm = rospy.ServiceProxy("CurrentAngle", RobotArmArray)

        rospy.loginfo("Initializing MCP")
        rospy.wait_for_message('/joint_states', JointState)
        rospy.loginfo("Topic '/joint_states' is now available!")

        request = RobotArmArrayRequest()
        response = self.srv_arm(request)
        current_angles = response.angles
        self.arm_joints = list(current_angles)
        rospy.loginfo("CurrentAngle: %s", self.arm_joints)  
        # Au repos : CurrentAngle: [90.0, 146.0, 0.0, 46.0, 89.0, 30.0]

        return 
    
        self.armjoint.id = 3
        self.armjoint.angle = 110
        self.pub_Arm.publish(self.armjoint)
        sleep(0.03)
        self.armjoint.id = 4
        self.armjoint.angle = 50
        self.pub_Arm.publish(self.armjoint)
        sleep(0.03)
        self.armjoint.id = 2
        self.armjoint.angle = 155
        self.pub_Arm.publish(self.armjoint)
        sleep(0.03)
        self.armjoint.id = 1
        self.armjoint.angle = 90
        self.pub_Arm.publish(self.armjoint)
        sleep(0.03)
        
        
        


    def Armcallback(self, msg):
        if not isinstance(msg, ArmJoint): return
        if len(msg.joints) != 0: self.arm_joints = list(msg.joints)
        else: self.arm_joints[msg.id - 1] = msg.angle
        rospy.loginfo("CurrentAngle: %s", self.arm_joints)


    def cancel(self):
        
        # Au repos : CurrentAngle: [90.0, 146.0, 0.0, 46.0, 89.0, 30.0]
        # ou CurrentAngle: [90.0, 147.0, 71.0, 47.0, 89.0, 30.0]

        return 
    
        self.armjoint.id = 1
        self.armjoint.angle = 90
        self.pub_Arm.publish(self.armjoint)
        sleep(0.03)
        self.armjoint.id = 2
        self.armjoint.angle = 147
        self.pub_Arm.publish(self.armjoint)
        sleep(0.03)
        self.armjoint.id = 3
        self.armjoint.angle = 75
        self.pub_Arm.publish(self.armjoint)
        sleep(0.03)
        self.armjoint.id = 4
        self.armjoint.angle = 46
        self.pub_Arm.publish(self.armjoint)
        sleep(0.03)
        self.sub_Arm.unregister()


if __name__ == '__main__':
    rospy.init_node('master_control')
    MCP = MasterControl()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('exception')
