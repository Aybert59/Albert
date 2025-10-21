#!/usr/bin/env python
# encoding: utf-8

import time
import rospy
import os
from time import sleep
from yahboomcar_msgs.msg import *
from yahboomcar_msgs.srv import *
from sensor_msgs.msg import JointState
import json
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from mcp.srv import TripsService, TripsServiceResponse

import actionlib
from mcp.msg import DoTripAction, DoTripGoal, DoTripResult, DoTripFeedback

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from dynamic_reconfigure.client import Client

class MasterControl:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        
        self.armjoint = ArmJoint() 
        self.arm_joints = [90, 90, 90, 90, 90, 90]
        self.pub_Arm = rospy.Publisher("TargetAngle", ArmJoint, queue_size=1000)
        self.sub_Arm = rospy.Subscriber("ArmAngleUpdate", ArmJoint, self.Armcallback, queue_size=1000)
        self.srv_arm = rospy.ServiceProxy("CurrentAngle", RobotArmArray)

        self.list_trips = rospy.Service("/ListTrips", TripsService, self.handle_list_trips)
        self.doTripActionSrv = actionlib.SimpleActionServer("doTrip", DoTripAction, execute_cb=self.doTripCallback, auto_start = False)
        self.doTripActionSrv.start() 
        self.currentTrip = None
        self.currentGoal = None
        self.dynamicReconfigureDWAP = Client('/move_base/DWAPlannerROS')
        self.move_base_listener = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.move_base_result_callback, queue_size=1)

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
    
    def configure_move_base(self, pose):
        if pose is None: return

        params = {}
        if "yaw_tolerance" in pose:
            params["yaw_goal_tolerance"] = pose["yaw_tolerance"]
        if "xy_tolerance" in pose:
            params["xy_goal_tolerance"] = pose["xy_tolerance"]
        if "speed" in pose:
            params["max_vel_trans"] = pose["speed"] / 100.0
        if "rotspeed" in pose:
            params["max_vel_theta"] = pose["rotspeed"] / 100.0 * 3.2

        #print(params)
        if len(params) > 0: 
            self.dynamicReconfigureDWAP.update_configuration(params)

    def move_xyz (self,x,y,z):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        q = Quaternion(*quaternion_from_euler(0, 0, z))
        goal.target_pose.pose.orientation = q
        client.send_goal(goal)
        #client.wait_for_result()
        self.currentGoal = client
        
    def navigate_to (self, pose):
        if "name" in pose:
            target = pose["name"]
            self.doTripActionSrv.publish_feedback(DoTripFeedback(target))
            rospy.loginfo("Moving to pose: %s", target)
            
        self.configure_move_base(pose)

        if self.doTripActionSrv.is_preempt_requested():
            rospy.loginfo('DoTripAction: Preempted')
            self.doTripActionSrv.set_preempted()
            return
        if (self.currentGoal != None):
            #rospy.loginfo('Canceling previous goal')
            #self.currentGoal.cancel_all_goals()
            #self.currentGoal = None
            return
        else:
            x = pose["x"]
            y = pose["y"]
            z = pose["z"]
            rospy.loginfo('navigate to (' + str(x) + ', ' + str(y) + ', ' + str(z) + ')')
            self.move_xyz(x, y, z)  

    def move_base_result_callback(self, msg):
        #status
        #    PENDING=0
        #    uint8 ACTIVE=1
        #    uint8 PREEMPTED=2 or canceled
        #    uint8 SUCCEEDED=3
        #    uint8 ABORTED=4
        #    uint8 REJECTED=5
        #    uint8 PREEMPTING=6
        #    uint8 RECALLING=7
        #    uint8 RECALLED=8
        #    uint8 LOST=9
        
        rospy.loginfo('status: ' + str(msg.status.status))

        if msg.status.status == 2: #canceled
            self.currentGoal = None
            self.currentTrip = None
            rospy.loginfo('DoTripAction: canceled')
            self.doTripActionSrv.set_succeeded(DoTripResult(2))
        elif msg.status.status == 4: #aborted
            self.currentGoal = None
            self.currentTrip = None
            rospy.loginfo('DoTripAction: aborted')
            self.doTripActionSrv.set_succeeded(DoTripResult(4))
            
        elif msg.status.status == 3: #succeeded
            self.currentGoal = None
            if len(self.currentTrip["poses"]) > 0:
                pose = self.currentTrip["poses"].pop(0)
                self.navigate_to (pose)
            else:
                rospy.loginfo('DoTripAction: finished')
                self.doTripActionSrv.set_succeeded(DoTripResult(100))
                self.currentTrip = None

    def doTripCallback(self, goal):
        # helper variables
        r = rospy.Rate(1)
        
        # publish info to the console for the user
        rospy.loginfo('DoTripAction: starting trip %s' % (goal.tripName))

        if goal.tripName.startswith("trip"):
            trip_full_name = "/root/albert_ws/config/" + goal.tripName
            if not os.path.exists(trip_full_name):
                rospy.logwarn("Trip does not exist: %s", trip_full_name)
                self.doTripActionSrv.set_aborted(DoTripResult(0), "Trip directory does not exist")
                return

            # read the file as json object
            with open(trip_full_name, 'r') as f:
                trip_data = json.load(f)
                self.currentTrip = trip_data["trip"]
                #print("Trip data loaded: " + json.dumps(self.currentTrip))
                f.close()

            self.configure_move_base(self.currentTrip)

            if len(self.currentTrip["poses"]) > 0:
                pose = self.currentTrip["poses"].pop(0)
                self.navigate_to(pose)  
        elif goal.tripName.startswith("spot"):
            parts = goal.tripName.split('/')
            if len(parts) != 4:
                rospy.logwarn("Invalid spot trip format: %s", goal.tripName)
                self.doTripActionSrv.set_aborted(DoTripResult(0), "Invalid spot trip format")
                return

            x = float(parts[1])
            y = float(parts[2])
            z = float(parts[3])
            rospy.loginfo('navigate to (' + str(x) + ', ' + str(y) + ', ' + str(z) + ')')
            self.move_xyz(x, y, z)

        #if success:
        #    rospy.loginfo('DoTripAction: Succeeded')
        #    self.doTripActionSrv.set_succeeded(DoTripResult(100))


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

    def handle_list_trips(self, req):
        # get parameter /map_mode        
        map_mode = rospy.get_param("/map_mode")
        if map_mode == "scan":
            rospy.logwarn("Robot is in scan mode. Cannot list trips.")
            return TripsServiceResponse('')
        
        trip_directory = "/root/albert_ws/config/" + req.str  # Change this to your trip directory
        rospy.loginfo("Listing trips in directory: %s", trip_directory)

        if not os.path.exists(trip_directory):
            rospy.logwarn("Trip directory does not exist: %s", trip_directory)
            return TripsServiceResponse('')
        
        # List all available trips in the directory
        trips = []
        for trip_name in os.listdir(trip_directory):
            if trip_name.endswith(".trip"):
                trips.append(trip_name)

        # transfrom trips to json string
        response = str(trips).replace("'", '"')  # Simple conversion to JSON-like string
        rospy.loginfo(response)

        return TripsServiceResponse(response)

    


if __name__ == '__main__':
    rospy.init_node('master_control')
    MCP = MasterControl()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo('exception')
