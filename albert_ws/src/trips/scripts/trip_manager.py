#!/usr/bin/env python
# encoding: utf-8

import  sys 
import  rospy 
import json
import actionlib

from time import sleep
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from trips.msg import trip
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult

from dynamic_reconfigure.client import Client


class tripManager:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.r = rospy.Rate(20)

        self.currentGoal = None

        self.sub_trip = rospy.Subscriber('/trip', trip, self.manageTrip, queue_size=1)
        self.move_base_listener = rospy.Subscriber('/move_base/result', MoveBaseActionResult, self.move_base_result_callback, queue_size=1)
        self.pub_trip_info = rospy.Publisher('/tripInfo', String, queue_size=1) 
        self.dynamicReconfigureDWAP = Client('/move_base/DWAPlannerROS')
        
        sleep(0.2)
        self.r.sleep()

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
            self.pub_trip_info.publish("end")
        elif msg.status.status == 4: #aborted
            self.currentGoal = None
            self.pub_trip_info.publish("end")
        elif msg.status.status == 3: #succeeded
            self.currentGoal = None
            if len(self.currentTrip["poses"]) > 0:
                pose = self.currentTrip["poses"].pop(0)
                xyt = None
                yawt = None
                spd = None
                rspd = None
                if "yaw_tolerance" in pose:
                    yawt = pose["yaw_tolerance"]
                if "xy_tolerance" in pose:
                    xyt = pose["xy_tolerance"]
                if "speed" in pose:
                    spd = pose["speed"] / 100.0
                if "rotspeed" in pose:
                    rspd = pose["rotspeed"] / 100.0 * 3.2
                self.pub_trip_info.publish(pose['name'])
                self.navigate_to (pose['x'], pose['y'], pose['z'], yawt, xyt, spd, rspd)
            else:
                self.pub_trip_info.publish("end")

    def navigate_to (self, x, y, z, yaw_tolerance, xy_tolerance, speed, rotspeed):
        if (self.currentGoal != None):
            rospy.loginfo('Canceling previous goal')
            self.currentGoal.cancel_all_goals()
            self.currentGoal = None
        else:
            rospy.loginfo('navigate to (' + str(x) + ', ' + str(y) + ', ' + str(z) + ')')
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


            params = {}
            if yaw_tolerance != None:
                params["yaw_goal_tolerance"] = yaw_tolerance
            if xy_tolerance != None:
                params["xy_goal_tolerance"] = xy_tolerance
            if speed != None:
                params["max_vel_trans"] = speed
            if rotspeed != None:
                    params["max_vel_theta"] = rotspeed

            if len(params) > 0: 
                self.dynamicReconfigureDWAP.update_configuration(params)

            rospy.loginfo('navigate to (' + str(x) + ', ' + str(y) + ', ' + str(z) + ') - ' + str(params))
            client.send_goal(goal)
            #client.wait_for_result()
            self.currentGoal = client

            
        


    def manageTrip(self, msg): 
        if msg.trip == "cancel":
            if self.currentGoal != None:
                self.currentTrip = None
                rospy.loginfo('Canceling previous goal')
                self.currentGoal.cancel_all_goals()
                self.currentGoal = None
            return

        self.currentTrip = json.loads(msg.trip)["trip"]
        self.pub_trip_info.publish("Trip : " + self.currentTrip["name"] + " is started.")

        params = {}
        if "yaw_tolerance" in self.currentTrip:
            params["yaw_goal_tolerance"] = self.currentTrip["yaw_tolerance"]
        if "xy_tolerance" in self.currentTrip:
            params["xy_goal_tolerance"] = self.currentTrip["xy_tolerance"]
        if "speed" in self.currentTrip:
            params["max_vel_trans"] = self.currentTrip["speed"] / 100.0
        if "rotspeed" in self.currentTrip:
            params["max_vel_theta"] = self.currentTrip["rotspeed"] / 100.0 * 3.2

        print(params)
        if len(params) > 0: 
            self.dynamicReconfigureDWAP.update_configuration(params)

        pose = self.currentTrip["poses"].pop(0)
        xyt = None
        yawt = None
        spd = None
        rspd = None
        if "yaw_tolerance" in pose:
            yawt = pose["yaw_tolerance"]
        if "xy_tolerance" in pose:
            xyt = pose["xy_tolerance"]
        if "speed" in pose:
            spd = pose["speed"] / 100.0
        if "rotspeed" in pose:
            rspd = pose["rotspeed"] / 100.0 * 3.2
        self.navigate_to (pose['x'], pose['y'], pose['z'], yawt, xyt, spd, rspd)
        
        


    def cancel(self):

        rospy.loginfo("Shutting down trip_manager node.")



if __name__ == '__main__':
    print("init trip manager")
    rospy.init_node('trip_manager', anonymous=False)
    tracker = tripManager()
    rospy.spin()
