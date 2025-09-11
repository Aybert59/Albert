#!/usr/bin/env python
# encoding: utf-8
import math
import numpy as np
import time
from common import *
from time import sleep
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from dynamic_reconfigure.server import Server
from yahboomcar_laser.cfg import laserAvoidPIDConfig

from laser_avoidance.msg import AlbertScan

RAD2DEG = 180 / math.pi

class laserAvoid:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.r = rospy.Rate(20)
        self.Moving = False
        self.switch = True
        self.angle_path = []
        self.CamScan = False
        self.ros_ctrl = ROSCtrl()
        Server(laserAvoidPIDConfig, self.dynamic_reconfigure_callback)
        self.linear = 0
        self.angular = 0.3
        self.lastangular = 0
        self.AlbertTwist = Twist()
        self.ResponseDist = 0.86 # soft warning
        self.ResponseStop = 0.50 # strong warning
        self.LaserAngle = 25  # 10~50 / angle de detection obstacle gauche et droite
        self.CenterAngle = 15  # moitié faisceau central. 
                               # angle 2*15° distance 60cm permet de passer si droite et gauche bouchés
                               # angle 2*10° distance 86cm Permet de passer si droite ou gauche bouché                          
        self.ObstacleValidAngle = 1  # largeur d'angle de detection de l'obstacle
        self.sub_laser = rospy.Subscriber('/scan', LaserScan, self.registerScan, queue_size=1)
        self.sub_camscan = rospy.Subscriber('/camera_scan', LaserScan, self.registerCamScan, queue_size=1)
        self.sub_vel = rospy.Subscriber('/Albert_desired_vel', Twist, self.registerAlbertVel, queue_size=1)
        self.firsttime = 1
        self.albert_scan_pub = rospy.Publisher('/AlbertScan', AlbertScan, queue_size=6)

	
    def cancel(self):
        self.ros_ctrl.pub_vel.publish(Twist())
        self.ros_ctrl.cancel()
        self.sub_laser.unregister()
        rospy.loginfo("Shutting down this node.")

    def dynamic_reconfigure_callback(self, config, level):
        self.switch = config['switch']
        self.linear = config['linear']
        self.angular = config['angular']
        self.LaserAngle = config['LaserAngle']
        self.ResponseDist = config['ResponseDist']
        return config


    def registerAlbertVel(self, albert_vel):
        if not isinstance(albert_vel, Twist): return

        # Record the Twist velocity requested by the joystick
        self.AlbertTwist = albert_vel

    def avgForDisplay(self, scan_data):
        
        # reduce the resolution of the laser scan for the display interface
        
        ranges = np.array(scan_data.ranges)
        nb = 0
        angle = int(scan_data.angle_min * RAD2DEG) + 90 #-90 90
        prev = angle
        rdata = np.zeros(181)
        

        for i in range(len(ranges)):
            #if ranges[i] is inf, -inf or nan skip it

            if math.isinf(ranges[i]) or math.isnan(ranges[i]):
                continue
            
            angle = int((scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG) + 90 #-90 90
            
            if angle == prev: 
                rdata[angle] += ranges[i]
                nb += 1
            else:
                rdata[prev] = rdata[prev]/nb
                rdata[angle] += ranges[i]
                prev = angle
                nb = 1 
        
        # moyenner le dernier
        rdata[angle] = rdata[angle]/nb

        return rdata
        

    def find_paths(self, scan_data, verbose=False, offset = 0.0):
        Right_warning = 0
        Left_warning = 0
        front_warning = 0
        Right_stop = 0
        Left_stop = 0
        front_stop = 0
        angle_min_left = 200
        angle_max_left = 200

        angle_path = [] 

        ranges = np.array(scan_data.ranges)

        # if we already have a last scan to compare to
        for i in range(len(ranges)):

            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG * 1 #-90 90

            if not (math.isinf(ranges[i]) or math.isnan(ranges[i])):
                #print(scan_data.angle_min)
                #print(int(5/(scan_data.angle_increment* RAD2DEG)))
                #print(angle)
                # if angle > 90: print "i: {},angle: {},dist: {}".format(i, angle, scan_data.ranges[i])
                #if self.firsttime == 1:
                #       print("i: "+str(i)+",angle: "+str(angle)+",dist: "+str(scan_data.ranges[i]))
                
                if -self.CenterAngle > angle > -self.CenterAngle-self.LaserAngle:
                    if ranges[i] < self.ResponseDist + offset: 
                        Right_warning += 1
                    if ranges[i] < self.ResponseStop + offset: 
                        Right_stop += 1
                        #print(angle)
                if self.CenterAngle+self.LaserAngle > angle > self.CenterAngle:
                    if ranges[i] < self.ResponseDist + offset: 
                        Left_warning += 1
                    if ranges[i] < self.ResponseStop + offset: 
                        Left_stop += 1
                        #print(angle)
                if abs(angle) < self.CenterAngle:
                    if ranges[i] <= self.ResponseDist + offset: 
                        front_warning += 1
                    if ranges[i] < self.ResponseStop + offset: 
                        front_stop += 1
                        #print(angle)
            
            #ignore if one point is not a number
                if (ranges[i] > self.ResponseDist + offset) or ((math.isnan(ranges[i]) or math.isinf(ranges[i])) and not(math.isnan(ranges[i-1]) or math.isinf(ranges[i-1]))):
                    if angle_min_left > angle:
                        if i==0:
                            angle_min_left = -120
                            angle_max_left = -120
                        else:
                            angle_min_left = angle
                            angle_max_left = angle
                    else:
                        angle_max_left = angle
                else:
                    if (abs(angle_max_left - angle_min_left)) > (2.0 * self.CenterAngle): # a path has been found
                        angle_path.append([angle_min_left, angle_max_left])
                        if (verbose): 
                            print ("found path: ", angle_min_left, angle_max_left)
                    angle_min_left = 200
                    angle_max_left = 200

            if verbose: 
                print(angle_min_left, angle_max_left)

            if i == len(ranges) - 1:
                if (angle - angle_min_left) > (2 * self.CenterAngle): # a path has been found
                    angle_path.append([angle_min_left, 120])

        return Right_warning, Left_warning, front_warning, Right_stop, Left_stop, front_stop, angle_path

    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan): return
        if not isinstance(self.CamScan, LaserScan): return


        albert_scan_msg = AlbertScan()
        albert_scan_msg.ResponseDist = self.ResponseDist
        albert_scan_msg.LaserAngle = self.LaserAngle

        albert_scan_msg.ranges = self.avgForDisplay(scan_data)
        albert_scan_msg.ranges2 = self.avgForDisplay(self.CamScan)
        self.albert_scan_pub.publish (albert_scan_msg)

        Right_warning, Left_warning, front_warning, Right_stop, Left_stop, front_stop, self.angle_path = self.find_paths(scan_data, False, 0.0)
        #print("Right_warning: ", Right_warning, "Left_warning: ", Left_warning, "front_warning: ", front_warning, "Right_stop: ", Right_stop, "Left_stop: ", Left_stop, "front_stop: ", front_stop)
        #print("angle_path: ", self.angle_path)

        # reste plus qu'à chercher les path du scan de caméra, puis intersection des deux
        # attention la caméra estg 14cm derrière le lidar
        CamRight_warning, CamLeft_warning, Camfront_warning, CamRight_stop, CamLeft_stop, Camfront_stop, Camangle_path = self.find_paths(self.CamScan, False, 0.14)
        #print("CamRight_warning: ", CamRight_warning, "CamLeft_warning: ", CamLeft_warning, "Camfront_warning: ", Camfront_warning, "CamRight_stop: ", CamRight_stop, "CamLeft_stop: ", CamLeft_stop, "Camfront_stop: ", Camfront_stop)
        #print("Camangle_path: ", Camangle_path)

        #judge real obstacle number
        valid_num=int(self.ObstacleValidAngle/(scan_data.angle_increment* RAD2DEG))
       
        if len(Camangle_path) == 0:
            #camera scan has no path : may be blind
            #print("Depth Camera can't see, stop")
            print("Depth Camera can't see, ignoring for navigation mode") # ignore for navigation mode, should be automatically used when in manual mode
            #Camfront_stop = 2*valid_num
        
        twist = Twist()
        twist.linear.x = self.AlbertTwist.linear.x
        twist.linear.y = self.AlbertTwist.linear.y
        twist.angular.z = self.AlbertTwist.angular.z

        #si obstacle devant camera l'ajouter à devant radar
        front_stop += Camfront_stop

        #remplacer le bloc de conditions par celui-ci  
        # pour le moment on s'occupe juste linear x et angular z
        if self.AlbertTwist.linear.x > 0 or self.AlbertTwist.angular.z != 0:
            if self.AlbertTwist.linear.x > 0 and front_warning < valid_num: # pas d'ostacle devant on y va
                twist.linear.x = self.AlbertTwist.linear.x
                # correction de la trajectoire si trop près d'un coté
                if Left_warning > valid_num and Right_stop < valid_num:
                    print ('correcting right')
                    twist.angular.z = -self.angular
                elif Right_warning > valid_num and Left_stop < valid_num:
                    print ('correcting left')
                    twist.angular.z = self.angular
                else:
                    twist.angular.z = self.AlbertTwist.angular.z

            elif self.AlbertTwist.linear.x > 0 and front_warning >= valid_num and front_stop < valid_num:
                # si pas d'obstacles droite ou gauche juste ralentir, on verrra plus tard pour tourner
                twist.angular.z = self.AlbertTwist.angular.z
                if Left_warning > valid_num or Right_warning > valid_num:
                    twist.linear.x = 0.2
                else:
                    #stop moving forward
                    twist.linear.x = 0
                    if len(self.angle_path) > 0:
                        path = self.angle_path[0]
                        if path[0] > -self.CenterAngle:
                            print ('found a path on the left')
                            twist.angular.z = self.angular # turn left
                        else:
                            print ('found a path on the right')
                            twist.angular.z = -self.angular # turn right

            elif self.AlbertTwist.linear.x > 0 and front_stop >= valid_num:
                #stop
                twist.linear.x = 0
                print('obstacle in front, stop')
                twist.angular.z = self.AlbertTwist.angular.z
                #decide here to turn left or right
                # first try : if there is one angle go for the first one
                if len(self.angle_path) > 0:
                    path = self.angle_path[0]
                    if path[0] > -self.CenterAngle:
                        print ('found a path on the left')
                        twist.angular.z = self.angular # turn left
                    else:
                        print ('found a path on the right')
                        twist.angular.z = -self.angular # turn right
                else:
                    print ('no path found, turning on place')
                    if self.lastangular == 0: 
                        self.lastangular = self.angular
                    twist.angular.z = self.lastangular

        # Left positive and right negative
        
        """ if self.front_warning > valid_num and self.Left_warning > valid_num and self.Right_warning > valid_num:
            print ('1, there are obstacles in the front, left and right, go backward')
            twist.linear.x = -0.15
            twist.angular.z = 0
        elif self.front_warning > valid_num and self.Left_warning <= valid_num and self.Right_warning > valid_num:
            print ('2, there is an obstacle in the middle right, turn left')
            twist.linear.x = 0
            twist.angular.z = self.angular
        elif self.front_warning > valid_num and self.Left_warning > valid_num and self.Right_warning <= valid_num:
            print ('4. There is an obstacle in the middle left, turn right')
            twist.linear.x = 0
            twist.angular.z = -self.angular
        elif self.front_warning > valid_num and self.Left_warning < valid_num and self.Right_warning < valid_num:
            print ('6, there is an obstacle in the middle, turn left')
            twist.linear.x = 0
            twist.angular.z = self.angular
        elif self.front_warning < valid_num and self.Left_warning > valid_num and self.Right_warning > valid_num:
            print ('7. There are obstacles on the left and right, go forward')
            twist.linear.x = self.linear
            twist.angular.z = 0 """
        #elif self.front_warning < valid_num and self.Left_warning > valid_num and self.Right_warning <= valid_num:
        #    print ('8, there is an obstacle on the left, turn right')
        #    twist.linear.x = 0
        #    twist.angular.z = -self.angular
        #elif self.front_warning < valid_num and self.Left_warning <= valid_num and self.Right_warning > valid_num:
        #    print ('9, there is an obstacle on the right, turn left')
        #    twist.linear.x = 0
        #    twist.angular.z = self.angular

        # display albert twist  linear velocity and angular velocity, and valid_number
        # linear : [-1; 1] angular : [-3.14; 3.14] valid_num = 11 si paramêtre = 2 degrés
        #print ('linear: {}, angular: {}, valid_num: {}'.format(twist.linear.x, twist.angular.z, valid_num))

        self.lastangular = twist.angular.z
        self.ros_ctrl.pub_vel.publish(twist)
        sleep(0.2)
        self.r.sleep()
        self.firsttime = 0
        # else : self.ros_ctrl.pub_vel.publish(Twist())

    def registerCamScan(self, scan_data):
        if not isinstance(scan_data, LaserScan): return
         
        #self.avgAndDisplay(self, scan_data)
        #register for later use
        self.CamScan = scan_data
           

        sleep(0.2)
        self.r.sleep()



if __name__ == '__main__':
    print("init laser_Avoidance")
    rospy.init_node('laser_Avoidance', anonymous=False)
    tracker = laserAvoid()
    rospy.spin()
