#!/usr/bin/env python3
# encoding: utf-8
import base64
import sys
import time
import rospy
import rospkg
import cv2 as cv
from time import sleep
from yahboomcar_msgs.msg import *
from sensor_msgs.msg import Image
from ultralytics import YOLO

#from classification.msg import detection 
#from transformers import AutoImageProcessor, AutoModelForImageClassification    #### tensorflow semble bien installé. Cf comment installer transformers
from cv_bridge import CvBridge, CvBridgeError

class ObjDetect:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.bridge = CvBridge()
        
        self.image_pub = rospy.Publisher("/camera/detection/image_raw",Image, queue_size=1)
        #self.detected_pub = rospy.Publisher("/classification/detected",detection, queue_size=6)
        self.pTime = self.cTime = 0
        self.device = rospy.get_param("~device", "OrinNX")
        self.param = 'param vide'
        #self.device = 'cuda'

        #try:
        #    self.param = rospkg.RosPack().get_path("img_classification") + '/param/' 
        #except:
        #    self.param = 'param vide'

        Model_name = rospy.get_param('/Albert_classification/ObjModel', 'none') 
        if (Model_name == 'none'):
            print ("------------------------------ No model name : ", Model_name)
            self.model = None
        else:
            print ("------------------------------ Model : ", Model_name, "forced to YOLO")
            self.model = YOLO("/home/jetson/ultralytics/ultralytics/yolo11n.pt")
         

        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.detect)
        rospy.on_shutdown(self.cancel)

    def cancel(self):
        '''self.pub_image.unregister()
        self.pub_msg.unregister()
        self.yolov5_wrapper.destroy()'''
        print("exit")

    def pub_imgMsg(self, frame):
        '''
        pic_base64 = base64.b64encode(frame)
        image = Image_Msg()
        size = frame.shape
        image.height = size[0]
        image.width = size[1]
        image.channels = size[2]
        image.data = pic_base64
        self.pub_image.publish(image)'''

    def detect(self,data):
        
        if rospy.is_shutdown():
            rospy.loginfo("Depth publisher node is not ready yet.")
            return

        Model_name = rospy.get_param('/Albert_classification/ObjModel', 'none')
        if (Model_name == 'none'):
            self.model = None
            return

        if (self.model == None):
            
            self.model = YOLO("/home/jetson/ultralytics/ultralytics/yolo11n.pt")
            print ("------------------------------ Model name : ", Model_name, "forced to YOLO")    
                

        Time1 = cv.getTickCount()
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)
        print('Starting detection ------------ on ' + self.device)

        results = model(frame)
        annotated_frame = results[0].plot()

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8"))
        except CvBridgeError as e:
            print(e)
        
        #category_name = self.model.config.id2label[class_id]
        Time2 = cv.getTickCount()

        d = (Time2 - Time1)/cv.getTickFrequency()
        print ("--------------------------- detection Time : ", d)

        
        rospy.sleep(2.0)


if __name__ == "__main__":
    print("Python version: ", sys.version)
    rospy.loginfo("init ObjDetect")
    rospy.init_node('obj_detection', anonymous=False)
    tracker = ObjDetect()
    rospy.loginfo("init ObjDetect done")
    rospy.spin()

