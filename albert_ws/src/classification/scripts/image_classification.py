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

import torch
from classification.msg import detection 
from transformers import AutoImageProcessor, AutoModelForImageClassification    #### tensorflow semble bien installé. Cf comment installer transformers
from cv_bridge import CvBridge, CvBridgeError

class ImgCL:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.bridge = CvBridge()
        
        self.detected_pub = rospy.Publisher("/classification/detected",detection, queue_size=6)
        self.pTime = self.cTime = 0
        self.device = rospy.get_param("~device", "OrinNX")
        self.param = 'param vide'

        self.device = 'cuda'

        try:
            self.param = rospkg.RosPack().get_path("img_classification") + '/param/' 
        except:
            self.param = 'param vide'

        Model_name = rospy.get_param('/Albert_classification/Model', 'none') 
        if (Model_name == 'none'):
            print ("------------------------------ No model name : ", Model_name)
            self.model = None
        else:
            HF_name = self.param + Model_name
        
            print ("------------------------------ Model name : ", HF_name)    
            self.preprocess = AutoImageProcessor.from_pretrained(HF_name)
            self.model = AutoModelForImageClassification.from_pretrained(HF_name)
            self.model.to(self.device)
            self.model.eval()     
         

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
        
        Model_name = rospy.get_param('/Albert_classification/Model', 'none')
        if (Model_name == 'none'):
            self.model = None
            return

        if (self.model == None):
            HF_name = self.param + Model_name
        
            print ("------------------------------ Model name : ", HF_name)    
            self.preprocess = AutoImageProcessor.from_pretrained(HF_name)
            self.model = AutoModelForImageClassification.from_pretrained(HF_name)
            self.model.to(self.device)
            self.model.eval()    

        Time1 = cv.getTickCount()
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)
        print('Starting detection ------------ on ' + self.device)

        batch = self.preprocess (frame, return_tensors='pt').to(self.device)
        with torch.no_grad():
            prediction = self.model(**batch)
        class_id = prediction.logits.argmax().item()
        score = prediction.logits.softmax(1)[0][class_id].item()
        
        category_name = self.model.config.id2label[class_id]
        Time2 = cv.getTickCount()

        d = (Time2 - Time1)/cv.getTickFrequency()
        print ("--------------------------- detection Time : ", d)

        detect_msg = detection()
        detect_msg.room = category_name
        detect_msg.probability = score

        self.detected_pub.publish (detect_msg)
        
        '''
        target_array = TargetArray()
        target = Target()
        frame, result_boxes, result_scores, result_classid = self.yolov5_wrapper.infer(frame)
        # Draw rectangles and labels on the original image
        for j in range(len(result_boxes)):
            box = result_boxes[j]
            self.yolov5_wrapper.plot_one_box(
                box,
                frame,
                label="{}:{:.2f}".format(
                    self.yolov5_wrapper.categories[int(result_classid[j])],
                    result_scores[j]
                ),
            )
            target.frame_id = self.yolov5_wrapper.categories[int(result_classid[j])]
            target.stamp = rospy.Time.now()
            target.scores = result_scores[j]
            # x1, y1, x2, y2
            target.ptx = box[0]
            target.pty = box[1]
            target.distw = box[2] - box[0]
            target.disth = box[3] - box[1]
            target.centerx = (box[2] - box[0]) / 2
            target.centery = (box[3] - box[1]) / 2
            target_array.data.append(target)
        self.cTime = time.time()
        fps = 1 / (self.cTime - self.pTime)
        self.pTime = self.cTime
        text = "FPS : " + str(int(fps))
        cv.putText(frame, text, (20, 30), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 1)
        self.pub_msg.publish(target_array)
        print(target_array)
        self.pub_imgMsg(frame)
        return frame'''

        rospy.sleep(2.0)


if __name__ == "__main__":
    print("Python version: ", sys.version)
    rospy.loginfo("init ImgCL")
    rospy.init_node('img_classification', anonymous=False)
    tracker = ImgCL()
    rospy.loginfo("init ImgCL done")
    rospy.spin()

