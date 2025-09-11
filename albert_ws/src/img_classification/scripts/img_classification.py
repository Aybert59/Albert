#!/usr/bin/env python3
# encoding: utf-8
import base64
import sys
import time
import rospy
import rospkg
import cv2 as cv
from yahboomcar_msgs.msg import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from img_classification.msg import detection

class ImgCL:
    def __init__(self):
        rospy.on_shutdown(self.cancel)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.detect)
        self.detected_pub = rospy.Publisher("/classification/detected",detection, queue_size=6)
        self.pTime = self.cTime = 0
        self.device = rospy.get_param("~device", "OrinNX")
        self.param = 'param vide'
        try:
            self.param = rospkg.RosPack().get_path("img_classification") + '/param/' 
        except:
            self.param = 'param vide'

        '''file_yaml = param_ + 'trash.yaml'
        PLUGIN_LIBRARY = param_ + device + "/libmyplugins.so"
        engine_file_path = param_ + device + "/yolov5s.engine"
        self.yolov5_wrapper = YoLov5TRT(file_yaml, PLUGIN_LIBRARY, engine_file_path)
        self.pub_image = rospy.Publisher('Detect/image_msg', Image_Msg, queue_size=10)
        self.pub_msg = rospy.Publisher('DetectMsg', TargetArray, queue_size=10)'''

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
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as e:
            print(e)
        #print('------------ ' + self.param + ' ------------' + self.device)


        
        detect_msg = detection()
        detect_msg.room = "unknown"
        detect_msg.probability = 100.0

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

if __name__ == "__main__":
    print("Python version: ", sys.version)
    rospy.loginfo("init ImgCL")
    rospy.init_node('img_classification', anonymous=False)
    tracker = ImgCL()
    rospy.loginfo("init ImgCL done")
    rospy.spin()
    '''
    capture = cv.VideoCapture(0)
    capture.set(6, cv.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    capture.set(cv.CAP_PROP_FRAME_WIDTH, 640)
    capture.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
    print("capture get FPS : ", capture.get(cv.CAP_PROP_FPS))
    detect = YoloDetect()
    while capture.isOpened():
        ret, frame = capture.read()
        action = cv.waitKey(1) & 0xFF
        frame = detect.detect(frame)
        if action == ord('q'): break
        cv.imshow('frame', frame)
    detect.cancel()
    capture.release()
    cv.destroyAllWindows()'''
