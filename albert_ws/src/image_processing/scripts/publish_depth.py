#!/usr/bin/env python
# -*- coding: utf-8 -*-


import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np



class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/camera/colored_depth/image_raw",Image, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/depth/image_raw",Image,self.callback)

  def callback(self,data):
    if rospy.is_shutdown():
        rospy.loginfo("Depth publisher node is not ready yet.")
        return

    try:
      frame = self.bridge.imgmsg_to_cv2(data, "passthrough")
    except CvBridgeError as e:
      print(e)

    # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
    # and publish the depth image
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(frame, alpha=0.03), cv2.COLORMAP_JET)
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(depth_colormap, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('albert_depth_publisher', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)