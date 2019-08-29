#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
#    self.image_pub = rospy.Publisher('image_topic_2',Image)
    # initialize CV Bridge 
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber('/robot/camera1/image_raw',Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    #Do whatever you want with the image here
    cv2.imshow("Window", cv_image)
    cv2.waitKey(1)

#    try:
#      image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
#    except CvBridgeError as e:
#      print(e)

def main():
  ic = image_converter()    # call the image convertor class
  rospy.init_node('image_converter', anonymous=True)
  rospy.spin()


if __name__ == '__main__':
  try:
     main()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()



