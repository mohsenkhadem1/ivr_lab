#!/usr/bin/env python

import rospy
from std_msgs.msg import Float
import numpy as np
import sys
import roslib
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# define topic
pub = rospy.Publisher('position', Float, queue_size=10)


def callback(joints):
	t0 = rospy.get_time();
	rate = rospy.Rate(10) # 10hz

	pos= Float32MultiArray()
	pos.data = 1000*shape[-1,:]
	rospy.loginfo(pos)
	pub.publish(pos)
        rospy.loginfo(rospy.get_time()-t0)
	rate.sleep()  
#    
def ctr():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('ctr', anonymous=True)
    rospy.Subscriber("joint_configuration", Float, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
  

if __name__ == '__main__':
    try:
        ctr()
    except rospy.ROSInterruptException:
        pass
