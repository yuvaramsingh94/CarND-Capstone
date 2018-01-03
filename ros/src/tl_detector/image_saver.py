#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class TLDetector(object):
    def __init__(self):
        rospy.init_node('saver')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.num = 214
        self.has_image = False
        self.cv_image = None
        self.bridge = CvBridge()

        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)
        while not rospy.is_shutdown():
        	self.save_image()

        	rospy.Rate(1).sleep()

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        

    def save_image(self):
    	#print('working')

    	if self.has_image:
    		self.num +=1
    		cv2.imwrite("/home/yuvaram/catkin_ws3/src/CarND-Capstone/images/"+str(self.num)+".png",self.cv_image)


if __name__ =='__main__':
	TLDetector()
