#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from cv_bridge import CvBridge

def gotrgb(rgbimage):
	imagergb = bridge.imgmsg_to_cv2(rgbimage, desired_encoding="bgr8")
	imagergb = imagergb[::2,::2]
	rgb_pub.publish( bridge.cv2_to_imgmsg(imagergb) )

def gotdepth(depthimage):
	imagedepth = bridge.imgmsg_to_cv2(depthimage, desired_encoding="passthrough")
	imagedepth2 = imagedepth[::2,::2]
	
	imagedepth2 = np.where(np.isnan(imagedepth2), np.ma.array(imagedepth2, mask = np.isnan(imagedepth2)).mean(axis=0), imagedepth2)	
	for i in range(len(imagedepth2)):
		for j in range(len(imagedepth2[1])):
			if (imagedepth2[i,j] == 0):
				imagedepth2[i,j] = np.amax(imagedepth2)	

			imagedepth2[i,j] = imagedepth2[i,j]*25.951
			if (imagedepth2[i,j] > 255):
				imagedepth2[i,j] = 255

	depth_pub.publish( bridge.cv2_to_imgmsg(imagedepth2) )


if __name__ == '__main__':
	rgb_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, gotrgb)
	depth_sub = rospy.Subscriber('/camera/depth_registered/image_raw', Image, gotdepth)
	bridge = CvBridge()
	
	rgb_pub = rospy.Publisher('/camera/rgb/image_raw_downsampled', Image, queue_size=1)
	depth_pub = rospy.Publisher('/camera/depth_registered/image_raw_downsampled', Image, queue_size=1)
	rospy.init_node('image_downsampler', anonymous=True)
	rospy.spin()
