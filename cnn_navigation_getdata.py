#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Jun 26 13:07:09 2019

@author: sunny
"""


import numpy as np
import scipy as sp
import message_filters
import rospy
import cv2
from PIL import Image
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import pandas as pd



depthimages = []
rgbimages = []
mergedimage = np.zeros((240,320,4))
mergedimages = []
cmdvel = []
cmdvels = []
odom = []
odoms = []
localgoal = []
localgoals = []
count=0

odomlocalvector = []
odomlocalvectors = []


############################################################################################################################

def callback(image_rgb, image_depth, odometry, local_plan, cmd_vel):
	# Solve all of perception here...
   
    #image_rgb_pub.publish(image_rgb)
    #image_depth_pub.publish(image_depth)
    #odom_pub.publish(odometry)
    #local_goal_pub.publish(local_plan)
    #cmd_vel_pub.publish(cmd_vel)
    #var = image_depth
    #float_pub.publish(var)
    
    importdata(image_rgb, image_depth, odometry, local_plan, cmd_vel)
    
    
def importdata(image_rgb, image_depth, odometry, local_plan, cmd_vel):
    global count, depthimages, imaged, imaged2, imagergb, cmdvel, cmdvels, odom, test
    cmdvel = []
    #odom = []
    #localgoal = []
    odomlocalvector = []
    mergedimage = np.zeros((240,320,4))
    
    #rgb and depth images
    imagergb = bridge.imgmsg_to_cv2(image_rgb, desired_encoding="bgr8")
    imaged = bridge.imgmsg_to_cv2(image_depth, desired_encoding="passthrough")
    
    #downsample images
    imaged2 = imaged
    imaged2 = imaged2[::2,::2]
    imagergb = imagergb[::2,::2]
    
    #Remove NaN values from depthimages
    imaged2 = np.where(np.isnan(imaged2), np.ma.array(imaged2, 
               mask = np.isnan(imaged2)).mean(axis = 0), imaged2)
    for i in range(len(imaged2)):
        for j in range(len(imaged2[1])):
            if imaged2[i,j] == 0:
                imaged2[i,j] = np.amax(imaged2)
            
            imaged2[i,j] = imaged2[i,j]*25.951
            if imaged2[i,j] > 255:
                imaged2[i,j] = 255
    
    #depthimages.insert(count, imaged2[:,:])
    #rgbimages.insert(count, imagergb)
    mergedimage[:,:,0:3] = imagergb[:,:,:]
    mergedimage[:,:,3] = imaged2[:,:]
    #mergedimages.insert(count, mergedimage)
    
    #cmd_vel
    cmdvel.insert(0, cmd_vel.twist.linear.x)
    cmdvel.insert(1, cmd_vel.twist.angular.z)
    cmdvels.insert(count, np.array(cmdvel))
    
    
    #odom-local_goal vector
    odomlocalvector.insert(0, odometry.pose.pose.position.x-local_plan.poses[1].pose.position.x)
    odomlocalvector.insert(1, odometry.pose.pose.position.y-local_plan.poses[1].pose.position.y)
    odomlocalvector.insert(2, odometry.pose.pose.position.z-local_plan.poses[1].pose.position.z)
    odomlocalvector.insert(3, odometry.pose.pose.orientation.x-local_plan.poses[1].pose.orientation.x)
    odomlocalvector.insert(4, odometry.pose.pose.orientation.y-local_plan.poses[1].pose.orientation.y)
    odomlocalvector.insert(5, odometry.pose.pose.orientation.z-local_plan.poses[1].pose.orientation.z)
    odomlocalvector.insert(6, odometry.pose.pose.orientation.w-local_plan.poses[1].pose.orientation.w)
    odomlocalvectors.insert(count, np.array(odomlocalvector))
    
    #cv2.imwrite('/home/sunny/catkin_ws/src/sunny_robot/data/Actual/BGR_Images/bgrimage'+str(count)+'.png', imagergb) 
    #cv2.imwrite('/home/sunny/catkin_ws/src/sunny_robot/data/Actual/Depth_Images/depthimage'+str(count)+'.png', imaged2) 
    #cv2.imwrite('/home/sunny/catkin_ws/src/sunny_robot/data/Actual/Merged_Images/mergedimage'+str(count)+'.png', mergedimage)
    #np.savetxt('/home/sunny/catkin_ws/src/sunny_robot/data/Actual/Vectors/vectors0_'+str(count)+'.csv', np.array(odomlocalvectors), delimiter=",")
    #np.savetxt('/home/sunny/catkin_ws/src/sunny_robot/data/Actual/CmdVels/cmdvels0_'+str(count)+'.csv', np.array(cmdvels), delimiter=",")
    
    #cv2.imwrite('/media/sunny/SunnyHardDisk/data2/BGR_Images/bgrimage'+str(count)+'.png', imagergb) 
    #cv2.imwrite('/media/sunny/SunnyHardDisk/data2/Depth_Images/depthimage'+str(count)+'.png', imaged2) 
    cv2.imwrite('/home/isera2/catkin_sunny/src/sunny_pioneer/data/test3/Merged_Images/mergedimage'+str(22740+count)+'.png', mergedimage)
    #np.savetxt('/home/isera2/catkin_sunny/src/sunny_pioneer/data/test3/Vectors/vectors0_'+str(count)+'.csv', np.array(odomlocalvectors), delimiter=",")
    #np.savetxt('/home/isera2/catkin_sunny/src/sunny_pioneer/data/test3/CmdVels/cmdvels0_'+str(count)+'.csv', np.array(cmdvels), delimiter=",")
    
    print("\nSaved frame: "+str(count)+"!")
    count = count+1
############################################################################################################################

image_rgb_sub = message_filters.Subscriber('/camera/rgb/image_raw', Image)
image_depth_sub = message_filters.Subscriber('/camera/depth_registered/image_raw', Image)
odom_sub = message_filters.Subscriber('/RosAria/pose', Odometry)
local_goal_sub = message_filters.Subscriber('/move_base/EBandPlannerROS/global_plan', Path)
cmd_vel_sub = message_filters.Subscriber('/RosAria/cmd_vel_stamped', TwistStamped)
bridge = CvBridge()

#image_rgb_pub = rospy.Publisher('image_rgb_sync', Image, queue_size=1)
#image_depth_pub = rospy.Publisher('image_depth_sync', Image, queue_size=1)
#odom_pub = rospy.Publisher('odom_sync', Odometry, queue_size=1)
#local_goal_pub = rospy.Publisher('local_goal_sync', Path, queue_size=1)
#cmd_vel_pub = rospy.Publisher('cmd_vel_sync', TwistStamped, queue_size=1) 
#float_pub = rospy.Publisher('xvalue', Image, queue_size=1)
rospy.init_node('cnn_navigation_getdata', anonymous=True)

ts = message_filters.ApproximateTimeSynchronizer([image_rgb_sub, image_depth_sub, odom_sub, local_goal_sub, cmd_vel_sub], 1, 0.8)
ts.registerCallback(callback)
rospy.spin()
