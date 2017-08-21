#!/usr/bin/python

########################################################
# Copyright (c) 2015-2017, Robological
# All rights reserved.
#
# This file is to use the Baxter robot left arm camera
# to find tig tag toe grid position
# 
#
#         
########################################################

import cv2
import threading
import numpy as np
import os
import sys
from cv_bridge import CvBridge
from copy import deepcopy

import matplotlib.pylab as plt
import matplotlib.cm as cm

import math

import baxter_interface
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from baxter_core_msgs.msg import EndpointState
import message_filters
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import String
import rospy
import signal



class CameraCalibrator(object):
    
    def __init__(self):
        
        self.UsingSavedImage = False
        self.ImageCounter = 0
        self.cur_img = None                             
        self.ImageThreadLock = threading.Lock()
        self.current_poses = None
        self.OcvBridge = CvBridge()
   
        left_camera_sub = rospy.Subscriber('/cameras/left_hand_camera/image', \
                                       Image, self._camera_callback)
                                    
        self.height = 600
        self.width = 960
        self._camera = baxter_interface.CameraController('left_hand_camera')
        
        self._camera.open()
        self._camera.resolution = [self.width, self.height]
        self._camera.gain = 16
        #self.cam_calib    = 0.0025                     # meters per pixel at 1 meter
        #self.cam_x_offset = 0.045                      # camera gripper offset
        #self.cam_y_offset = -0.01
        self.ImageFolder = "/home/robo-01/ros_ws/src/phm/camera_calibration/"
        
        
        
        left_ir_msg = message_filters.Subscriber('/robot/range/left_hand_range/state',Range)
        right_ir_msg = message_filters.Subscriber('/robot/range/right_hand_range/state',Range)
        ts = message_filters.ApproximateTimeSynchronizer([left_ir_msg, right_ir_msg], 10, 0.05)
        ts.registerCallback(self._ir_sensor_callback)
        
        self.current_ir_ranges = {'left':65.0, 'right':65.0}
        
        left_arm_msg = message_filters.Subscriber("/robot/limb/left/endpoint_state",EndpointState)
        right_arm_msg = message_filters.Subscriber("/robot/limb/right/endpoint_state",EndpointState)
        ts = message_filters.ApproximateTimeSynchronizer([left_arm_msg, right_arm_msg], 10, 0.05)
        ts.registerCallback(self._pose_callback)
    
        self.current_poses
    
    def get_ir_range(self, side):
        range = self.current_ir_ranges[side]
        return range
    
    def image_from_disk(self, flag):
        self.UsingSavedImage = flag
        
        return self.UsingSavedImage
    def _pose_callback(self, left_msg, right_msg):
    
        pose1 = left_msg.pose
        pose2 = right_msg.pose
        cp2 = [pose2.position.x, \
               pose2.position.y, \
               pose2.position.z, \
               pose2.orientation.x, \
               pose2.orientation.y, \
               pose2.orientation.z, \
               pose2.orientation.w]
            
        cp1 = [pose1.position.x, \
               pose1.position.y, \
               pose1.position.z, \
               pose1.orientation.x, \
               pose1.orientation.y, \
               pose1.orientation.z, \
               pose1.orientation.w]
            
        
        self.current_poses = {'left':cp1, 'right':cp2}
        
        
        
        #print "\nleft: \n", cp1, "\n\nnew left: \n", cp11
            
        return
    
    def _ir_sensor_callback(self, left_msg, right_msg): #def _ir_sensor_callback(self, msg, side):
        
        #print "\nLeft IR: \n", left_msg.range
        self.current_ir_ranges={'left':left_msg.range, 'right':right_msg.range}
        return
    
    def _camera_callback(self, image):
        with self.ImageThreadLock:
            try:
                self.cur_img = self.OcvBridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
                #cv_img = deepcopy(self.cur_img)
            except Exception:
                
                print 'OH NO - IMAGE WENT WRONG!!'
        
    def get_image(self):
        c_img = None
        with self.ImageThreadLock:
            c_img = deepcopy(self.cur_img)
        
        return c_img
    
    def make_pose_stamp(self, pose_list, header):
        
        ps = PoseStamped()
        ps.header = header
        ps.pose.position = Point(x = pose_list[0], \
                                 y = pose_list[1], \
                                 z = pose_list[2], )

        ps.pose.orientation = Quaternion(x = pose_list[3], \
                                         y = pose_list[4], \
                                         z = pose_list[5], \
                                         w = pose_list[6], )
        
        return ps
    
    def data_collection(self, side):
        c_img = deepcopy(self.cur_img)
        c_range = self.current_ir_ranges[side]
        c_pose = self.current_poses[side]
        output_list = [c_range, c_pose[0], c_pose[1], c_pose[2], c_pose[3], c_pose[4], c_pose[5], c_pose[6]]
        #output_list.append(c_pose)
        #output_list.append(c_range)
        img_filename = side+'_image' + str(self.ImageCounter) + '.png'
        self.ImageCounter = self.ImageCounter + 1
        output_list.append(img_filename)
        full_img_filename = self.ImageFolder + img_filename
        cv2.imwrite(full_img_filename, c_img)
        file = open("/home/robo-01/ros_ws/src/phm/camera_calibration/index.txt", "a")
        for item in output_list:
            file.write("%s," % item)
        file.write('\n')
        file.close()
        pass
    
        
    

def signal_handler(signal, frame):
    print 'You pressed Ctrl+C!'
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    
    rospy.init_node("phm_camera_calibration")
    cc = CameraCalibrator()
    
    key = 0
    while not rospy.is_shutdown():
        
        img = cc.get_image()
        if img != None:
            cv2.imshow('current_image', img)
            key = cv2.waitKey(10)
            if key == 13:
                cc.data_collection('left')
                print "Collect one date"
    

if __name__ == '__main__':
    
    try:
        main()
    except KeyboardInterrupt:
        print "\nCtrl-C Pressed\n"
        flag = True
        sys.exit()
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        