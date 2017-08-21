#!/usr/bin/python

########################################################
# Copyright (c) 2015-2017, Robological
# All rights reserved.
#
# This file is to capture ROIs from an image
# as data set
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


class ImageRoiCollector(object):
    
    def __init__(self):
        
        pass
    
    
    
    
    
    
    
class BaxterImageGrabber(object):
    
    def __init__(self, img_width, img_height, side):
        
        
        
        self.cur_img = None
        self.OcvBridge = CvBridge()
        self.height = img_height
        self.width = img_width
        camera_name = side + '_hand_camera'
        self._camera = baxter_interface.CameraController(camera_name)
        self._camera.open()
        self._camera.resolution = [self.width, self.height]
        self._camera.gain = 10
        self.ImageThreadLock = threading.Lock()
        camera_topic_name = '/cameras/' + side + '_hand_camera/image'
        camera_sub = rospy.Subscriber(camera_topic_name, \
                                      Image, self._camera_callback)
        pass
        
    
    def _camera_callback(self, image):
        with self.ImageThreadLock:
            try:
                self.cur_img = self.OcvBridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
                #cv_img = deepcopy(self.cur_img)
            except Exception:
                
                print 'OH NO - IMAGE WENT WRONG!!'
        
            
    def get_image(self):
        
        return deepcopy(self.cur_img)


def signal_handler(signal, frame):
    print 'You pressed Ctrl+C!'
    sys.exit(0)
    

def mouse_callback(event,x,y,flags,param):
    
    pass
    
def main():
    
    signal.signal(signal.SIGINT, signal_handler)
    
    rospy.init_node("data_collection")
    bg = BaxterImageGrabber(960, 600, 'left')

    print "\nStart image capturing\n"
    while not rospy.is_shutdown():
        
        img = bg.get_image()
        
        if img != None:
            cv2.imshow('image',img)
            k = cv2.waitKey(1) & 0xFF
            if k == ord('m'):
                pass
            elif k == 27:
                print "\nExit\n"
                break
        

        
        
        rospy.sleep(0.1)
    
if __name__ == '__main__':
    
    try:
        main()
    except KeyboardInterrupt:
        print "\nCtrl-C Pressed\n"
        flag = True
        sys.exit()
    
    
    
    
    
    
