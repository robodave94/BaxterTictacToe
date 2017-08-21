#!/usr/bin/python

########################################################
# Copyright (c) 2015-2017, Robological
# All rights reserved.
#
# This file is to display image on Baxter robot 
#
# Head Monitor
#
#         
########################################################



import cv2
import threading
import numpy as np
import os
import sys
import time
from cv_bridge import CvBridge
from copy import deepcopy

from sensor_msgs.msg import (
    Image,
)



import matplotlib.pylab as plt
import matplotlib.cm as cm

import math

import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from baxter_core_msgs.msg import EndpointState
from baxter_core_msgs.msg import AssemblyState
#from baxter_interface import DigitalIO
from baxter_core_msgs.msg import DigitalIOState
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
from random import randint


class Baxter_Display(object):
    
    def __init__(self):
        
        self.DisplayCmdString = ''
        
        rospy.init_node("display_image")
        
        rospy.Subscriber('display_cmd', String, self.message_callback)
        
        self.CmdString = {'waitforbutton':'17.png', \
                          'estop':'14.png', \
                          'robotwin':'03.png', \
                          'humanwin':'05.png', \
                          'draw':'06.png', \
                          'running':'02.png', \
                          'scang00':'15_02.png', \
                          'scang01':'15_03.png', \
                          'scang02':'15_04.png', \
                          'scang03':'15_05.png', \
                          'scang04':'15_06.png', \
                          'scang05':'15_07.png', \
                          'scang06':'15_08.png', \
                          'scang07':'15_09.png', \
                          'scang08':'15_10.png', \
                          'scanr00':'15_11.png', \
                          'scanr01':'15_12.png', \
                          'scanr02':'15_13.png', \
                          'scanr03':'15_14.png', \
                          'scanr04':'15_15.png', \
                          'scanl00':'15_16.png', \
                          'scanl01':'15_17.png', \
                          'scanl02':'15_18.png', \
                          'scanl03':'15_19.png', \
                          'scanl04':'15_20.png', \
                          'cleang00':'16_02.png', \
                          'cleang01':'16_03.png', \
                          'cleang02':'16_04.png', \
                          'cleang03':'16_05.png', \
                          'cleang04':'16_06.png', \
                          'cleang05':'16_07.png', \
                          'cleang06':'16_08.png', \
                          'cleang07':'16_09.png', \
                          'cleang08':'16_10.png', \
                          'resettable':'16_01.png', \
                          'manualresettable':'18.png', \
                          'myturn':'09.png', \
                        }
        self.Images = {}
        pass
        
    def message_callback(self, msg):
        
        self.DisplayCmdString = msg.data
        
        return
    
    def get_display_cmd(self):
        
        cur_cmd = self.DisplayCmdString
        
        while cur_cmd == '':
            
            cur_cmd = self.DisplayCmdString
            rospy.sleep(0.1)
            
        self.DisplayCmdString = ''
        
        return cur_cmd
    
    def interpret_message(self, msg_string):
        
        if msg_string == '':
            return
        
        cmd_segments = msg_string.split()
        
        cmd_string = ''
        flash_string = ''
        
        if len(cmd_segments)==1:
            cmd_string = msg_string
            flash_string = 'off'
            
        elif len(cmd_segments)==2:
            
            cmd_string = msg_string
            flash_string = 'on'
            
        else:
            print "Incorrect Display Command String Segments..."
                    
        return cmd_string, flash_string 
    
    def load_images(self):
        
        image_folder = '/home/ruser/ros_ws/src/phm/images/'
        keys = self.CmdString.keys()
        for item in keys:
            image_file = image_folder + self.CmdString[item]
            img = cv2.imread(image_file)
            self.Images.update({item:img})
            print "item"
            
        
        return
    
    def display_image(self, image_tag):
        """
        Send the image located at the specified path to the head
        display on Baxter.

        @param path: path to the image file to load and send
        """
        
        if image_tag not in self.Images.keys():
            print "Image Name not in List..."
            return
        img = self.Images[image_tag]
        msg = CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
        pub.publish(msg)
        # Sleep to allow for image to be published.
        #rospy.sleep(1)
        
    def run(self):
        
        self.load_images()
        print "All Images Loaded..."
        
        counter = 0
        
        while not rospy.is_shutdown():
            
            cur_cmd = self.get_display_cmd()
            
            if cur_cmd == '':
                rospy.sleep(0.1)
                continue

            image_tag, flash_tag = self.interpret_message(cur_cmd)
            print "Command: ", image_tag
            
            if image_tag == '':
                rospy.sleep(0.1)
                continue
            
            self.display_image(image_tag)
            
            rospy.sleep(0.1)
        
        
        
        
        return




def signal_handler(signal, frame):
    print 'You pressed Ctrl+C!'
    sys.exit(0)
   
def main():

    signal.signal(signal.SIGINT, signal_handler)
    display = Baxter_Display()
    
    display.run()
    

if __name__ == '__main__':
    
    try:
        main()
    except KeyboardInterrupt:
        print "\nCtrl-C Pressed\n"
        flag = True
        sys.exit()