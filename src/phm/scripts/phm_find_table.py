#!/usr/bin/python

#####################################################
# Copyright (c) 2015-2017, Robological
# All rights reserved.
#
# This file is to use the Baxter robot infra-red 
# sensor to detect the location of the tig tag toe
# game playing table
#
# Note:
#       1. Before running this program, robot left
#          arm shall be manully put to a proper  
#          position (close to the middle of the table
#          10-30cm above the table)        
#####################################################

import cv2
import threading
import numpy as np
import os
import sys
from copy import deepcopy

import matplotlib.pylab as plt
import matplotlib.cm as cm
import rospy
import baxter_interface
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from std_msgs.msg import String
import message_filters
import time

import math

class GameTableFinder(object):

    def __init__(self, table_width, table_length, table_height):
        
        self.__ready = False
        
        self.table_width = table_width
        self.table_length = table_length
        self.table_height = table_height
        
        self.rb_cmd_pub = rospy.Publisher('robot_arms_cmd', String, queue_size=10)
        #self._left_ir_sensor  = rospy.Subscriber('/robot/range/left_hand_range/state' ,Range, callback=self._ir_sensor_callback, callback_args="left",queue_size=1)
        #self._right_ir_sensor = rospy.Subscriber('/robot/range/right_hand_range/state' ,Range, callback=self._ir_sensor_callback, callback_args="right",queue_size=1)
        left_ir_msg = message_filters.Subscriber('/robot/range/left_hand_range/state',Range)
        right_ir_msg = message_filters.Subscriber('/robot/range/right_hand_range/state',Range)
        ts = message_filters.ApproximateTimeSynchronizer([left_ir_msg, right_ir_msg], 10, 0.05)
        ts.registerCallback(self._ir_sensor_callback)
        
        self.current_ir_ranges = {'left':65.0, 'right':65.0}
        
    
    
    # Callback funtion for IR ranger data
    def _ir_sensor_callback(self, left_msg, right_msg): #def _ir_sensor_callback(self, msg, side):
        
        #print "\nLeft IR: \n", left_msg.range
        self.current_ir_ranges={'left':left_msg.range, 'right':right_msg.range}
        return
    
    
    
    # Check if it's ready to find table
    def is_ready():
        
        return self.__ready
        
    
    # Adjust height of the arms for table finding
    # height: the height of ir sensor above the table (must be in between 4~40cm or min_ir_range~max_ir_range)
    
    def adjust_height(self, height):
        # To DO: height error checking
        #if self.current_ir_ranges['left']>0.5 or self.current_ir_ranges['left']<0.0:
            #return
            
        while self.current_ir_ranges['left']>height:
            msg_string = 'left:move:0.0,0.0,-0.02,0.0,0.0,0.0,0.0'
            self.rb_cmd_pub.publish(msg_string)
            rospy.loginfo(msg_string)
            print self.current_ir_ranges
            time.sleep(3)
            
            
        return
        
    # scan to the left to find edge of table
    # init_height: set height of arm endpoint above the table
       
    def find_left_edge(self, init_height):
    
        #To Do: height error checking
        cur_ir_range = self.current_ir_ranges['left']
        
        while abs(cur_ir_range-init_height)<0.2:
        
            msg_string = 'left:move:0.0,0.02,0.0,0.0,0.0,0.0,0.0'
            self.rb_cmd_pub.publish(msg_string)
            print cur_ir_range
            cur_ir_range = self.current_ir_ranges['left']
            time.sleep(2)
    
        pass


    def find_back_edge(self, init_height):
    
        msg_string = 'left:move:0.0,-0.1,0.0,0.0,0.0,0.0,0.0'
        self.rb_cmd_pub.publish(msg_string)
        time.sleep(2)
        
        cur_ir_range = self.current_ir_ranges['left']
        while abs(cur_ir_range-init_height)<0.2:
        
            msg_string = 'left:move:-0.02,0.0,0.0,0.0,0.0,0.0,0.0'
            self.rb_cmd_pub.publish(msg_string)
            print cur_ir_range
            cur_ir_range = self.current_ir_ranges['left']
            time.sleep(2)
                




def main():

    gtf = GameTableFinder(1.0, 0.5, 0.9)
    rospy.init_node('tablefinder',anonymous = True)
    gtf.adjust_height(0.35)
    gtf.find_left_edge(0.35)
    gtf.find_back_edge(0.35)
    while not rospy.is_shutdown():
        
        
        
        rospy.sleep(0.1)



if __name__ == '__main__':
    sys.exit(main())
