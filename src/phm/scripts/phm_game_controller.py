#!/usr/bin/python

##########################################################
# Copyright (c) 2015-2017, Robological
# All rights reserved.
#
# This file is to use the Baxter robot to play Tig Tag Toe
# game (as a central controller)
#
#         
##########################################################

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



class TigTagToe(object):
    
    def __init__(self):
        
        self.GameArray = np.zeros((3,3,1), np.uint8)
        self.ArmCmdPub = rospy.Publisher('robot_arms_cmd', String, queue_size=10)
        self.VisionCmdPub = rospy.Publisher('robot_vision_cmd', String, queue_size=10)
        self.GridStatusPub = rospy.Publisher('game_engine_cmd', String, queue_size=10)
        self.DisplayCmdPub = rospy.Publisher('display_cmd', String, queue_size=10)
        self.VisionReply = ''
        self.ArmReply = ''
        self.NextMoveReply = ''
        self.LeftSlots = ['x','x','x','x','x']
        self.RightSlots = ['o','o','o','o','o']
        self.GridStatus = ['b','b','b','b','b','b','b','b','b']
        self.FirstPlayMarker = ''
        self.LeftSlotsLocation = []
        self.RightSlotsLocation = []
        rospy.init_node("game_controller")
        rospy.Subscriber('arm_reply', String, self.arms_reply_callback)
        rospy.Subscriber('vision_reply', String, self.vision_reply_callback)
        rospy.Subscriber('next_move', String, self.next_move_callback)
        rospy.Subscriber('/robot/digital_io/right_shoulder_button/state', DigitalIOState, self.button_callback)
        #rospy.Subscriber('/robot/digital_io/right_itb_button0/state', DigitalIOState, self.button_callback)
        rospy.Subscriber('/robot/state', AssemblyState, self.robot_state_callback)
        
        
        display_image_paths = {'running':'/home/ruser/ros_ws/src/phm/images/running.png', \
                                'estop':'/home/ruser/ros_ws/src/phm/images/estop.png', \
                                'waitforbutton':'/home/ruser/ros_ws/src/phm/images/waitforbutton.png', \
                                'leftwin':'/home/ruser/ros_ws/src/phm/images/baxter_left_won.png', \
                                'rightwin':'/home/ruser/ros_ws/src/phm/images/baxter_right_won.png', \
                                'draw':'/home/ruser/ros_ws/src/phm/images/baxter_draw.png', \
                                'leftplay':'/home/ruser/ros_ws/src/phm/images/baxter_left_play.png', \
                                'rightplay':'/home/ruser/ros_ws/src/phm/images/baxter_right_play.png', \
                                'resettable':'/home/ruser/ros_ws/src/phm/images/baxter_clean.png', \
                                }
                                
        img1 = cv2.imread(display_image_paths['running'])
        img2 = cv2.imread(display_image_paths['estop'])
        img3 = cv2.imread(display_image_paths['waitforbutton'])
        img4 = cv2.imread(display_image_paths['leftwin'])
        img5 = cv2.imread(display_image_paths['rightwin'])
        img6 = cv2.imread(display_image_paths['draw'])
        img7 = cv2.imread(display_image_paths['leftplay'])
        img8 = cv2.imread(display_image_paths['rightplay'])
        img9 = cv2.imread(display_image_paths['resettable'])
        
        self.display_images = {'running':img1, 'estop':img2, 'waitforbutton':img3, \
                                'leftwin':img4, 'rightwin':img5, 'draw':img6, \
                                'leftplay':img7, 'rightplay':img8, 'resettable':img9}
        print "Display Image Loaded"
        
        
        self.ButtonStatus = 0
        self.EstopStatus = 0
        self.GridCenter = []
        self.TableHeight = 0.0
        self.GripperLength = 0.125
        self.LeftArmInitPose = [0.4, 0.65, 0.15, 0.0, 1.0, 0.0, 0.0]
        self.RightArmInitPose = [0.4, -0.65, 0.15, 0.0, 1.0, 0.0, 0.0]
        self.current_poses = None
        self.GridLocations = []
        self.GridRoiLocations = []
        self.GridRoiPose = []
        self.TableZ = 100.0
        self.BlockHeight = 0.03
        self.GameState = ''
        self.GameError = ''
        
        #rs = baxter_interface.RobotEnable(CHECK_VERSION)
        
        self.head = baxter_interface.Head()
        self.head.set_pan(0.0)
        
        left = baxter_interface.Gripper('left', CHECK_VERSION)
        right = baxter_interface.Gripper('right', CHECK_VERSION)
        self.baxter_grippers = {'left':left, 'right':right}
        
        left_arm = baxter_interface.Limb("left")
        right_arm = baxter_interface.Limb("right")
        self.arms = {'left':left_arm, 'right':right_arm}
        
        pose_left_front = [0.6085+0.01, 0.2332, -0.0263, 0.05, 0.9999, -0.0005, 0.0002]
        pose_right_front = [0.6076, -0.2891, -0.0254, 0.01, 0.9999, 0.0003, -0.0011]
        
        pose_left_back = [0.3871+0.01, 0.2413, -0.0252, 0.04, 0.9999, 0.0005, -0.0012]
        pose_right_back = [0.3910, -0.2889, -0.0247, 0.01, 0.9999, -0.0003, 0.0005]
        
        lb = pose_left_back
        lf = pose_left_front
        
        self.LeftSlotsLocation = [ \
                                    [lb[0], lb[1], lb[2], lb[3], lb[4], lb[5], lb[6]], \
                                    [(lf[0]+lb[0])/2+0.005, (lf[1]+lb[1])/2, (lf[2]+lb[2])/2, lf[3], lf[4], lf[5], lf[6]], \
                                    [lf[0]+0.005, lf[1], lf[2], lf[3], lf[4], lf[5], lf[6]], \
                                    [(lf[0]+lb[0])/2+0.01, (lf[1]+lb[1])/2+0.11, (lf[2]+lb[2])/2, lf[3], lf[4], lf[5], lf[6]], \
                                    [lf[0]+0.01, lf[1]+0.11, lf[2], lf[3], lf[4], lf[5], lf[6]], \
                                    
                                    
                                    
                                ]
        
        self.LeftSlotsLocation1 = self.LeftSlotsLocation
        
        lb = pose_right_back
        lf = pose_right_front

        
        self.RightSlotsLocation = [ \
                                    [lb[0]+0.005, lb[1], lb[2], lb[3], lb[4], lb[5], lb[6]], \
                                    [(lf[0]+lb[0])/2+0.005, (lf[1]+lb[1])/2, (lf[2]+lb[2])/2, lf[3], lf[4], lf[5], lf[6]], \
                                    [lf[0]+0.008, lf[1]-0.005, lf[2], lf[3], lf[4], lf[5], lf[6]], \
                                    [(lf[0]+lb[0])/2+0.003, (lf[1]+lb[1])/2-0.11-0.002, (lf[2]+lb[2])/2, lf[3], lf[4], lf[5], lf[6]], \
                                    [lf[0]+0.005, lf[1]-0.11-0.006, lf[2], lf[3], lf[4], lf[5], lf[6]], \
                                    
                                    
                                    
                                ]
        
        self.RightSlotsLocation1 = self.RightSlotsLocation
        
        
        
        lf = pose_left_front
        rf = pose_right_front
        
        lb = pose_left_back
        rb = pose_right_back
        
        center_x = (rb[0]+lb[0]+rf[0]+lf[0])/4
        center_y = (rb[1]+lb[1]+rf[1]+lf[1])/4
        center_z = (rb[2]+lb[2]+rf[2]+lf[2])/4
        center_ox = (rb[3]+lb[3]+rf[3]+lf[3])/4
        center_oy = (rb[4]+lb[4]+rf[4]+lf[4])/4
        center_oz = (rb[5]+lb[5]+rf[5]+lf[5])/4
        center_ow = (rb[6]+lb[6]+rf[6]+lf[6])/4
        
        self.GridForLeftArm = [ \
                                [center_x-0.11-0.01, center_y-0.11,center_z, center_ox, center_oy,center_oz,center_ow ], \
                                [center_x-0.005, center_y-0.11-0.005,center_z, center_ox, center_oy,center_oz,center_ow ], \
                                [center_x+0.11-0.01, center_y-0.11-0.012,center_z, center_ox, center_oy,center_oz,center_ow ], \
                                [center_x-0.11-0.005, center_y, center_z, center_ox, center_oy,center_oz,center_ow ], \
                                [center_x, center_y-0.007, center_z, center_ox, center_oy, center_oz, center_ow ], \
                                [center_x+0.11, center_y-0.012, center_z, center_ox, center_oy,center_oz,center_ow ], \
                                [center_x-0.11, center_y+0.11-0.002,center_z, center_ox, center_oy,center_oz,center_ow ], \
                                [center_x, center_y+0.11-0.007,center_z, center_ox, center_oy,center_oz,center_ow ], \
                                [center_x+0.11+0.003, center_y+0.11-0.01,center_z, center_ox, center_oy,center_oz,center_ow ], \
                                
                              ]
                            
        
        self.GridForLeftArm1 = self.GridForLeftArm
        
        
        self.GridForRightArm = [ \
                                [center_x-0.11+0.003, center_y-0.11+0.012,center_z, center_ox, center_oy,center_oz,center_ow ], \
                                [center_x+0.003, center_y-0.11+0.005,center_z, center_ox, center_oy,center_oz,center_ow ], \
                                [center_x+0.11, center_y-0.11+0.003,center_z, center_ox, center_oy,center_oz,center_ow ], \
                                [center_x-0.11, center_y+0.013, center_z, center_ox, center_oy,center_oz,center_ow ], \
                                [center_x+0.002, center_y+0.008, center_z, center_ox, center_oy, center_oz, center_ow ], \
                                [center_x+0.11, center_y+0.003, center_z, center_ox, center_oy,center_oz,center_ow ], \
                                [center_x-0.11, center_y+0.11+0.015,center_z, center_ox, center_oy,center_oz,center_ow ], \
                                [center_x+0.003, center_y+0.11+0.01,center_z, center_ox, center_oy,center_oz,center_ow ], \
                                [center_x+0.11+0.005, center_y+0.11+0.002,center_z, center_ox, center_oy,center_oz,center_ow ], \
                                
                              ]
        
        self.GridForRightArm1 = self.GridForRightArm
        
                                    
                                    
        '''
        #pick
        self.LeftSlotsLocation = [ [0.3951+0.007, 0.2694, -0.080, 0.0147, 0.9999, -0.0085, -0.0051], \
                                    [0.5089, 0.2679, -0.080, -0.0001, 0.9996, -0.0093, -0.0243], \
                                    [0.6177+0.003, 0.2716-0.002, -0.080, 0.0166, 0.9995, -0.0033, -0.0266], \
                                    [0.5031+0.003, 0.3795+0.003, -0.080, 0.0153, 0.9998, -0.0091, -0.0435], \
                                    [0.6150+0.006, 0.3809, -0.080, 0.0033, 0.9998, 0.0030, -0.0086] ]
        #place                            
        self.LeftSlotsLocation1 = [ [0.3951+0.004, 0.2694, -0.080, 0.0147, 0.9999, -0.0085, -0.0051], \
                                    [0.5089-0.005, 0.2679+0.005, -0.080, -0.0001, 0.9996, -0.0093, -0.0243], \
                                    [0.6177-0.013, 0.2716+0.003, -0.080, 0.0166, 0.9995, -0.0033, -0.0266], \
                                    [0.5031-0.002, 0.3795, -0.080, 0.0153, 0.9998, -0.0091, -0.0435], \
                                    [0.6150+0.008, 0.3809+0.002, -0.080, 0.0033, 0.9998, 0.0030, -0.0086] ]
        #pick
        self.RightSlotsLocation = [ [0.4040+0.015, -0.2569-0.002, -0.076, -0.0206, 0.9997, 0.0018, 0.0164], \
                                    [0.5131+0.016, -0.2545-0.001, -0.076, -0.0057, 0.9998, 0.0029, 0.0198], \
                                    [0.6174+0.015, -0.2559-0.001, -0.076, -0.0179, 0.9996, -0.0142, -0.0104], \
                                    [0.5224+0.008, -0.3682+0.002, -0.076, -0.0186, 0.9997, 0.0058, -0.0127], \
                                    [0.6285+0.012, -0.3660+0.001, -0.076, 0.0031, 0.9998, 0.0031, 0.0191] ]
        #place                            
        self.RightSlotsLocation1 = [ [0.4040+0.005, -0.2569-0.002, -0.076, -0.0206, 0.9997, 0.0018, 0.0164], \
                                    [0.5131+0.01, -0.2545-0.003, -0.076, -0.0057, 0.9998, 0.0029, 0.0198], \
                                    [0.6174+0.015, -0.2559, -0.076, -0.0179, 0.9996, -0.0142, -0.0104], \
                                    [0.5224, -0.3682, -0.076, -0.0186, 0.9997, 0.0058, -0.0127], \
                                    [0.6285+0.005, -0.3660-0.008, -0.076, 0.0031, 0.9998, 0.0031, 0.0191] ]
        #pick
        self.GridForLeftArm = [ [0.3965, -0.1069+0.004, -0.076, 0.03617, 0.9991, -0.0098, -0.0198], \
                                [0.5040+0.01, -0.1058-0.002, -0.076, 0.0147, 0.9994, 0.0002, -0.0316], \
                                [0.6115+0.007, -0.0908-0.022, -0.076, 0.0146, 0.9999, 0.0064, -0.0047], \
                                [0.3917+0.004, 0.0117-0.01, -0.076, 0.0073, 0.9993, 0.0042, -0.0364], \
                                [0.4983+0.009, 0.0108-0.01, -0.076, -0.0066, 0.9999, 0.0015, -0.0127], \
                                [0.6093+0.013, 0.0097-0.011, -0.076, 0.0154, 0.9997, 0.0029, -0.0194], \
                                [0.3914+0.004, 0.1200-0.01, -0.076, 0.0126, 0.9994, -0.0002, -0.0333], \
                                [0.5029+0.006, 0.1203-0.011, -0.076, 0.0020, 0.9993, 0.0008, -0.0378], \
                                [0.6103+0.01, 0.1161-0.006, -0.076, 0.0195, 0.9992, -0.0010, -0.0352], \
        
                                ]
        #place                        
        self.GridForLeftArm1 = [ 
                                [0.3965, -0.1069-0.003, -0.076, 0.03617, 0.9991, -0.0098, -0.0198], \
                                [0.5040-0.005, -0.1058, -0.076, 0.0147, 0.9994, 0.0002, -0.0316], \
                                [0.6115+0.002, -0.0908-0.018, -0.076, 0.0146, 0.9999, 0.0064, -0.0047], \
                                [0.3917, 0.0117-0.01, -0.076, 0.0073, 0.9993, 0.0042, -0.0364], \
                                [0.4983+0.01, 0.0108-0.008, -0.076, -0.0066, 0.9999, 0.0015, -0.0127], \
                                [0.6093+0.01, 0.0097-0.008, -0.076, 0.0154, 0.9997, 0.0029, -0.0194], \
                                [0.3914+0.012, 0.1200-0.01, -0.076, 0.0126, 0.9994, -0.0002, -0.0333], \
                                [0.5029, 0.1203-0.008, -0.076, 0.0020, 0.9993, 0.0008, -0.0378], \
                                [0.6103+0.009, 0.1161-0.005, -0.076, 0.0195, 0.9992, -0.0010, -0.0352], \
                                
                               ]
        #pick
        self.GridForRightArm = [ [0.4019+0.007, -0.0972, -0.076, -0.0221, 0.9997, 0.0069, -0.0032], \
                                [0.5125+0.002, -0.0957, -0.076, -0.0178, 0.9996, 0.0053, -0.0213], \
                                [0.6169+0.004, -0.0947-0.004, -0.076, -0.0172, 0.9996, -0.0009, 0.0209], \
                                [0.3964,  0.0073+0.005, -0.076, -0.0174, 0.9998, -0.0050, -0.0050], \
                                [0.5020+0.015, 0.0090+0.005, -0.076, -0.0068, 0.9998, -0.0050, 0.0195], \
                                [0.6169+0.006, 0.0147, -0.076, -0.0059, 0.9999, 0.0022, -0.0015], \
                                [0.3948+0.01, 0.1160+0.008, -0.076, -0.0070, 0.9998, -0.0034, -0.0180], \
                                [0.5042+0.002, 0.1188+0.006, -0.076, -0.0058, 0.9999, -0.0073, 0.0087], \
                                [0.6150+0.012, 0.1181+0.005, -0.076, -0.0235, 0.9995, -0.0124, 0.0146], \ 
                               ]
        #place                        
        self.GridForRightArm1 =[ [0.4019+0.007, -0.0972, -0.076, -0.0221, 0.9997, 0.0069, -0.0032], \
                                [0.5125+0.008, -0.0957, -0.076, -0.0178, 0.9996, 0.0053, -0.0213], \
                                [0.6169+0.012, -0.0947-0.001, -0.076, -0.0172, 0.9996, -0.0009, 0.0209], \
                                [0.3964+0.01,  0.0073+0.007, -0.076, -0.0174, 0.9998, -0.0050, -0.0050], \
                                [0.5020+0.015, 0.0090+0.006, -0.076, -0.0068, 0.9998, -0.0050, 0.0195], \
                                [0.6169+0.009, 0.0147, -0.076, -0.0059, 0.9999, 0.0022, -0.0015], \
                                [0.3948+0.002, 0.1160+0.01, -0.076, -0.0070, 0.9998, -0.0034, -0.0180], \
                                [0.5042+0.005, 0.1188+0.008, -0.076, -0.0058, 0.9999, -0.0073, 0.0087], \
                                [0.6150+0.012, 0.1181+0.005, -0.076, -0.0235, 0.9995, -0.0124, 0.0146] ]
        '''
                                
        self.RightSlots = ['o', 'o', 'o', 'o', 'o']
        self.LeftSlots = ['x', 'x', 'x', 'x', 'x']
        self.GridStatus = ['b', 'b', 'b', 'b', 'b', 'b', 'b', 'b', 'b']
    
    def display_image(self, image_tag):
        """
        Send the image located at the specified path to the head
        display on Baxter.

        @param path: path to the image file to load and send
        """
        img = self.display_images[image_tag]
        msg = CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
        pub.publish(msg)
        # Sleep to allow for image to be published.
        rospy.sleep(1)
    
    
        
    def robot_state_callback(self, msg):
        
        self.EstopStatus = msg
        if self.EstopStatus.estop_button == 1 or self.EstopStatus.estop_button==2:
            if self.GameState =='NoInit':
                self.GameError = 'NoInit'
                
            self.GameState = 'Estop_on'
            
        
        return
        
    def button_callback(self, msg):
        
        #button_state = msg.data
        #print "Button State", msg.state
        self.ButtonStatus = msg.state
        return
    
    def wait_for_estop(self):
        
        if self.EstopStatus.enabled == True:
            return
        
        reset_flag = False
        while self.EstopStatus.estop_button == 2 or self.EstopStatus.estop_button == 1:
            if self.EstopStatus.estop_button == 2:
                self.reset_estop()
            rospy.sleep(0.1)
        
        #while self.ButtonStatus == 0:
        #    rospy.sleep(0.1)
        
        print "E Stop is reset"
        self.GameState = 'Estop_reset'
        
        return
        
        
        
        
    def get_button_status(self):
        
        button_status = self.ButtonStatus
        
        return button_status
    
    # wait for button on for second
    def wait_button_on(self, min_seconds, max_seconds):
        
        button_status = self.ButtonStatus
        counter = 0
        counter1 = 0
        max_count = math.floor(max_seconds/0.1)
        min_count = math.floor(min_seconds/0.1)
        while counter<max_count:
            
            if self.GameState == 'Estop_on':
                return -1
            button_status = self.ButtonStatus
            #print "Button Status: ", button_status
            if (button_status == 0):
                counter1 = counter1 + 1
            else:
                counter1 = 0
            rospy.sleep(0.1)
            counter = counter + 1
            if counter1>=min_count:
                return 1
            
        return 0 # return 0 : time out, return 1, button pressed correctly
            
    def wait_button_on1(self, min_seconds):
        
        button_status = self.ButtonStatus
        counter = 0
        counter1 = 0
       
        min_count = math.floor(min_seconds/0.1)
        while not rospy.is_shutdown():
            
            if self.GameState == 'Estop_on':
                
                rospy.sleep(0.1)
                return -1
            
            
            button_status = self.ButtonStatus
            #print "Button Status: ", button_status
            if (button_status == 0):
                counter1 = counter1 + 1
            else:
                counter1 = 0
            rospy.sleep(0.1)
            counter = counter + 1
            if counter1>=min_count:
                
                
                #else:
                return 1
            
        return 0 # return 0 : time out, return 1, button pressed correctly    
    
    def get_vision_reply(self):
        
        cv_reply = self.VisionReply
        
        while cv_reply == '':
            
            if self.GameState== 'Estop_on':
                return ''
            cv_reply = self.VisionReply
            rospy.sleep(0.1)
            
        self.VisionReply = ''
        return cv_reply
    
    def interpret_vision_reply(self, msg_string):
        msg_segment = msg_string.split(':')
        if len(msg_segment)!= 3:
            rospy.logerr("Vision Reply Failure: msg segment not right")
            return '', []
        item_name = msg_segment[0]
        item_pose = [float(m) for m in msg_segment[2].split(',')]
        
        return item_name, item_pose
    
    def interpret_grid_checking_reply(self, msg_string):
        
        msg_segment = msg_string.split(':')
        print msg_segment
        if len(msg_segment)!= 3:
            rospy.logerr("Vision Reply Failure: msg segment not right")
            return '', []
        
        if msg_segment[1] != 'grid_status':
            rospy.logerr("Vision Reply Failure: msg type not right")
            return '', []
        
        grid_status = msg_segment[2].split()
        if len(grid_status) != 1:
            rospy.logerr("Vision Reply Failure: number of grids not right")
            return '', []
        
        return msg_segment[0], grid_status
    
    def interpret_grid_checking_reply1(self, msg_string):
        
        msg_segment = msg_string.split(':')
        print msg_segment
        if len(msg_segment)!= 4:
            rospy.logerr("Vision Reply Failure: msg segment not right")
            return '', [], []
        
        if msg_segment[1] != 'grid_status':
            rospy.logerr("Vision Reply Failure: msg type not right")
            return '', [], []
        
        grid_status = msg_segment[2].split()
        item_xy = msg_segment[3].split(',')
        xy_list = [float(item_xy[0]), float(item_xy[1])]
        #item_xy.remove('')
        if len(grid_status) != 1:
            rospy.logerr("Vision Reply Failure: number of grids not right")
            return '', [],[]
        
        return msg_segment[0], grid_status, xy_list
        
    def gripper_control(self, side, action):
        gripper = self.baxter_grippers[side]
        if action == 'calibrate':
            gripper.calibrate()
        elif action == 'open':
            gripper.open()
        elif action == 'close':
            gripper.close()
        
        
    def move_arm(self, side, target_pose):
        
        # if e stop is on
        if self.GameState == 'Estop_on':
            self.LeftSlots = ['x','x','x','x','x']
            self.RightSlots = ['o','o','o','o','o']
            self.GridStatus = ['b','b','b','b','b','b','b','b','b']
            return
        
        if len(target_pose)!=7:
            print "Move arm target pose list number not correct..."
            print "Error target pose: ", target_pose
            return
        
        x = target_pose[0]
        y = target_pose[1]
        z = target_pose[2]
        ox = target_pose[3]
        oy = target_pose[4]
        oz = target_pose[5]
        ow = target_pose[6]
        
        msg_string = side + ':move_to:' + \
                      str(x) + \
                      ',' + \
                      str(y) + \
                      ',' + \
                      str(z) + \
                      ',' + \
                      str(ox) + \
                      ',' + \
                      str(oy) + \
                      ',' + \
                      str(oz) + \
                      ',' + \
                      str(ow)
        self.ArmCmdPub.publish(msg_string)
        #print self.current_poses
        cur_pose = self.current_poses[side]
        dist_x = math.fabs(cur_pose.pose.position.x-x)
        dist_y = math.fabs(cur_pose.pose.position.y-y)
        dist_z = math.fabs(cur_pose.pose.position.z-z)
        dist = math.sqrt(dist_x*dist_x+dist_y*dist_y+dist_z*dist_z)
        
        counter = 0
        while dist>0.005: #dist_x>0.005 or dist_y>0.005 or dist_z>0.005:
            
            # if e stop is on
            if self.GameState == 'Estop_on':
                self.LeftSlots = ['x','x','x','x','x']
                self.RightSlots = ['o','o','o','o','o']
                self.GridStatus = ['b','b','b','b','b','b','b','b','b']
                
                return
            
            cur_pose = self.current_poses[side]
            
            dist_x = math.fabs(cur_pose.pose.position.x-x)
            dist_y = math.fabs(cur_pose.pose.position.y-y)
            dist_z = math.fabs(cur_pose.pose.position.z-z)
            dist = math.sqrt(dist_x*dist_x+dist_y*dist_y+dist_z*dist_z)
            if counter > 100:
                return
            counter = counter +1
            rospy.sleep(0.05)
        #print "Postion Reached", dist_x, dist_y, dist_z
        #print target_pose
    
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
    
    def init_msgs(self):
        
        left_arm_msg = message_filters.Subscriber("/robot/limb/left/endpoint_state",EndpointState)
        right_arm_msg = message_filters.Subscriber("/robot/limb/right/endpoint_state",EndpointState)
        ts = message_filters.ApproximateTimeSynchronizer([left_arm_msg, right_arm_msg], 10, 0.05)
        ts.registerCallback(self.pose_callback)
    
    def pose_callback(self, left_msg, right_msg):
    
        pose1 = left_msg.pose
        pose2 = right_msg.pose
        
        header = Header(stamp=rospy.Time.now(), frame_id='base')
        cp1 = self.make_pose_stamp([pose1.position.x, \
                                     pose1.position.y, \
                                     pose1.position.z, \
                                     pose1.orientation.x, \
                                     pose1.orientation.y, \
                                     pose1.orientation.z, \
                                     pose1.orientation.w], \
                                     header)
        
        cp2 = self.make_pose_stamp([pose2.position.x, \
                                     pose2.position.y, \
                                     pose2.position.z, \
                                     pose2.orientation.x, \
                                     pose2.orientation.y, \
                                     pose2.orientation.z, \
                                     pose2.orientation.w], \
                                     header)
        
        self.current_poses = {'left':cp1, 'right':cp2}
        
        
        #print self.current_poses
        #print "\nleft: \n", cp1, "\n\nnew left: \n", cp2
            
        return
        
    def init_game(self):
        
        if self.GameState == 'Estop_on':
            return
        self.gripper_control('left', 'calibrate')
        if self.GameState == 'Estop_on':
            return
        self.gripper_control('left', 'open')
        if self.GameState == 'Estop_on':
            return
        self.gripper_control('right', 'calibrate')
        if self.GameState == 'Estop_on':
            return
        self.gripper_control('right', 'open')
        if self.GameState == 'Estop_on':
            return
        rospy.sleep(1)
        print "Game Init Starts"
        #self.move_arm('left', [0.0, 0.6, 0.2, 0.0, 1.0, 0.0, 0.0])
        #self.move_arm('left', self.LeftArmInitPose)
        joints_left_list = {'left_s0':1.6977, 'left_s1':0.1446, 'left_e0':-1.5858, 'left_e1':2.4106, 'left_w0':1.6409, 'left_w1':1.6409, 'left_w2':0.1472}
        #self.arms['left'].set_joint_position_speed(0.3)
        if self.GameState == 'Estop_on':
            return
        self.arms['left'].move_to_joint_positions(joints_left_list)
        if self.GameState == 'Estop_on':
            return
        joints_left_list = {'left_s0':0.3, 'left_s1':0.1446, 'left_e0':-1.5858, 'left_e1':2.4106, 'left_w0':1.6409, 'left_w1':1.6409, 'left_w2':0.1472}
        #self.arms['left'].move_to_joint_positions(joints_left_list)
        #rospy.sleep(2)
        #self.move_arm('right', [0.0, -0.6, 0.2, 0.0, 1.0, 0.0, 0.0])
        #self.move_arm('right', self.RightArmInitPose)
        joints_right_list = {'right_s0':-1.6977, 'right_s1':0.1446, 'right_e0':1.5858, 'right_e1':2.4106, 'right_w0':-1.6409, 'right_w1':1.6574, 'right_w2':-0.1472}
        #self.arms['right'].set_joint_position_speed(0.5)
        if self.GameState == 'Estop_on':
            return
        self.arms['right'].move_to_joint_positions(joints_right_list)
        if self.GameState == 'Estop_on':
            return
        joints_right_list = {'right_s0':-0.3, 'right_s1':0.1446, 'right_e0':1.5858, 'right_e1':2.4106, 'right_w0':-1.6409, 'right_w1':1.6574, 'right_w2':-0.1472}
        #self.arms['right'].move_to_joint_positions(joints_right_list)
        rospy.sleep(2)
        
        msg_string = 'game_engine:o:quit_session'
        self.GridStatusPub.publish(msg_string)
        
        #self.GridCenter = self.GridLocations[4]
        
        # Up to here, the center of the Grid Pattern is found.
    
    def check_one_grid1(self, side, pose_list, grid_id):
        
        x = pose_list[0]
        y = pose_list[1]
        z = pose_list[2]
        ox = pose_list[3]
        oy = pose_list[4]
        oz = pose_list[5]
        ow = pose_list[6]
        
        if self.GameState == 'Estop_on':
            return
        
##        if side == 'left':
##            if grid_id == 2:
##                pose_temp = list(self.GridForLeftArm1[0])
##                pose_temp[2] = pose_temp[2] + 0.2
##                self.move_arm('left', pose_temp)
##            elif grid_id == 5:
##                pose_temp = list(self.GridForLeftArm1[3])
##                pose_temp[2] = pose_temp[2] + 0.2
##                self.move_arm('left', pose_temp)
##            elif grid_id == 8:
##                pose_temp = list(self.GridForLeftArm1[6])
##                pose_temp[2] = pose_temp[2] + 0.2
##                self.move_arm('left', pose_temp)
##        
##        elif side =='right':
##        
##            if grid_id == 2:
##                pose_temp = list(self.GridForRightArm1[0])
##                pose_temp[2] = pose_temp[2] + 0.2
##                self.move_arm('right', pose_temp)
##            elif grid_id == 5:
##                pose_temp = list(self.GridForRightArm1[3])
##                pose_temp[2] = pose_temp[2] + 0.2
##                self.move_arm('right', pose_temp)
##            elif grid_id == 8:
##                pose_temp = list(self.GridForRightArm1[6])
##                pose_temp[2] = pose_temp[2] + 0.2
##                self.move_arm('right', pose_temp)                
##            
##            
##        
        new_pose = list(pose_list)
        
        new_pose[2] = new_pose[2] + 0.17
        
        self.move_arm(side, new_pose)
        
        if self.GameState == 'Estop_on':
            return
        
        rospy.sleep(1)
        
        msg_string = side+':check1:'+str(grid_id)
        print "Check Grid1 Cmd: ", msg_string
        self.VisionCmdPub.publish(msg_string)
        
        cv_reply = self.get_vision_reply()
    
        reply_cmd, grid_status, xy_list = self.interpret_grid_checking_reply1(cv_reply)
        
        return grid_status, xy_list
        
    def check_one_grid(self, side, grid_id):
        
        print "Checking Grid Status of grid id : ", grid_id
        center_pose = self.GridLocations[4]
        x = center_pose[0]
        y = center_pose[1]
        z = center_pose[2]
        ox = center_pose[3]
        oy = center_pose[4]
        oz = center_pose[5]
        ow = center_pose[6]
        
        new_pose = list(self.GridLocations[grid_id])
        
        new_pose[2] = new_pose[2] + 0.17
        
        self.move_arm(side, new_pose)
        
        rospy.sleep(2)
        
        msg_string = side+':check1:'+str(grid_id)
        print "Check Grid1 Cmd: ", msg_string
        self.VisionCmdPub.publish(msg_string)
        
        cv_reply = self.get_vision_reply()
    
        reply_cmd, grid_status, xy_list = self.interpret_grid_checking_reply1(cv_reply)
        
        return grid_status, xy_list
    
    def pick_from_xy(self, side, pose):
        
        x = pose[0]
        y = pose[1]
        z = pose[2]
        ox = pose[3]
        oy = pose[4]
        oz = pose[5]
        ow = pose[6]
        
        #pose_list = [x, 0.3, z + 0.2, ox, oy, oz, ow]
        #if self.GameState == 'Estop_on':
        #    return
        #self.move_arm(side, pose_list)
        
        pose_list = [x, y, z + 0.2, ox, oy, oz, ow]
        if self.GameState == 'Estop_on':
            return
        self.move_arm(side, pose_list)
        if self.GameState == 'Estop_on':
            return
        #rospy.sleep(1)
        if self.GameState == 'Estop_on':
            return
        self.gripper_control(side, 'open')
        pose_list2 = [x, y, z + 0.05, ox, oy, oz, ow]
        if self.GameState == 'Estop_on':
            return
        self.move_arm(side, pose_list2)
        pose_list1 = [x, y, z  , ox, oy, oz, ow]
        if self.GameState == 'Estop_on':
            return
        self.move_arm(side, pose_list1)
        if self.GameState == 'Estop_on':
            return
        rospy.sleep(0.5)
        if self.GameState == 'Estop_on':
            return
        self.gripper_control(side, 'close')
        rospy.sleep(0.5)
        if self.GameState == 'Estop_on':
            return
        self.move_arm(side, pose_list)
        
        return
        
        
    def place_to_xy(self, side, pose):
        
        if len(pose)!= 7:
            print "Place to xy error, pose list length incorrect"
            print "Pose List: ", pose
            return
        
        x = pose[0]
        y = pose[1]
        z = pose[2]
        ox = pose[3]
        oy = pose[4]
        oz = pose[5]
        ow = pose[6]
        
        
        
        if self.GameState == 'Estop_on':
            return
        pose_list = [x, y, z + 0.2, ox, oy, oz, ow]
        self.move_arm(side, pose_list)
        if self.GameState == 'Estop_on':
            return
        #rospy.sleep(1)
        
        pose_list2 = [x, y, z + 0.05, ox, oy, oz, ow]
        self.move_arm(side, pose_list2)
        if self.GameState == 'Estop_on':
            return
        pose_list1 = [x, y, z +0.015 , ox, oy, oz, ow]
        self.move_arm(side, pose_list1)
        if self.GameState == 'Estop_on':
            return
        #rospy.sleep(0.5)
        
        self.gripper_control(side, 'open')
        if self.GameState == 'Estop_on':
            return
        rospy.sleep(1)
        
        #pose_list2 = [x+0.01, y, z + 0.03, ox, oy, oz, ow]
        #self.move_arm(side, pose_list2)
        #rospy.sleep(2)
        if self.GameState == 'Estop_on':
            return
        self.move_arm(side, pose_list)
        
        pose_list = [x, y, z + 0.2, ox, oy, oz, ow]
        if self.GameState == 'Estop_on':
            return
        self.move_arm(side, pose_list)
        
        #pose_list = [x, 0.3, z + 0.2, ox, oy, oz, ow]
        #if self.GameState == 'Estop_on':
        #    return
        #self.move_arm(side, pose_list)
        
        #rospy.sleep(2)
        
        
        
        
        
        
        
    # Place all the blocks in the grids back to start position    
    # Assume a block can be in the grids or already at the start positions
    # will check each grid first, pick away one type('x'/'o') with one arm, then pick 
    # all the other type with the other arm
    def place_all_blocks(self):
        
        grid_ids = [0, 3, 6, 1, 4, 7, 2, 5, 8]
        grid_status_list = []
        xy_list = []
        for id in grid_ids:
            
            grid_status, xy = self.check_one_grid('left', id)
            print "Grid is : ", grid_status
            print "Item xy: ", xy
            grid_status_list.append(grid_status[0])
            xy_list.append(xy)
            
        print "Ready to clean the table..."
        print grid_status_list
        print xy_list
        counter = 0
        for item in grid_status_list:
            
            self.wait_for_estop()
            cur_index = counter #grid_status_list.index(item)
            grid_id = grid_ids[cur_index]
            print "Process item in grid: ", grid_id
            xy = xy_list[cur_index]
            if xy[0]>10 or xy[1]>10:
                counter = counter + 1
                continue
            
            if item == 'x':
                
                self.move_arm('right', self.RightArmInitPose)
                self.pick_from_xy('left', self.GridForRightArm[grid_id])
                x1 = self.GridForRightArm1[6][0]
                y1 = self.GridForRightArm1[6][1]+0.16
                left_slot = [x1, y1, -0.0740, 0.0, 1.0, 0.0111, 0.0]
                self.place_to_xy('left', left_slot)
                
            elif item == 'o':
                
                self.move_arm('left', self.LeftArmInitPose)
                x1 = self.GridLocations[0][0]
                y1 = self.GridLocations[0][1]-0.16
                right_slot = [x1, y1, -0.0740, 0.0, 1.0, 0.0111, 0.0]
                self.pick_from_xy('right', self.GridLocations[grid_id])
                self.place_to_xy('right', right_slot)
                
            elif item == 'b':
                
                pass
                
            counter = counter + 1
        
        
            
        return    
    
    def pick_one_grid(self, side):
        
        
        
        
        return
    
          
    def check_grid(self, side):
        
        print "Checking Grid Status..."
        center_pose = self.GridLocations[4]
        x = center_pose[0]
        y = center_pose[1]
        z = center_pose[2]
        ox = center_pose[3]
        oy = center_pose[4]
        oz = center_pose[5]
        ow = center_pose[6]
        
        pose_list1 = [x-0.02, y, z+0.12, ox, oy, oz, ow]
        pose_list2 = [x+0.1, y, z+0.12, ox, oy, oz, ow]
        pose_list3 = [x+0.22, y, z+0.12, ox, oy, oz, ow]
        poses = []
        #poses.append(pose_list1)
        #poses.append(pose_list2)
        #poses.append(pose_list3)
        for item in self.GridLocations:
            
            new_list = list(item)
            new_list[2] = z + 0.14
            poses.append(new_list)
        
        
        roi_order_list = [0, 3, 6, 1, 4, 7, 2, 5, 8]
        
        grid_final_status = []
        for pose in poses:
            self.move_arm(side, pose)
            rospy.sleep(0.5)
            grid_id = roi_order_list[poses.index(pose)]
            msg_string = side+':check:'+str(grid_id)
            print "Check Grid Cmd: ", msg_string
            self.VisionCmdPub.publish(msg_string)
            
            cv_reply = self.get_vision_reply()
        
            reply_cmd, grid_status = self.interpret_grid_checking_reply(cv_reply)
            grid_final_status.extend(grid_status)
            print "Grid Status: ", grid_status
            
        
        if len(grid_final_status) == 9:
            return grid_final_status
        else:
            return ''
            #send the grid status to game engine

        
    def pick_item(self, side, target_name):
        
        self.move_arm(side, [self.GridCenter[0], self.GridCenter[1]+0.3, self.TableZ+0.10, 0.0, 1.0, 0.0, 0.0])
        rospy.sleep(2)
        msg_string = side+':detect:'+ target_name
        self.VisionCmdPub.publish(msg_string)
        
        cv_reply = self.get_vision_reply()
        print "Grid Tracking result: ", cv_reply
        item_name, item_pose = self.interpret_vision_reply(cv_reply)
        cur_pose = self.current_poses[side]
        self.gripper_control('left', 'open')
        self.move_arm(side, [item_pose[0], item_pose[1], self.TableZ+self.BlockHeight+0.02, 0.0, 1.0, 0.0, 0.0])
        rospy.sleep(2)
        
        
        '''
        msg_string = side+':fine_tune:'+ target_name
        print "Fine tune Cmd: ", msg_string
        self.VisionCmdPub.publish(msg_string)
        cv_reply = self.get_vision_reply()
        print "Fine tune Reply: ", cv_reply
        item_name, item_pose = self.interpret_vision_reply(cv_reply)
        dx = item_pose[0]
        dy = item_pose[1]
        angle = item_pose[2]
        print "Fine tune parameters: ", dx, dy, angle
        sign_x = 1.0
        sign_y = 1.0
        while (math.fabs(dx)>15 or math.fabs(dy)>15) and not rospy.is_shutdown():
            
            cur_pose = self.current_poses[side]
            x = cur_pose.pose.position.x
            y = cur_pose.pose.position.y
            z = cur_pose.pose.position.z
            ox = cur_pose.pose.orientation.x
            oy = cur_pose.pose.orientation.y
            oz = cur_pose.pose.orientation.z
            ow = cur_pose.pose.orientation.w
            if math.fabs(dy)>15:
                
                x1 = x+0.004*sign_x
            else:
                x1 = x
            if math.fabs(dx)>15:
                
                y1 = y+0.004*sign_y
            else:
                y1 = y
                
            self.move_arm(side, [x1, y1, z, ox, oy, oz, ow])
            print "Fine Tuning Moving..."
            rospy.sleep(0.2)
            msg_string = side+':fine_tune:'+ target_name
            self.VisionCmdPub.publish(msg_string)
            cv_reply = self.get_vision_reply()
            item_name, item_pose = self.interpret_vision_reply(cv_reply)
            dx1 = item_pose[0]
            dy1 = item_pose[1]
            angle1 = item_pose[2]
            if math.fabs(dx)>math.fabs(dx1):
                sign_x = sign_x * -1.0
            
            if math.fabs(dy)>math.fabs(dy1):
                sign_x = sign_x * -1.0
                
            dx = dx1
            dy = dy1
            angle = angle1
            
            print "Fine Tuning Parameters: ", dx, dy, angle
            rospy.sleep(2)
            
        print "Fine Tuning Is Done..."  
        '''
        
        init_pose = self.current_poses[side]
        x = init_pose.pose.position.x
        y = init_pose.pose.position.y
        z = init_pose.pose.position.z
        ox = init_pose.pose.orientation.x
        oy = init_pose.pose.orientation.y
        oz = init_pose.pose.orientation.z
        ow = init_pose.pose.orientation.w
        
        self.move_arm(side, [x, y, self.TableZ+self.BlockHeight-0.01, 0.0, 1.0, 0.0, 0.0])
        rospy.sleep(2)
        self.gripper_control('left', 'close')
        rospy.sleep(1)
        self.move_arm(side, [x, y, self.TableZ+self.BlockHeight+0.06, 0.0, 1.0, 0.0, 0.0])
        
        #self.move_arm(side, [item_pose[0], item_pose[1], self.TableHeight+self.GripperLength+0.05, 0.0, 1.0, 0.0, 0.0])
        rospy.sleep(2)
        #self.gripper_control('left', 'close')
    
    def place_item(self, side, col, row):
        
        init_pose = self.current_poses['left']
        x = init_pose.pose.position.x
        y = init_pose.pose.position.y
        z = init_pose.pose.position.z
        ox = init_pose.pose.orientation.x
        oy = init_pose.pose.orientation.y
        oz = init_pose.pose.orientation.z
        ow = init_pose.pose.orientation.w
        
        if col>2 or col<0 or row>2 or row<0:
            return
        print "about to place..."
        
        self.move_arm(side, [x, y, self.TableZ+self.BlockHeight+0.2, 0.0, 1.0, 0.0, 0.0])
        rospy.sleep(1.5)
        
        pose_list = self.GridLocations[col + row*3]
        pose_list1 = list(pose_list)
        pose_list1[2] = pose_list1[2]+0.15
        self.move_arm(side, pose_list1)
        rospy.sleep(1.5)
        
        pose_list1 = list(pose_list)
        pose_list1[2] = pose_list1[2]+0.025
        self.move_arm(side, pose_list1)
        rospy.sleep(1.5)
        self.gripper_control(side, 'open')
        pose_list1 = list(pose_list)
        pose_list1[2] = pose_list1[2]+0.15
        self.move_arm(side, pose_list1)
        
        
    def load_location_data(self):
        self.GridLocations = []
        self.TableZ = 0.0
        file = open("./src/phm/grids_location.txt", "r")
        for i in range(0, 9):
            line = file.readline()
            temp_list = []
            for j in line.split():
                temp_list.append(float(j))
            self.GridLocations.append(temp_list)
            #print temp_list
            self.TableZ = self.TableZ + temp_list[2]
        self.TableZ = self.TableZ / 9
        file.close()
        print self.GridLocations
        print "Table Z Value:", self.TableZ
        
        self.GridRoiLocations = []
        self.GridRoiPose = []
        file = open("./src/phm/grids_roi_location.txt", "r")
        for i in range(0, 11):
            line = file.readline()
            temp_list = []
            for j in line.split():
                if i<9:
                    temp_list.append(int(j))
                elif i==9:
                    temp_list.append(float(j))
            
            if i<9:
                self.GridRoiLocations.append(temp_list)
            elif i==9:
                self.GridRoiPose = temp_list
        
        file.close()
        print self.GridRoiLocations
        print self.GridRoiPose

    def start_roi_recording(self):
        pass
        
    def pick_test(self, col, row):
        
        self.gripper_control('left', 'calibrate')
        self.gripper_control('left', 'open')
        rospy.sleep(1)
        init_pose = self.current_poses['left']
        x = init_pose.pose.position.x
        y = init_pose.pose.position.y
        z = init_pose.pose.position.z
        ox = init_pose.pose.orientation.x
        oy = init_pose.pose.orientation.y
        oz = init_pose.pose.orientation.z
        ow = init_pose.pose.orientation.w
        
        if col>2 or col<0 or row>2 or row<0:
            return
        print "about to pick"
        pose_list = self.GridLocations[col + row*3]
        print pose_list
        pose_list1 = list(pose_list)
        pose_list1[2] = pose_list1[2]+0.1
        self.move_arm('left', pose_list)
        rospy.sleep(1)
        return
        self.move_arm('left', pose_list)
        rospy.sleep(1)
        self.gripper_control('left', 'close')
        
        self.move_arm('left', pose_list1)
        rospy.sleep(1)
        init_pose_list = [x, y, z, ox, oy, oz, ow]
        self.move_arm('left', init_pose_list)
        rospy.sleep(1)
    
    def next_move_callback(self, msg):
        
        self.NextMoveReply = msg.data
        print "Next Move reply in callback: ",self.NextMoveReply
    
    
    def wait_for_next_move(self):
        
        print "Wait For Next Move Reply..."
        next_move_reply = self.NextMoveReply
        
        while next_move_reply == '':
            
            if self.GameState == 'Estop_on':
                return 'Estop_on'
            
            next_move_reply = self.NextMoveReply
            rospy.sleep(0.1)
            
        self.NextMoveReply = ''
        
        return next_move_reply
        
    def interpret_next_move(self, msg_string):
        if msg_string == '':
            rospy.logerr("Next Move Reply Failure: msg is empty")
        
        msg_segments = msg_string.split()
        
        if len(msg_segments)!= 2:
            rospy.logerr("Next Move Reply Failure: msg segment not correct")
            return '', -1
            
        item = msg_segments[0]
        id = int(msg_segments[1]) # grid id: 0~8
        if item in ['x', 'o']:
            return item, id
        elif item == 'draw':
            return 'draw', -1
        elif item == 'win':
            return 'win', id
        else:
            return '', -1
    
    def find_in_slots(self, side, item):
        
        if side == 'left':
            
            for item in self.LeftSlots:
                
                if item != 'b':
                    return self.LeftSlots.index(item)
                
        elif side == 'right':
            
            for item in self.RightSlots:
                
                if item != 'b':
                    return self.RightSlots.index(item)
                
        return -1
                
    def get_grid_status_string(self):
        
        return_string = ''
        for item in self.GridStatus:
            return_string = return_string + item + ' '
            
        return return_string

    def find_empty_slot(self, item):
        
        counter = 0
        if item == 'x':
            for item in self.LeftSlots:
                if self.LeftSlots[counter] == 'b':
                    return counter
                counter = counter + 1
        
        elif item == 'o':
            
            for item in self.RightSlots:
                if self.RightSlots[counter] == 'b':
                    return counter
                
                counter = counter + 1
                
        return -1
    
    def place_all_blocks1(self):
        
        #self.LeftSlots = ['x','x','x','x','b']
        #self.RightSlots = ['b','o','o','o','o']
        #self.GridStatus = ['b','b','b','b','b','b','b','b','x']
        
        print "Reset the Table..."
        grid_ids = [0, 3, 6, 1, 4, 7, 2, 5, 8]
        grid_status_list = []
        xy_list = []
        
        self.move_arm('right', self.RightArmInitPose)
        counter = 0
        for item in self.GridStatus:
            
            if self.GameState == 'Estop_on':
                return
            
            
            if item == 'x':
                
                if self.GameState == 'Estop_on':
                    return
                #self.move_arm('right', self.RightArmInitPose)
                if self.GameState == 'Estop_on':
                    return
                self.pick_from_xy('left', self.GridForLeftArm[counter])
                if self.GameState == 'Estop_on':
                    return
                
                slot_id = self.find_empty_slot('x')
                self.place_to_xy('left', self.LeftSlotsLocation1[slot_id])
                if self.GameState == 'Estop_on':
                    return
                self.LeftSlots[slot_id] = 'x'
                self.GridStatus[counter] = 'b'
                
            
                
            elif item == 'b':
                
                pass
                
            counter = counter + 1
        
        self.move_arm('left', self.LeftArmInitPose)
        counter = 0
        for item in self.GridStatus:
            
            if self.GameState == 'Estop_on':
                return
            
            if item == 'o':
                
                if self.GameState == 'Estop_on':
                    return
                
                #self.move_arm('left', self.LeftArmInitPose)
                if self.GameState == 'Estop_on':
                    return
                
                self.pick_from_xy('right', self.GridForRightArm[counter])
                if self.GameState == 'Estop_on':
                    return
                
                slot_id = self.find_empty_slot('o')
                self.place_to_xy('right', self.RightSlotsLocation1[slot_id])
                if self.GameState == 'Estop_on':
                    return
                
                self.RightSlots[slot_id] = 'o'
                self.GridStatus[counter] = 'b'
                
            elif item == 'b':
                
                pass
                
            counter = counter + 1
        self.move_arm('right', self.RightArmInitPose)
            
        return
    
    def place_all_blocks2(self):
        
        #self.LeftSlots = ['x','x','x','x','b']
        #self.RightSlots = ['b','o','o','o','o']
        #self.GridStatus = ['b','b','b','b','b','b','b','b','x']
        
        print "Init the Table..."
        grid_ids = [0, 3, 6, 1, 4, 7, 2, 5, 8]
        grid_status_list = []
        xy_list = []
        
        print "Left Slots: ", self.LeftSlots
        print "Right Slots: ", self.RightSlots
        print "Grid Status: ", self.GridStatus
        
        self.move_arm('right', self.RightArmInitPose)
        counter = 0
        
        number_x_in_grid = self.GridStatus.count('x')
        number_x_left_slot = self.LeftSlots.count('x')
        number_x_right_slot = self.RightSlots.count('x')
        number_o_in_grid = self.GridStatus.count('o')
        number_o_left_slot = self.LeftSlots.count('o')
        number_o_right_slot = self.RightSlots.count('o')
        
        
        number_of_swapping = number_x_right_slot+number_o_left_slot
        print "Number of swapping: ", number_of_swapping
        number_of_empty_grid = 9-number_x_in_grid-number_o_in_grid
        print "Empty grid slots: ", number_of_empty_grid
        
        if (number_x_in_grid==0 and number_x_left_slot==5 and \
            number_o_in_grid==0 and number_o_right_slot==5) or \
            (number_x_in_grid==0 and number_x_right_slot==5 and \
            number_o_in_grid==0 and number_o_left_slot==5):
            
            print "The table is ready..."
            return True
        
        elif (number_of_swapping<=number_of_empty_grid):
            # less x stay right side, less o stay left side, will make left side all x right side all o
            print "Number of swapping less than empty grids..."
            counter = 0
            for item in self.LeftSlots:
                
                if item=='o':
                    
                    pose1 = self.LeftSlotsLocation[counter]
                    #print "Pick o from 
                    self.pick_from_xy('left', pose1)
                                        
                    counter1 = 0
                    for grid_item in self.GridStatus:
                        
                        if grid_item == 'b':
                            pose2 = self.GridForLeftArm[counter1]
                            self.place_to_xy('left', pose2)
                            self.GridStatus[counter1] = 'o'
                            self.LeftSlots[counter] = 'b'
                            break
                            
                        counter1 = counter1 + 1
                
                counter = counter + 1
            self.move_arm('left', self.LeftArmInitPose)
                
            counter = 0
            for item in self.RightSlots:
                
                if item=='x':
                    pose3 = self.RightSlotsLocation[counter]
                    self.pick_from_xy('right', pose3)
                                        
                    counter1 = 0
                    for grid_item in self.GridStatus:
                        
                        if grid_item == 'b':
                            pose4 = self.GridForRightArm[counter1]
                            self.place_to_xy('right', pose4)
                            self.GridStatus[counter1] = 'x'
                            self.RightSlots[counter] = 'b'
                            break
                            
                        counter1 = counter1 + 1
                
                counter = counter + 1
            self.move_arm('right', self.RightArmInitPose)
            

                            
                            
                            
                    
            
            
            
            pass
            
        elif (number_of_swapping>number_of_empty_grid):
            # less o stay left side, less x stay right side, will make left side all o right side all x
            
            print "Number of swapping more than empty grids..."
            
            pass
            
        
        self.place_all_blocks1()
    
      
    def move_to_init(self, side):
        
        if side == 'left':
            joints_left_list = {'left_s0':1.6977, 'left_s1':0.1446, 'left_e0':-1.5858, 'left_e1':2.4106, 'left_w0':1.6409, 'left_w1':1.6409, 'left_w2':0.1472}
        
            self.arms['left'].move_to_joint_positions(joints_left_list)
        #joints_left_list = {'left_s0':0.3, 'left_s1':0.1446, 'left_e0':-1.5858, 'left_e1':2.4106, 'left_w0':1.6409, 'left_w1':1.6409, 'left_w2':0.1472}
        elif side == 'right':
            joints_right_list = {'right_s0':-1.6977, 'right_s1':0.1446, 'right_e0':1.5858, 'right_e1':2.4106, 'right_w0':-1.6409, 'right_w1':1.6574, 'right_w2':-0.1472}
        
            self.arms['right'].move_to_joint_positions(joints_right_list)
        
        return
    
    
    def demo_play(self):
        
        
        
        print "Entering Demo Mode..."
        
        self.LeftSlots = ['x','x','x','x','x']
        self.RightSlots = ['o','o','o','o','o']
        self.GridStatus = ['b','b','b','b','b','b','b','b','b']
        
        
        while self.GameState != 'Estop_on':
##            t = time.localtime(time.time())
##            print "Current Time: ", t.tm_hour, ":", t.tm_min
##            if (t.tm_hour >= 16 and t.tm_min>45) or (t.tm_hour<=9 and t.tm_min<30):
##                print "Not in working time period"
##                rospy.sleep(1)
##                continue
            
            first_play_id = randint(0,1)
            first_grid_id = randint(0,8)
            
            msg_string = ''
            next_move = ''
            if first_play_id == 0: # 'o' placed first
                
                self.display_image('rightplay')
                print "First Place is 'o' at %d" % first_grid_id
                self.FirstPlayMarker = 'o'
                msg_string = 'game_engine:o:start'
                self.GridStatus[first_grid_id] = 'o'
                self.GridStatusPub.publish(msg_string)
                msg_string = 'game_status:o:' + self.get_grid_status_string()
                slot_id = self.find_in_slots('right', 'o')
                
                if slot_id in range(0, 5):
                    
                    if self.GameState == 'Estop_on':
                        return
                    self.pick_from_xy('right', self.RightSlotsLocation[slot_id])
                    if self.GameState == 'Estop_on':
                        return
                    
                    self.place_to_xy('right', self.GridForRightArm1[first_grid_id])
                    if self.GameState == 'Estop_on':
                        return
                    
                    self.move_arm('right', self.RightArmInitPose)
                    if self.GameState == 'Estop_on':
                        return
                    #self.move_to_init('right')
                    
                self.RightSlots[slot_id] = 'b'
                
            else: # 'x' placed first
                
                self.display_image('leftplay')
                print "First Place is 'x' at %d" % first_grid_id
                
                self.FirstPlayMarker = 'x'
                msg_string = 'game_engine:x:start'
                self.GridStatus[first_grid_id] = 'x'
                self.GridStatusPub.publish(msg_string)
                msg_string = 'game_status:x:' + self.get_grid_status_string()
                
                slot_id = self.find_in_slots('left', 'x')
                if slot_id in range(0, 6):
                    
                    if self.GameState == 'Estop_on':
                        return
                    self.pick_from_xy('left', self.LeftSlotsLocation[slot_id])
                    if self.GameState == 'Estop_on':
                        return
                    self.place_to_xy('left', self.GridForLeftArm1[first_grid_id])
                    if self.GameState == 'Estop_on':
                        return
                    
                    self.move_arm('left', self.LeftArmInitPose)
                    if self.GameState == 'Estop_on':
                        return
                    #self.move_to_init('left')
                self.LeftSlots[slot_id] = 'b'
            self.GridStatusPub.publish(msg_string)
            
            sessionDone = False
            while not sessionDone:
            
                next_move = self.wait_for_next_move()
                
                if next_move == 'Estop_on' or self.GameState == 'Estop_on':
                    
                    self.LeftSlots = ['x','x','x','x','x']
                    self.RightSlots = ['o','o','o','o','o']
                    self.GridStatus = ['b','b','b','b','b','b','b','b','b']
                    msg_string = 'game_engine:o:quit_session'
                    self.GridStatusPub.publish(msg_string)
                    
                    return
                
                item, id = self.interpret_next_move(next_move)
                print "Item: ", item, "Id: ", id
                
                if item in ['x'] and id in range(0, 9):
                    
                    self.display_image('leftplay')
                    
                    slot_id = self.find_in_slots('left', 'x')
                    print "Pick x from left slot: ", self.LeftSlots[slot_id]
                    print "place it to: ", self.GridForLeftArm[id]
                    if self.GameState == 'Estop_on':
                        return
                    self.pick_from_xy('left', self.LeftSlotsLocation[slot_id])
                    if self.GameState == 'Estop_on':
                        return
                    self.place_to_xy('left', self.GridForLeftArm1[id])
                    if self.GameState == 'Estop_on':
                        return
                    self.move_arm('left', self.LeftArmInitPose)
                    if self.GameState == 'Estop_on':
                        return
                    print "Left Arm Back to Init Position..."
                    #self.move_to_init('left')
                    self.LeftSlots[slot_id] = 'b'
                    self.GridStatus[id] = item
                    msg_string = 'game_status:x:' + self.get_grid_status_string()
                    self.GridStatusPub.publish(msg_string)
                    
                elif item in ['o'] and id in range(0, 9):
                    
                    self.display_image('rightplay')
                    slot_id = self.find_in_slots('right', 'o')
                    print "Pick o from right slot: ", self.LeftSlots[slot_id]
                    print "place it to: ", self.GridForRightArm[id]
                    if self.GameState == 'Estop_on':
                        return
                    self.pick_from_xy('right', self.RightSlotsLocation[slot_id])
                    if self.GameState == 'Estop_on':
                        return
                    self.place_to_xy('right', self.GridForRightArm1[id])
                    if self.GameState == 'Estop_on':
                        return
                    self.move_arm('right', self.RightArmInitPose)
                    if self.GameState == 'Estop_on':
                        return
                    print "Right Arm Back to Init Position..."
                    #self.move_to_init('right')
                    self.GridStatus[id] = item
                    self.RightSlots[slot_id] = 'b'
                    msg_string = 'game_status:o:' + self.get_grid_status_string()
                    self.GridStatusPub.publish(msg_string)
                
                elif item == 'draw':
                    
                    self.display_image('draw')
                    sessionDone = True
                    self.GameStatus = 'Done'
                    rospy.sleep(2)
                    break
                    
                elif item == 'win':
                    if id==1:
                        self.display_image('rightwin')
                    elif id==0:
                        self.display_image('leftwin')
                    sessionDone = True
                    self.GameStatus = 'Done'
                    rospy.sleep(2)
                    break
            
                    
            self.display_image('resettable')
            self.place_all_blocks1()
            
            
            
            
    def reset_estop(self):
        
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        restart_status = False
        while not restart_status:
            try:
                rs.reset()
                restart_status = True
                #self.display_image('running')
            except Exception, e:
                rospy.logerr(e.strerror)
                restart_status = False
            rospy.sleep(0.1)
        
        
        return    
    
    def check_all_grids(self, side):
        
        grid_ids = range(0,9)
        pose_list = []
        grid_results = ['b','b','b','b','b','b','b','b','b']
        id_strings = ['00', '01', '02', '03', '04', '05', '06', '07', '08']
        counter = 0
        for id in grid_ids:
            slot_id = counter
            msg_string = 'scang'+id_strings[slot_id]
            
            if self.GameState == 'Estop_on':
                return
            
            self.DisplayCmdPub.publish(msg_string)
            
            if side=='left':
                pose_list = list(self.GridForLeftArm[id])
            elif side =='right':
                pose_list = list(self.GridForRightArm[id])
                
##            if side == 'left':
##                if slot_id == 2:
##                    pose_temp = list(self.GridForLeftArm1[0])
##                    pose_temp[2] = pose_temp[2] + 0.2
##                    self.move_arm('left', pose_temp)
##                elif slot_id == 5:
##                    pose_temp = list(self.GridForLeftArm1[3])
##                    pose_temp[2] = pose_temp[2] + 0.2
##                    self.move_arm('left', pose_temp)
##                elif slot_id == 8:
##                    pose_temp = list(self.GridForLeftArm1[6])
##                    pose_temp[2] = pose_temp[2] + 0.2
##                    self.move_arm('left', pose_temp)
##        
##            elif side =='right':
##        
##                if slot_id == 2:
##                    pose_temp = list(self.GridForRightArm1[0])
##                    pose_temp[2] = pose_temp[2] + 0.2
##                    self.move_arm('right', pose_temp)
##                elif slot_id == 5:
##                    pose_temp = list(self.GridForRightArm1[3])
##                    pose_temp[2] = pose_temp[2] + 0.2
##                    self.move_arm('right', pose_temp)
##                elif slot_id == 8:
##                    pose_temp = list(self.GridForRightArm1[6])
##                    pose_temp[2] = pose_temp[2] + 0.2
##                    self.move_arm('right', pose_temp)                
##            
            #pose_list[0] = pose_list[0]+0.14
            if counter in [2, 5, 8]:
                pose_list[0] = pose_list[0]-0.10
                slot_id = 9
            else:
                pose_list[0] = pose_list[0]+0.14
                
            grid_status, xy_list = self.check_one_grid1(side, pose_list, slot_id)
            if grid_status[0] in ['x', 'o']:
                grid_results[counter] = grid_status[0]
                
            counter = counter + 1    
            
        self.move_arm('left', self.LeftArmInitPose)
        return list(grid_results)
    
    def update_grid_status(self, side):
        
        counter = 0
        pose_list = []
        id_strings = ['00', '01', '02', '03', '04', '05', '06', '07', '08']
        
        for grid in self.GridStatus:
            slot_id = counter
            if self.GameState == 'Estop_on':
                return
                        
            if grid == 'b':
                if side == 'left':
                    if slot_id == 1:
                        pose_temp = list(self.GridForLeftArm1[0])
                        pose_temp[2] = pose_temp[2] + 0.2
                        self.move_arm('left', pose_temp)
                    elif slot_id == 4:
                        pose_temp = list(self.GridForLeftArm1[0])
                        pose_temp[2] = pose_temp[2] + 0.2
                        self.move_arm('left', pose_temp)
                    elif slot_id == 7:
                        pose_temp = list(self.GridForLeftArm1[0])
                        pose_temp[2] = pose_temp[2] + 0.2
                        self.move_arm('left', pose_temp)
        
                elif side =='right':
        
                    if slot_id == 1:
                        pose_temp = list(self.GridForRightArm1[0])
                        pose_temp[2] = pose_temp[2] + 0.2
                        self.move_arm('right', pose_temp)
                    elif slot_id == 4:
                        pose_temp = list(self.GridForRightArm1[0])
                        pose_temp[2] = pose_temp[2] + 0.2
                        self.move_arm('right', pose_temp)
                    elif slot_id == 7:
                        pose_temp = list(self.GridForRightArm1[0])
                        pose_temp[2] = pose_temp[2] + 0.2
                        self.move_arm('right', pose_temp) 
                break
            
        counter = 0    
        for grid in self.GridStatus:
            slot_id = counter
            
            msg_string = 'scang'+id_strings[slot_id]
            
            if self.GameState == 'Estop_on':
                return
            
            self.DisplayCmdPub.publish(msg_string)
            if grid == 'b':
                
                
                
                if side=='left':
                    pose_list = list(self.GridForLeftArm[counter])
                elif side =='right':
                    pose_list = list(self.GridForRightArm[counter])
                #pose_list[0] = pose_list[0]+0.14
                
                if counter in [2, 5, 8]:
                    pose_list[0] = pose_list[0]-0.10
                    slot_id = 9
                else:
                    pose_list[0] = pose_list[0]+0.14
                
                grid_status, xy_list = self.check_one_grid1(side, pose_list, slot_id)
                
                
                if grid_status[0] in ['x', 'o', 'b']:
                    
                    self.GridStatus[counter] = grid_status[0]
                    
                
            counter = counter + 1
            
        #self.move_arm('left', self.LeftArmInitPose)
        print "Updated Grid Status: ", self.GridStatus
        return
    
    def check_left_slots(self):

        left_slot_result = ['x', 'x', 'x', 'x', 'x']
        id_strings = ['00', '01', '02', '03', '04']
        slot_ids = range(0, 5)
        counter = 0
        for id in slot_ids:
            slot_id = counter
            pose_list = list(self.LeftSlotsLocation[id])
            
            if self.GameState == 'Estop_on':
                return
            
            msg_string = 'scanl'+id_strings[slot_id]
            
            self.DisplayCmdPub.publish(msg_string)
            
            if counter in [2, 4]:
                pose_list[0] = pose_list[0]-0.10
                slot_id = 9
            else:
                pose_list[0] = pose_list[0]+0.14
                
            #pose_list[3] = 1.0   
            
            slot_status, xy_list = self.check_one_grid1('left', pose_list, slot_id)
            if slot_status[0] in ['x', 'o', 'b']:
                left_slot_result[counter] = slot_status[0]
                
            counter = counter + 1
        self.move_arm('left', self.LeftArmInitPose)
        return list(left_slot_result)
    
    def update_left_slots(self):
        
        pose_list = []
        counter = 0
        id_strings = ['00', '01', '02', '03', '04']
        for slot in self.LeftSlots:
            slot_id = counter
            pose_list = list(self.LeftSlotsLocation[counter])
            
            if self.GameState == 'Estop_on':
                return
            
            msg_string = 'scanl'+id_strings[slot_id]
            
            self.DisplayCmdPub.publish(msg_string)
            #pose_list[0] = pose_list[0]+0.14
            if counter in [2, 4]:
                pose_list[0] = pose_list[0]-0.10
                slot_id = 9
            else:
                pose_list[0] = pose_list[0]+0.14
            #pose_list[3] = 1.0
            slot_status, xy_list = self.check_one_grid1('left', pose_list, slot_id)
            if slot_status[0] in ['x', 'o', 'b']:
                self.LeftSlots[counter] = slot_status[0]
            counter = counter + 1    
        
        self.move_arm('left', self.LeftArmInitPose)
        return        
        
    def check_right_slots(self):

        right_slot_result = ['o', 'o', 'o', 'o', 'o']
        id_strings = ['00', '01', '02', '03', '04']
        slot_ids = range(0, 5)
        counter = 0
        for id in slot_ids:
            slot_id = counter
            
            if self.GameState == 'Estop_on':
                return
                        
            pose_list = list(self.RightSlotsLocation[id])
            msg_string = 'scanr'+id_strings[slot_id]
            
            self.DisplayCmdPub.publish(msg_string)
            #pose_list[0] = pose_list[0]+0.14
            #pose_list[3] = 1.0
            if counter in [2, 4]:
                pose_list[0] = pose_list[0]-0.10
                slot_id = 9
            else:
                pose_list[0] = pose_list[0]+0.14
            slot_status, xy_list = self.check_one_grid1('right', pose_list, slot_id)
            if slot_status[0] in ['x', 'o', 'b']:
                right_slot_result[counter] = slot_status[0]
                
            counter = counter + 1
        
        
        self.move_arm('right', self.RightArmInitPose)
        return list(right_slot_result) 
    
    def update_right_slots(self):
        
        pose_list = []
        counter = 0
        id_strings = ['00', '01', '02', '03', '04']
        for slot in self.RightSlots:
            slot_id = counter
            pose_list = list(self.RightSlotsLocation[counter])
            
            if self.GameState == 'Estop_on':
                return
            
            msg_string = 'scanr'+id_strings[slot_id]
            
            self.DisplayCmdPub.publish(msg_string)
            #pose_list[0] = pose_list[0]+0.14
            if counter in [2, 4]:
                pose_list[0] = pose_list[0]-0.10
                slot_id = 9
            else:
                pose_list[0] = pose_list[0]+0.14
            #pose_list[3] = 1.0
            slot_status, xy_list = self.check_one_grid1('right', pose_list, slot_id)
            if slot_status[0] in ['x', 'o', 'b']:
                self.RightSlots[counter] = slot_status[0]
            counter = counter + 1
        self.move_arm('right', self.RightArmInitPose)    
        return
    
    def display_image1(self, image_string):
        
        self.DisplayCmdPub.publish(image_string)
        
        return
    
    def game_play(self):
        
        self.LeftSlots = ['b','b','b','b','b']
        self.RightSlots = ['b', 'b', 'b', 'b', 'b']
        print "Entering Play Mode..."
        
        while self.LeftSlots.count('b')>=2 or self.RightSlots.count('b')>=2:
        
            print "Check Table..."
            self.LeftSlots =  self.check_left_slots() #['x','x','x','x','x']
            print "Left slots: ", self.LeftSlots
            self.RightSlots = self.check_right_slots() #['o','o','o','o','o']
            print "Right slots: ", self.RightSlots
        
##        if (not (self.LeftSlots == ['x','x','x','x','x'] and self.RightSlots == ['o','o','o','o','o'])) \
##           or (not (self.RightSlots == ['x','x','x','x','x'] and self.LeftSlots == ['o','o','o','o','o'])):
##            self.GridStatus = self.check_all_grids('left') #['b','b','b','b','b','b','b','b','b']
##            self.display_image1('resettable')
##            print "Reset the table..."
##            self.place_all_blocks2()
##        else:
##            self.GridStatus = ['b','b','b','b','b','b','b','b','b']
##            
            if (self.LeftSlots == ['x','x','x','x','x'] and self.RightSlots == ['o','o','o','o','o']) \
               or (self.RightSlots == ['x','x','x','x','x'] and self.LeftSlots == ['o','o','o','o','o']):
                
                self.GridStatus = ['b','b','b','b','b','b','b','b','b']
                print "Left Right Slots all set, not scanning grids..."
                
            else:
                
                self.GridStatus = self.check_all_grids('left') #['b','b','b','b','b','b','b','b','b']
                self.display_image1('resettable')
                print "Reset the table..."
                self.place_all_blocks2()

            
        if self.GameState == 'Estop_on':
            return
            
            
            
        print "Grid Status: ", self.GridStatus
        
        o_side = 'right'
        if self.LeftSlots.count('o')>=4:
            o_side = 'left'
        elif self.RightSlots.count('o')>=4:
            o_side = 'right'
            
        
        
        
        #self.place_all_blocks2()
        #To Do, left arm back to init
        #**To Do, when win occurs, and grid is not full, need to check win in update_grid event in game engine
        #To Do, add head turning nodding
        #To Do, add robot winning/losing motion
        #To Do, a face d    `   isplaying node? (either direct display or using ros to publish image)
        
        
        while self.GameState != 'Estop_on':
            
            print "Session starts..."
            
            print "Place a block to start first, then press the button"
            print "Or press the button , let robot place block first"
            
            self.display_image1('waitforbutton')
            wait_status = self.wait_button_on1(0.1)
            if self.GameState == 'Estop_on':
                return
            self.display_image1('running')
            #self.head.set_pan(0.5, speed=30, timeout=0)
            rospy.sleep(2)
            #self.head.set_pan(-0.5, speed=30, timeout=0)
            #rospy.sleep(2)
            #self.head.set_pan(0.0, speed=30, timeout=0)
            #check all slots at the beginning of each game session
            #self.update_grid_status('left')
            #if (not 'x' in self.GridStatus) and (not 'o' in self.GridStatus):
            
            #self.GridStatus = self.check_all_grid('left')
            if (not (self.LeftSlots == ['x','x','x','x','x'] and self.RightSlots == ['o','o','o','o','o'])) \
              or (not (self.RightSlots == ['x','x','x','x','x'] and self.LeftSlots == ['o','o','o','o','o'])):
            
                print "Updated grid status after button pressed..."
        
                self.update_grid_status('left')
                
            self.FirstStarter = ''
            self.FirstStarterMarker = ''
            self.NextStep = ''
            self.NextMarker = ''
            
            
            msg_string = ''
            next_move = ''
            
            if 'x' in self.GridStatus:
                print "Human start first with 'x', next move shall be robot pick 'o'"
                self.FirstStarter = 'human'
                self.FirstStarterMarker = 'x'
                msg_string = 'game_engine:x:start'
                self.GridStatusPub.publish(msg_string)
                rospy.sleep(0.5)
                
                msg_string = 'game_status:x:' + self.get_grid_status_string()
                self.NextStep = 'game_engine'
                self.NextMarker = 'o'
                self.GridStatusPub.publish(msg_string)
                
                print "Game engine cmd sent: ", msg_string
                
                
                pass
                
            elif 'o' in self.GridStatus:
                print "Human start first with 'o', next move shall be robot pick 'x'"
                self.FirstStarter = 'human'
                self.FirstStarterMarker = 'o'
                msg_string = 'game_engine:o:start'
                self.GridStatusPub.publish(msg_string)
                rospy.sleep(0.5)
                
                msg_string = 'game_status:o:' + self.get_grid_status_string()
                self.NextStep = 'game_engine'
                self.NextMarker = 'x'
                self.GridStatusPub.publish(msg_string)
                rospy.sleep(0.5)
                print "Game engine cmd sent: ", msg_string
                
                pass
            
            elif self.GridStatus.count('b')==9:
                slot_id = 0
                first_grid_id = randint(0,8)
                print "Robot can start first, robot will pick from left side slots"
                self.FirstStarter = 'robot'
                if o_side == 'right':
                    print "Robot start first using x..."
                    self.FirstStarterMarker = 'x'
                    msg_string = 'game_engine:x:start'
                    self.GridStatus[first_grid_id] = 'x'
                    self.NextMarker = 'o'
                    #slot_id = self.find_in_slots('right', 'x')
                elif o_side=='left':
                    print "Robot start first using o..."
                    self.FirstStarterMarker = 'o'
                    msg_string = 'game_engine:o:start'
                    self.GridStatus[first_grid_id] = 'o'
                    self.NextMarker = 'x'
                    #slot_id = self.find_in_slots('right', 'o')
                    
                
                self.GridStatusPub.publish(msg_string)
                rospy.sleep(0.5)                
                print "Game engine cmd sent: ", msg_string
                
                #msg_string = 'game_status:x:' + self.get_grid_status_string()
                
                slot_id = self.find_in_slots('left', 'x') #x or o doesn't matter
                if slot_id in range(0, 6):
                    
                    if self.GameState == 'Estop_on':
                        return
                    self.pick_from_xy('left', self.LeftSlotsLocation[slot_id])
                    if self.GameState == 'Estop_on':
                        return
                    self.place_to_xy('left', self.GridForLeftArm1[first_grid_id])
                    if self.GameState == 'Estop_on':
                        return
                    
                    self.move_arm('left', self.LeftArmInitPose)
                    if self.GameState == 'Estop_on':
                        return
                    #self.move_to_init('left')
                self.LeftSlots[slot_id] = 'b'
                self.NextStep = 'pass_to_human'
                
                
                #self.GridStatusPub.publish(msg_string)
            

                
                pass
                
            
                
            # Debug point 1
            
            #first_play_id = randint(0,1)
            
            
            sessionDone = False
            while not sessionDone:
                if self.NextStep=='pass_to_human':
                    
                    #self.update_left_slots()
                    #self.update_right_slots()
                    
                    self.display_image1('waitforbutton')
                    print "place a block and then press the button"
                    old_grid_status = list(self.GridStatus)
                    print "Old Grid Status: ", old_grid_status
                    self.wait_button_on1(0.1)
                    self.display_image1('running')
                    self.update_grid_status('left')
                    if old_grid_status == self.GridStatus:
                        
                        print "Current Grid Status: ", self.GridStatus
                        print "Nothing changed in the grid, Human please place a block"
                        self.NextStep = 'pass_to_human'
                    elif 'o' not in self.GridStatus:
                        #if no 'o' in grids (indicate > 1 'x' in grids
                        
                        
                        pass
                    else:
                        print "Prepare message to game engine..."
                        self.NextStep = 'game_engine'
                        if self.NextMarker == 'o':
                            msg_string = 'game_status:o:' + self.get_grid_status_string()
                            self.NextMarker = 'x'
                        elif self.NextMarker == 'x':
                            msg_string = 'game_status:x:' + self.get_grid_status_string()
                            self.NextMarker = 'o'
                        self.GridStatusPub.publish(msg_string)
                        rospy.sleep(0.4)
                        
                        pass

                    
                    continue
                elif self.NextStep == 'pass_to_robot':
                    
                    self.display_image1('waitforbutton')
                    print "Human, place a block and then press the button..."
                    self.wait_button_on1(0.1)
                    self.NextStep = 'game_engine'
                    
                    continue
                elif self.NextStep == 'game_engine':
                    #print "Send request to game engine: ", msg_string
                    #rospy.sleep(2)
                    next_move = self.wait_for_next_move()
                    self.NextStep = 'pick'
                    
                
                    if next_move == 'Estop_on' or self.GameState == 'Estop_on':
                        
                        self.LeftSlots = ['x','x','x','x','x']
                        self.RightSlots = ['o','o','o','o','o']
                        self.GridStatus = ['b','b','b','b','b','b','b','b','b']
                        msg_string = 'game_engine:o:quit_session'
                        self.GridStatusPub.publish(msg_string)
                        
                        return
                
                    item, id = self.interpret_next_move(next_move)
                    print "Item: ", item, "Id: ", id
                    
                    
                    if o_side == 'right' and item in ['x'] and id in range(0, 9):
                        
                        #self.display_image('leftplay')
                        self.display_image1('myturn')
                        self.move_arm('right', self.RightArmInitPose)
                        slot_id = self.find_in_slots('left', 'x')
                        print "Pick x from left slot: ", self.LeftSlots[slot_id]
                        print "place it to: ", self.GridForLeftArm[id]
                        if self.GameState == 'Estop_on':
                            return
                        
                        if slot_id in [2, 4]:
                            pose_temp = list(self.LeftSlotsLocation[0])
                            pose_temp[2] = pose_temp[2] + 0.2
                            self.move_arm('left', pose_temp)
                        self.pick_from_xy('left', self.LeftSlotsLocation[slot_id])
                        if self.GameState == 'Estop_on':
                            return
                        
                        if id == 2:
                            pose_temp = list(self.GridForLeftArm1[0])
                            pose_temp[2] = pose_temp[2] + 0.2
                            self.move_arm('left', pose_temp)
                        elif id == 5:
                            pose_temp = list(self.GridForLeftArm1[3])
                            pose_temp[2] = pose_temp[2] + 0.2
                            self.move_arm('left', pose_temp)
                        elif id == 8:
                            pose_temp = list(self.GridForLeftArm1[6])
                            pose_temp[2] = pose_temp[2] + 0.2
                            self.move_arm('left', pose_temp)
                            
                
                        self.place_to_xy('left', self.GridForLeftArm1[id])
                        if self.GameState == 'Estop_on':
                            return
                        self.move_arm('left', self.LeftArmInitPose)
                        if self.GameState == 'Estop_on':
                            return
                        print "Left Arm Back to Init Position..."
                        #self.move_to_init('left')
                        self.LeftSlots[slot_id] = 'b'
                        self.GridStatus[id] = item
                        msg_string = 'update_grid:x:' + self.get_grid_status_string()
                        self.GridStatusPub.publish(msg_string)
                        next_move1 = self.wait_for_next_move()
                    
                        item1, id1 = self.interpret_next_move(next_move1)
                        print "Item: ", item, "Id: ", id1
                        if item1=='win':
                            
                            if id1==0:
                                #self.display_image1('rightwin')
                                print "X won..."
                                if self.FirstStarterMarker == 'x' and self.FirstStarter=='human':
                                    self.display_image1('humanwin')
                                elif self.FirstStarterMarker == 'x' and self.FirstStarter=='robot':
                                    self.display_image1('robotwin')
                            elif id1==1:
                                #self.display_image1('leftwin')
                                print "O won..."
                                if self.FirstStarterMarker == 'o' and self.FirstStarter=='human':
                                    self.display_image1('humanwin')
                                elif self.FirstStarterMarker == 'o' and self.FirstStarter=='robot':
                                    self.display_image1('robotwin')
                            
                                
                            sessionDone = True
                            self.GameStatus = 'Done'
                            rospy.sleep(2)
                            self.NextPlayer = ''
                            self.NextStep = ''
                            break
                        elif item1 == 'draw':
                            
                            self.display_image1('draw')
                            sessionDone = True
                            self.GameStatus = 'Done'
                            rospy.sleep(2)
                            self.NextPlayer = ''
                            self.NextStep = ''
                            
                            
                            break
                            
                        self.NextPlayer = 'human'
                        self.NextStep = 'pass_to_human'
                        self.NextMarker = 'o'
                        
                    elif o_side == 'left' and item in ['x'] and id in range(0, 9):
                        
                        #self.display_image('leftplay')
                        self.display_image1('myturn')
                        self.move_arm('left', self.LeftArmInitPose)
                        slot_id = self.find_in_slots('right', 'x')
                        print "Pick x from right slot: ", self.RightSlots[slot_id]
                        print "place it to: ", self.GridForRightArm[id]
                        if self.GameState == 'Estop_on':
                            return
                        self.display_image1('myturn')
                        if slot_id in [2, 4]:
                            pose_temp = list(self.RightSlotsLocation[0])
                            pose_temp[2] = pose_temp[2] + 0.2
                            self.move_arm('right', pose_temp)
                        self.pick_from_xy('right', self.RightSlotsLocation[slot_id])
                        if self.GameState == 'Estop_on':
                            return
                        
                        if id == 2:
                            pose_temp = list(self.GridForRightArm1[0])
                            pose_temp[2] = pose_temp[2] + 0.2
                            self.move_arm('right', pose_temp)
                        elif id == 5:
                            pose_temp = list(self.GridForRightArm1[3])
                            pose_temp[2] = pose_temp[2] + 0.2
                            self.move_arm('right', pose_temp)
                        elif id == 8:
                            pose_temp = list(self.GridForRightArm1[6])
                            pose_temp[2] = pose_temp[2] + 0.2
                            self.move_arm('right', pose_temp)
                            
                
                        self.place_to_xy('right', self.GridForRightArm1[id])
                        if self.GameState == 'Estop_on':
                            return
                        self.move_arm('right', self.RightArmInitPose)
                        if self.GameState == 'Estop_on':
                            return
                        print "Right Arm Back to Init Position..."
                        #self.move_to_init('left')
                        self.RightSlots[slot_id] = 'b'
                        self.GridStatus[id] = item
                        msg_string = 'update_grid:x:' + self.get_grid_status_string()
                        self.GridStatusPub.publish(msg_string)
                        next_move1 = self.wait_for_next_move()
                    
                        item1, id1 = self.interpret_next_move(next_move1)
                        print "Item: ", item, "Id: ", id1
                        if item1=='win':
                            
                            if id1==0:
                                #self.display_image1('rightwin')
                                print "X won..."
                                if self.FirstStarterMarker == 'x' and self.FirstStarter=='human':
                                    self.display_image1('humanwin')
                                elif self.FirstStarterMarker == 'x' and self.FirstStarter=='robot':
                                    self.display_image1('robotwin')
                            elif id1==1:
                                #self.display_image1('leftwin')
                                print "O won..."
                                if self.FirstStarterMarker == 'o' and self.FirstStarter=='human':
                                    self.display_image1('humanwin')
                                elif self.FirstStarterMarker == 'o' and self.FirstStarter=='robot':
                                    self.display_image1('robotwin')
                            
                                
                            sessionDone = True
                            self.GameStatus = 'Done'
                            rospy.sleep(2)
                            self.NextPlayer = ''
                            self.NextStep = ''
                            break
                        elif item1 == 'draw':
                            
                            self.display_image1('draw')
                            sessionDone = True
                            self.GameStatus = 'Done'
                            rospy.sleep(2)
                            self.NextPlayer = ''
                            self.NextStep = ''
                            
                            
                            break
                            
                        self.NextPlayer = 'human'
                        self.NextStep = 'pass_to_human'
                        self.NextMarker = 'o'
                    
                    
                    elif o_side=='right' and item in ['o'] and id in range(0, 9):
                        
                        self.display_image1('myturn')
                        self.move_arm('left', self.LeftArmInitPose)
                        
                        slot_id = self.find_in_slots('right', 'o')
                        print "Pick o from right : ", self.RightSlots[slot_id]
                        print "place it to: ", self.GridForRightArm[id]
                        if self.GameState == 'Estop_on':
                            return
                        #self.display_image1('myturn')
                        if slot_id in [2, 4]:
                            pose_temp = list(self.RightSlotsLocation[0])
                            pose_temp[2] = pose_temp[2] + 0.2
                            self.move_arm('right', pose_temp)
                        self.pick_from_xy('right', self.RightSlotsLocation[slot_id])
                        
                        
                        if self.GameState == 'Estop_on':
                            return
                        
                        
                        
                        if id == 2:
                            pose_temp = list(self.GridForRightArm1[0])
                            pose_temp[2] = pose_temp[2] + 0.2
                            self.move_arm('right', pose_temp)
                        elif id == 5:
                            pose_temp = list(self.GridForRightArm1[3])
                            pose_temp[2] = pose_temp[2] + 0.2
                            self.move_arm('right', pose_temp)
                        elif id == 8:
                            pose_temp = list(self.GridForRightArm1[6])
                            pose_temp[2] = pose_temp[2] + 0.2
                            self.move_arm('right', pose_temp)
                        
                        self.place_to_xy('right', self.GridForRightArm1[id])
                        if self.GameState == 'Estop_on':
                            return
                        self.move_arm('right', self.RightArmInitPose)
                        if self.GameState == 'Estop_on':
                            return
                        print "Right Arm Back to Init Position..."
                        #self.move_to_init('right')
                        self.GridStatus[id] = item
                        self.RightSlots[slot_id] = 'b'
                        msg_string = 'update_grid:o:' + self.get_grid_status_string()
                        self.GridStatusPub.publish(msg_string)
                        
                        next_move1 = self.wait_for_next_move()
                    
                        item1, id1 = self.interpret_next_move(next_move1)
                        print "Item: ", item, "Id: ", id1
                        if item1=='win':
                            
                            if id1==0:
                                #self.display_image('rightwin')
                                print "X won..."
                                if self.FirstStarterMarker == 'x' and self.FirstStarter=='human':
                                    self.display_image1('humanwin')
                                    rospy.sleep(2)
                                elif self.FirstStarterMarker == 'x' and self.FirstStarter=='robot':
                                    self.display_image1('robotwin')
                                    rospy.sleep(2)
                            elif id1==1:
                                #self.display_image('leftwin')
                                print "O won..."
                                if self.FirstStarterMarker == 'o' and self.FirstStarter=='human':
                                    self.display_image1('humanwin')
                                    rospy.sleep(2)
                                elif self.FirstStarterMarker == 'o' and self.FirstStarter=='robot':
                                    self.display_image1('robotwin')
                                    rospy.sleep(2)
                            
                                
                            sessionDone = True
                            self.GameStatus = 'Done'
                            rospy.sleep(2)
                            self.NextPlayer = ''
                            self.NextStep = ''
                            break
                        
                        elif item1 == 'draw':
                            
                            self.display_image1('draw')
                            sessionDone = True
                            self.GameStatus = 'Done'
                            rospy.sleep(2)
                            self.NextPlayer = ''
                            self.NextStep = ''
                            
                            break
                        
                        self.NextPlayer = 'human'
                        self.NextStep = 'pass_to_human'
                        self.NextMarker = 'x'
                    
                    elif o_side=='left' and item in ['o'] and id in range(0, 9):
                        
                        self.display_image1('myturn')
                        self.move_arm('right', self.RightArmInitPose)
                        
                        slot_id = self.find_in_slots('left', 'o')
                        print "Pick o from Left : ", self.LeftSlots[slot_id]
                        print "place it to: ", self.GridForLeftArm[id]
                        if self.GameState == 'Estop_on':
                            return
                        #self.display_image1('myturn')
                        if slot_id in [2, 4]:
                            pose_temp = list(self.LeftSlotsLocation[0])
                            pose_temp[2] = pose_temp[2] + 0.2
                            self.move_arm('left', pose_temp)
                        self.pick_from_xy('left', self.RightSlotsLocation[slot_id])
                        
                        
                        if self.GameState == 'Estop_on':
                            return
                        
                        
                        
                        if id == 2:
                            pose_temp = list(self.GridForLeftArm1[0])
                            pose_temp[2] = pose_temp[2] + 0.2
                            self.move_arm('left', pose_temp)
                        elif id == 5:
                            pose_temp = list(self.GridForLeftArm1[3])
                            pose_temp[2] = pose_temp[2] + 0.2
                            self.move_arm('left', pose_temp)
                        elif id == 8:
                            pose_temp = list(self.GridForLeftArm1[6])
                            pose_temp[2] = pose_temp[2] + 0.2
                            self.move_arm('left', pose_temp)
                        
                        self.place_to_xy('left', self.GridForLeftArm1[id])
                        if self.GameState == 'Estop_on':
                            return
                        self.move_arm('left', self.RightArmInitPose)
                        if self.GameState == 'Estop_on':
                            return
                        print "Right Arm Back to Init Position..."
                        #self.move_to_init('right')
                        self.GridStatus[id] = item
                        self.RightSlots[slot_id] = 'b'
                        msg_string = 'update_grid:o:' + self.get_grid_status_string()
                        self.GridStatusPub.publish(msg_string)
                        
                        next_move1 = self.wait_for_next_move()
                    
                        item1, id1 = self.interpret_next_move(next_move1)
                        print "Item: ", item, "Id: ", id1
                        if item1=='win':
                            
                            if id1==0:
                                #self.display_image('rightwin')
                                print "X won..."
                                if self.FirstStarterMarker == 'x' and self.FirstStarter=='human':
                                    self.display_image1('humanwin')
                                    rospy.sleep(2)
                                elif self.FirstStarterMarker == 'x' and self.FirstStarter=='robot':
                                    self.display_image1('robotwin')
                                    rospy.sleep(2)
                            elif id1==1:
                                #self.display_image('leftwin')
                                print "O won..."
                                if self.FirstStarterMarker == 'o' and self.FirstStarter=='human':
                                    self.display_image1('humanwin')
                                    rospy.sleep(2)
                                elif self.FirstStarterMarker == 'o' and self.FirstStarter=='robot':
                                    self.display_image1('robotwin')
                                    rospy.sleep(2)
                            
                                
                            sessionDone = True
                            self.GameStatus = 'Done'
                            rospy.sleep(2)
                            self.NextPlayer = ''
                            self.NextStep = ''
                            break
                        
                        elif item1 == 'draw':
                            
                            self.display_image1('draw')
                            sessionDone = True
                            self.GameStatus = 'Done'
                            rospy.sleep(2)
                            self.NextPlayer = ''
                            self.NextStep = ''
                            
                            break
                        
                        self.NextPlayer = 'human'
                        self.NextStep = 'pass_to_human'
                        self.NextMarker = 'x'
                    
                    elif item == 'draw':
                        
                        self.display_image1('draw')
                        sessionDone = True
                        self.GameStatus = 'Done'
                        rospy.sleep(2)
                        self.NextPlayer = ''
                        self.NextStep = ''
                        break
                        
                    elif item == 'win':
                        
                        
                        if id==0:
                            #self.display_image('rightwin')
                            if self.FirstStarterMarker == 'x' and self.FirstStarter=='human':
                                self.display_image1('humanwin')
                                rospy.sleep(2)
                            elif self.FirstStarterMarker == 'x' and self.FirstStarter=='robot':
                                self.display_image1('robotwin')
                                rospy.sleep(2)
                        elif id==1:
                            #self.display_image('leftwin')
                            if self.FirstStarterMarker == 'o' and self.FirstStarter=='human':
                                self.display_image1('humanwin')
                                rospy.sleep(2)
                            elif self.FirstStarterMarker == 'o' and self.FirstStarter=='robot':
                                self.display_image1('robotwin')
                                rospy.sleep(2)
                        sessionDone = True
                        self.GameStatus = 'Done'
                        rospy.sleep(2)
                        self.NextPlayer = ''
                        self.NextStep = ''
                        break
            
##                self.update_grid_status('left')
##                self.update_left_slots()
##                self.update_right_slots()
##                self.display_image('waitforbutton')
##                print "Wait for User input..."
##                self.wait_button_on1(0.1)
##                if self.NextPlayer!='':
##                    if self.NextPlayer=='human':
##                        print "Next Turn will be robot"
##                        self.NextPlayer = 'robot'
##                    elif self.NextPlayer == 'robot':
##                        print "Next turn will be human"
##                        self.NextPlayer = 'human'
##                
            
            self.move_arm('left', self.LeftArmInitPose)
            
            while self.LeftSlots.count('b')>=2 or self.RightSlots.count('b')>=2:
                print "Check Table at the End..."
                self.display_image1('manualresettable')
                self.wait_button_on1(1.5)
                if self.GameState == 'Estop_on':
                    return
                self.LeftSlots =  self.check_left_slots() #['x','x','x','x','x']
                print "Left slots: ", self.LeftSlots
                self.RightSlots = self.check_right_slots() #['o','o','o','o','o']
                print "Right slots: ", self.RightSlots
                if (self.LeftSlots == ['x','x','x','x','x'] and self.RightSlots == ['o','o','o','o','o']) \
                  or (self.RightSlots == ['x','x','x','x','x'] and self.LeftSlots == ['o','o','o','o','o']):
                    
                    self.GridStatus = ['b','b','b','b','b','b','b','b','b']
                
                
                #self.GridStatus = ['b','b','b','b','b','b','b','b','b']
                    print "Left Right Slots all set, not scanning grids..."
                else:
                    self.GridStatus = self.check_all_grids('left')
                #self.GridStatus = self.check_all_grids('left') #['b','b','b','b','b','b','b','b','b']
                    print "Grid Status: ", self.GridStatus
                    self.display_image1('resettable')
                    self.place_all_blocks2()
                    
                
            if self.LeftSlots.count('o')>=4:
                o_side = 'left'
            elif self.RightSlots.count('o')>=4:
                o_side = 'right'    
                
                
            
                
        
        
        
        return
        
    def robot_head(self):
        
        
        
        return 
    
    def robot_win_motion(self):
        
        
        
        
        return
    
    def run(self):
        
        self.GameState = 'NoInit'
        print "Loading Game Data..."
        self.display_image1('running')
        #self.load_location_data()
        #rs = baxter_interface.RobotEnable(CHECK_VERSION)
        #rs.enable()
        
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        restart_status = False
        while not restart_status or self.GameState=='Estop_on':
            try:
                if self.GameState=='Estop_on':
                    
                    self.display_image1('estop')
                    print "E Stop is on, waiting for a reset"
                    self.wait_for_estop()
                    rs.reset()
                    self.GameState = 'Estop_reset'
                    
                rs.enable()
                restart_status = True
                
                #self.display_image('running')
            except Exception, e:
                rospy.logerr(e.strerror)
                restart_status = False
            rospy.sleep(0.1)
        
        #self.pick_test(1,0)
        self.display_image1('waitforbutton')
        print "Wait For Button"
        
        wait_status = self.wait_button_on1(0.1) # User has to keep pressing the button for at least 1 second
        while wait_status== -1 or self.GameState=='Estop_on':
            
            if self.GameState=='Estop_on':
                self.display_image1('estop')
                print "E Stop is on, waiting for a reset"
                self.wait_for_estop()
                self.reset_estop()
                #rospy.sleep(0.1)
            
            elif self.GameState == 'Estop_reset':
                self.display_image1('waitforbutton')
                print "Wait for Button"
                wait_status = self.wait_button_on1(0.1)
                #rospy.sleep(0.1)
            rospy.sleep(0.1)
            
            
        restart_status = False
        while not restart_status:
            try:
                rs.reset()
                rs.enable()
                
                restart_status = True
                #self.display_image('running')
            except Exception, e:
                rospy.logerr(e.strerror)
                restart_status = False
            rospy.sleep(0.1)
            
        self.GameState = 'NoInit'    
        self.display_image1('running')
        ##self.pck_item('left', 'x')
        ##self.place_item('left', 1, 1)
        #self.check_grid('left')
        #grid_status = self.check_grid('left')
        #print "Grid Status: ", grid_status
        #self.place_all_blocks()
        grid_xy = [[0,0], [1,0], [2,0], [0,1], [1,1], [2,1], [0,2], [1,2], [2,2]]
        counter = 0
        
        

        #msg_string = 'game_status:x:'
        
        #for item in grid_status:
        #    msg_string = msg_string+item + ' '
            
        #self.GridStatusPub.publish(msg_string)
        
        while not rospy.is_shutdown():
            
##            t = time.localtime(time.time())
##            print "Current Time: ", t.tm_hour, ":", t.tm_min
##            if (t.tm_hour >= 16 and t.tm_min>45) or (t.tm_hour<=9 and t.tm_min<30):
##                print "Not in working time period"
##                rospy.sleep(1)
##                continue
            if self.GameState == 'WaitForButton':
                
                print "Enter WaitForButton Step"
                self.display_image1('waitforbutton')
                wait_status = self.wait_button_on1(0.1)
                if wait_status == -1 and self.GameState!='Estop_on':
                    self.GameState = 'Estop_reset'
                    rospy.sleep(1)
                    continue
                elif wait_status ==-1 and self.GameState=='Estop_on':
                    #rospy.sleep(0.1)
                    continue
                pass
                
            elif self.GameState == 'NoInit':
                
                print "Enter NoInit Step"
                self.init_game()
                
                print "Game Initilized"
                self.GameState = 'Init'
                
                
            
        
            elif self.GameState == 'Init':
                
                print "Enter Init Step"
                if self.GameError == 'NoInit':
                    self.GameState = 'Estop_reset'
                    self.GameError = ''
                    print "NoInit Error, will Re Init"
                    rospy.sleep(0.1)
                    continue
                if 'o' in self.GridStatus or 'x' in self.GridStatus:
                    
                    #self.place_all_blocks()
                    pass
                self.GameState = 'InitDone'
            
            elif self.GameState == 'Done':
                
                self.GameState = 'InitDone'
                rospy.sleep(1)
                
            elif self.GameState == 'InitDone':
                
                #self.display_image('running')
                #self.demo_play()
                self.game_play()
                
            elif self.GameState == 'Estop_on':
                
                print "Enter Estop on Step"
                self.display_image1('estop')
                print "E Stop is on, waiting for a reset"
                self.wait_for_estop()
                self.GameState = 'Estop_reset'
                
                print "Estop reset, ready to move on"
                pass
                
            elif self.GameState == 'Estop_reset':
                
                print "Enter Estop reset Step"
                
                self.display_image1('waitforbutton')
                print "Press Button to Re enalbe robot"
                wait_status = self.wait_button_on1(0.1)
                if wait_status==-1:
                    rospy.sleep(0.1)
                    continue
                rs = baxter_interface.RobotEnable(CHECK_VERSION)
                restart_status = False
                while not restart_status:
                    try:
                        rs.reset()
                        rs.enable()
                        restart_status = True
                        self.display_image1('running')
                    except Exception, e:
                        rospy.logerr(e.strerror)
                        restart_status = False
                    rospy.sleep(0.1)
                
                print "Task Restart"
                self.GameState = 'NoInit'
                
                
            
            
                
                
                
                
                
                
            
            '''
            item, id = self.interpret_next_move(self.wait_for_next_move())
            if item == '':
                rospy.sleep(0.1)
                continue
            #elif id>9:
                
            self.pick_item('left', item)
            col = grid_xy[id][0]
            row = grid_xy[id][1]
            self.place_item('left', col, row)
            rospy.sleep(4)
            grid_status = self.check_grid('left')
            
            msg_string = 'game_status:x:'
        
            for item in grid_status:
                msg_string = msg_string+item + ' '
                
            self.GridStatusPub.publish(msg_string)
            
            self.wait_button_on(0.2)
            '''
            
            '''
            grid_status = self.check_grid('left')
            print "previous grid status: ", grid_status
            print "Current grid status: ", grid_status1
            if grid_status1 == grid_status:
                print "Grid Status not Changed..."
                rospy.sleep(0.1)
                continue
            
            counter = 0
            for grid in grid_status1:
                
                if grid == 'b':
                    self.pick_item('left', 'x')
                    col = grid_xy[counter][0]
                    row = grid_xy[counter][1]
                    self.place_item('left', col, row)
                    
                    break
                
                counter = counter + 1
            
            grid_status = list(grid_status1)
            '''
            
            rospy.sleep(0.1)
        
    
    def vision_reply_callback(self, msg):
        self.VisionReply = msg.data
        
    def arms_reply_callback(self, msg):
        self.ArmReply = msg.data
    
    
def signal_handler(signal, frame):
    print 'You pressed Ctrl+C!'
    sys.exit(0)
    
    
def main():

    signal.signal(signal.SIGINT, signal_handler)
    ttt_game = TigTagToe()
    ttt_game.init_msgs()
    
    ttt_game.run()
    

if __name__ == '__main__':
    
    try:
        main()
    except KeyboardInterrupt:
        print "\nCtrl-C Pressed\n"
        flag = True
        sys.exit()
    
    
    
    