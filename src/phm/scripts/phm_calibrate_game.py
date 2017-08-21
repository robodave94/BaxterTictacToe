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
import baxter_external_devices
from baxter_interface import CHECK_VERSION
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


class GameCalibrator(object):
    
    def __init__(self, arm):
        
        self.arm = arm
        self.GridLocations = []
        self.GridRoiPose = None
        self.GridRois = []
        self.current_poses = None
        self.OcvBridge = CvBridge()
        self.cur_img = None                             
        self.ImageThreadLock = threading.Lock()
        
        self.ArmCmdPub = rospy.Publisher('robot_arms_cmd', String, queue_size=10)
        
        camera_sub = rospy.Subscriber('/cameras/'+ self.arm + '_hand_camera/image', \
                                       Image, self._camera_callback)
                                    
        self.height = 600
        self.width = 960
        self._camera = baxter_interface.CameraController(self.arm + '_hand_camera')
        
        
        self._camera.open()
        self._camera.resolution = [self.width, self.height]
        self._camera.gain = 10
        rospy.init_node("game_calibrator")
        
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        left = baxter_interface.Gripper('left', CHECK_VERSION)
        right = baxter_interface.Gripper('right', CHECK_VERSION)
        self.baxter_grippers = {'left':left, 'right':right}
        
        self.mx = 0
        self.my = 0
        self.GridRoiDoneFlag = False
        self.GridRoiCounter = 0
        self.TempGridList = []
    
    def init_msgs(self):
        
        left_arm_msg = message_filters.Subscriber("/robot/limb/left/endpoint_state",EndpointState)
        right_arm_msg = message_filters.Subscriber("/robot/limb/right/endpoint_state",EndpointState)
        left_ir_msg = message_filters.Subscriber('/robot/range/left_hand_range/state',Range)
        right_ir_msg = message_filters.Subscriber('/robot/range/right_hand_range/state',Range)
        ts = message_filters.ApproximateTimeSynchronizer([left_arm_msg, right_arm_msg, left_ir_msg, right_ir_msg], 10, 0.05)
        ts.registerCallback(self.pose_callback)    
    
    def pose_callback(self, left_msg, right_msg, left_ir_msg, right_ir_msg):
    
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
        
    
    def get_image(self):
        c_img = None
        with self.ImageThreadLock:
            c_img = deepcopy(self.cur_img)
        
        return c_img

    def make_pose_stamp(self, pose_list, header):
        
        #print "\npose list: ", pose_list
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
    
    def _camera_callback(self, image):
        with self.ImageThreadLock:
            try:
                self.cur_img = self.OcvBridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
                #cv_img = deepcopy(self.cur_img)
            except Exception:
                
                print 'OH NO - IMAGE WENT WRONG!!'

    def save_grid_locations(self):
        if len(self.GridLocations) !=9:
            print "Number of Grids recorded is wrong"
            return
        file = open("./src/phm/grids_location.txt", "w")
        for pose in self.GridLocations:
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z
            ox = pose.pose.orientation.x
            oy = pose.pose.orientation.y
            oz = pose.pose.orientation.z
            ow = pose.pose.orientation.w
            
            file.write("%.4f %.4f %.4f %.4f %4f %.4f %.4f\n" % (x, y, z, ox, oy, oz, ow))
        
        file.close()
            
            
    
    def grids_location_recorder(self):
        
        done_flag = False
        #x_counter = 0
        #y_counter = 0
        counter = 0
        self.GridLocations = []
        print "Enter Keyboard Command:"
        while not done_flag and not rospy.is_shutdown():
            
            c = baxter_external_devices.getch()
            if c:
                if c in ['q']:
                    print "Grid Location Recorder Stops"
                    done_flag = True
                elif c in ['r']:
                    cur_pose = self.current_poses[self.arm]
                    self.GridLocations.append(cur_pose) # will be 9 in total
                    print "Current Pose", cur_pose
                    print len(self.GridLocations), " number of grid location recorded"
                elif c in ['u']:
                    pass    
            #rospy.sleep(0.1)   
        print "Saving Grid Location Data"
        self.save_grid_locations()
        
    def mouse_callback(self, event, x, y, flags, param):
        
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.GridRoiCounter<4:
                self.TempGridList.append([x,y])
                self.GridRoiCounter = self.GridRoiCounter + 1
            if self.GridRoiCounter == 4:
                #self.TempGridList.append([x,y])
                if len(self.GridRois)<9:
                    self.GridRois.append(self.TempGridList)
                else:
                    print "All Roi are Recorded already"
                self.GridRoiCounter = 0
                print "Current ROi Points: ", self.TempGridList
                self.TempGridList = []
            
            
            
        elif event == cv2.EVENT_MOUSEMOVE:
            
            pass
            
        elif event == cv2.EVENT_LBUTTONUP:
            
            pass
            
        
    
    def save_roi_location(self):
        
        file = open("./src/phm/grids_roi_location.txt", "w")
        for r in self.GridRois:
            for item in r:
                x = item[0]
                y = item[1]
                file.write("%d %d " % (x,y))
            file.write('\n')
        pose = self.GridRoiPose
        x = pose.pose.position.x
        y = pose.pose.position.y
        z = pose.pose.position.z
        ox = pose.pose.orientation.x
        oy = pose.pose.orientation.y
        oz = pose.pose.orientation.z
        ow = pose.pose.orientation.w
        
        file.write("%.4f %.4f %.4f %.4f %4f %.4f %.4f" % (x, y, z, ox, oy, oz, ow))
        file.close()
    
    def grids_roi_recorder(self):
        
        cv2.namedWindow('current_image')
        cv2.setMouseCallback('current_image', self.mouse_callback)
        done_flag = False
        
        self.GridRoiCounter = 0
        self.GridRois = []
        while (not done_flag) and (not rospy.is_shutdown()):
            
            img = self.get_image()
            if img == None:
                rospy.sleep(0.1)
                continue
            
            cv2.imshow('current_image', img)
            key = cv2.waitKey(10)
            if key == 27:
                print "Quiting Roi Recording..."
                self.GridRoiPose = self.current_poses[self.arm]
                print "Pose to Capture Roi: ", self.GridRoiPose
                done_flag = True
                
            elif key == 13:
                
                self.GridRoiPose = self.current_poses[self.arm]
                print "Pose to Capture Roi: ", self.GridRoiPose
        
        print "Saving Grid Roi..."
        self.save_roi_location()

    def run(self):
        
        #print "Recording Grids Location..."
        #self.grids_location_recorder()
        #print "Recording Grid Roi..."
        #self.grids_roi_recorder()
        
        cur_arm = 'left'
        
        step = 0.004
        print "Input: "
        while not rospy.is_shutdown():
            
            
            
            c = baxter_external_devices.getch(1.0)
            
            
            if c:
                if c in ['w']:
                    
                    pose = self.current_poses['left']
                    x = pose.pose.position.x
                    y = pose.pose.position.y
                    z = -0.0247 #pose.pose.position.z
                    ox = 0.0
                    oy = 1.0
                    oz = 0.0
                    ow = 0.0
                    
                    print " "
                    print "Get w"
                    x = x + step
                    cur_arm = 'left'
                
                elif c in ['z']:
                    
                    pose = self.current_poses['left']
                    x = pose.pose.position.x
                    y = pose.pose.position.y
                    z = -0.0247 #pose.pose.position.z
                    ox = 0.0
                    oy = 1.0
                    oz = 0.0
                    ow = 0.0
                    print " "
                    print "Get z"
                    x = x - step
                    cur_arm = 'left'
                    
                elif c in ['a']:
                    pose = self.current_poses['left']
                    x = pose.pose.position.x
                    y = pose.pose.position.y
                    z = -0.0247 #pose.pose.position.z
                    ox = 0.0
                    oy = 1.0
                    oz = 0.0
                    ow = 0.0
                    print " "
                    print "Get a"
                    y = y + step
                    cur_arm = 'left'
                    
                elif c in ['s']:
                    
                    pose = self.current_poses['left']
                    x = pose.pose.position.x
                    y = pose.pose.position.y
                    z = -0.0247 #pose.pose.position.z
                    ox = 0.0
                    oy = 1.0
                    oz = 0.0
                    ow = 0.0
                    
                    print " "
                    print "Get s"
                    y = y - step
                    cur_arm = 'left'
                    
                if c in ['i']:
                    
                    pose = self.current_poses['right']
                    x = pose.pose.position.x
                    y = pose.pose.position.y
                    z = -0.0247 #pose.pose.position.z
                    ox = 0.0
                    oy = 1.0
                    oz = 0.0
                    ow = 0.0
                    print " "
                    print "Get i"
                    x = x + step
                    cur_arm = 'right'
                
                elif c in ['m']:
                    
                    pose = self.current_poses['right']
                    x = pose.pose.position.x
                    y = pose.pose.position.y
                    z = -0.0247 #pose.pose.position.z
                    ox = 0.0
                    oy = 1.0
                    oz = 0.0
                    ow = 0.0
                    print " "
                    print "Get m"
                    x = x - step
                    cur_arm = 'right'
                    
                elif c in ['j']:
                    
                    pose = self.current_poses['right']
                    x = pose.pose.position.x
                    y = pose.pose.position.y
                    z = -0.0247 #pose.pose.position.z
                    ox = 0.0
                    oy = 1.0
                    oz = 0.0
                    ow = 0.0
                    print " "
                    print "Get j"
                    y = y + step
                    cur_arm = 'right'
                    
                elif c in ['k']:
                    
                    pose = self.current_poses['right']
                    x = pose.pose.position.x
                    y = pose.pose.position.y
                    z = -0.0247 #pose.pose.position.z
                    ox = 0.0
                    oy = 1.0
                    oz = 0.0
                    ow = 0.0
                    y = y - step
                    cur_arm = 'right'  
                    print " "
                    print "Get k"
                
                
            else:
                rospy.sleep(0.1)
                continue
                    
                    
            msg_string = cur_arm + ':move_to:' + \
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
            
            rospy.sleep(0.1)
        
        


def signal_handler(signal, frame):
    print 'You pressed Ctrl+C!'
    sys.exit(0)
    
    
def main():

    signal.signal(signal.SIGINT, signal_handler)
    gc = GameCalibrator('left')
    gc.init_msgs()
    
    rospy.sleep(2)
    gc.run()
    

if __name__ == '__main__':
    
    try:
        main()
    except KeyboardInterrupt:
        print "\nCtrl-C Pressed\n"
        flag = True
        sys.exit()
    











