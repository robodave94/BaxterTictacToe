#!/usr/bin/python

########################################################
# Copyright (c) 2015-2017, Robological
# All rights reserved.
#
# This file is to use the Baxter robot to collect image
# for offline testing of shape matching
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

class ImageCollecting(object):
    
    def __init__(self, arm, counter):
        
        self.arm = arm
        self.current_poses = None
        self.OcvBridge = CvBridge()
        self.ImageThreadLock = threading.Lock()
        self.ArmCmdPub = rospy.Publisher('robot_arms_cmd', String, queue_size=10)
        self.ArmReply = ''
        self.counter = counter
        rospy.init_node("data_collecting")
        camera_sub = rospy.Subscriber('/cameras/'+ arm + '_hand_camera/image', \
                                       Image, self._camera_callback)
                                    
                                    
        self.height = 600
        self.width = 960
        self._camera = baxter_interface.CameraController(self.arm + '_hand_camera')
        
        
        self._camera.open()
        self._camera.resolution = [self.width, self.height]
        self._camera.gain = 5
        self.cam_calib    = 0.0025                     # meters per pixel at 1 meter
        self.cam_x_offset = 0.045                      # camera gripper offset
        self.cam_y_offset = -0.01
        
    def get_image(self):
        c_img = None
        with self.ImageThreadLock:
            c_img = deepcopy(self.cur_img)
        
        return c_img

    def move_arm(self, side, target_pose):
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
        cur_pose = self.current_poses[self.arm]
        dist_x = math.fabs(cur_pose.pose.position.x-x)
        dist_y = math.fabs(cur_pose.pose.position.y-y)
        dist_z = math.fabs(cur_pose.pose.position.z-z)
        
        while dist_x>0.005 or dist_y>0.005 or dist_z>0.005:
            cur_pose = self.current_poses[side]
            
            dist_x = math.fabs(cur_pose.pose.position.x-x)
            dist_y = math.fabs(cur_pose.pose.position.y-y)
            dist_z = math.fabs(cur_pose.pose.position.z-z)
            rospy.sleep(0.1)
        print "Postion Reached", dist_x, dist_y, dist_z
        print target_pose
    
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
        
    
    def _camera_callback(self, image):
        with self.ImageThreadLock:
            try:
                self.cur_img = self.OcvBridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
                #cv_img = deepcopy(self.cur_img)
            except Exception:
                
                print 'OH NO - IMAGE WENT WRONG!!'
                
        cv2.imshow('current_image', self.cur_img)
        cv2.waitKey(5)
    
    def capture_image(self, image_filename):
        
        if image_filename == '':
            print "Error, filename of image is empty"
            return
        
        file = open("./src/phm/image_pose.txt", "a")
        cur_pose = self.current_poses[self.arm]
        x = cur_pose.pose.position.x
        y = cur_pose.pose.position.y
        z = cur_pose.pose.position.z
        ox = cur_pose.pose.orientation.x
        oy = cur_pose.pose.orientation.y
        oz = cur_pose.pose.orientation.z
        ow = cur_pose.pose.orientation.w
        
        img = self.get_image()
        if img == None:
            rospy.sleep(0.05)
            return
        
        cur_pose = self.current_poses[self.arm]
        self.counter = self.counter + 1
        x = cur_pose.pose.position.x
        y = cur_pose.pose.position.y
        z = cur_pose.pose.position.z + 0.01
        ox = cur_pose.pose.orientation.x
        oy = cur_pose.pose.orientation.y
        oz = cur_pose.pose.orientation.z
        ow = cur_pose.pose.orientation.w
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        kernel_smooth = np.ones((5,5),np.float32)/25
        gray_img = cv2.filter2D(gray_img, -1, kernel_smooth)
    
        bw_img = cv2.adaptiveThreshold(gray_img,255,\
                                   cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                                   cv2.THRESH_BINARY,103,1)
                                   
                                   
        bw_img1 = cv2.bitwise_not(bw_img)
    
        #bw_img2 = cv2.bitwise_and(bw_img1,bw_img1,mask = mask_img)#deepcopy(bw_img1)
            
        
        
        
        
        cv2.imwrite(image_filename, bw_img1)
        
        
        file.write("%s %.4f %.4f %.4f %.4f %4f %.4f %.4f\n" % (image_filename, x, y, z, ox, oy, oz, ow))
        file.close()
    
    def init_pose(self):
        
        cur_pose = self.current_poses[self.arm]
        x = cur_pose.pose.position.x
        y = cur_pose.pose.position.y
        z = cur_pose.pose.position.z
        ox = cur_pose.pose.orientation.x
        oy = cur_pose.pose.orientation.y
        oz = cur_pose.pose.orientation.z
        ow = cur_pose.pose.orientation.w
        pose_list = [x, y, z, -1.0, 1.0, 0.0, 0.0]
        self.move_arm('left', pose_list)  
    
    def up_recording(self, image_folder):
        
        img_folder = image_folder
        while not rospy.is_shutdown():
            img_filename = image_folder + 'image_'+str(self.counter).zfill(4)+'.png'
            self.capture_image(img_filename)
            
            rospy.sleep(1)
    
    
    def turn_recording(self, image_folder):
        
        for i in range(0, 360):
            img_filename = image_folder + 'image_'+str(self.counter).zfill(4)+'.png'
            #self.capture_image(img_filename)
            cur_pose = self.current_poses[self.arm]
            self.counter = self.counter + 1
            x = cur_pose.pose.position.x
            y = cur_pose.pose.position.y
            z = cur_pose.pose.position.z
            ox = cur_pose.pose.orientation.x + 1/360
            oy = cur_pose.pose.orientation.y
            oz = cur_pose.pose.orientation.z
            ow = cur_pose.pose.orientation.w
            self.move_arm('left', [x, y, z, ox, oy, oz, ow])
            rospy.sleep(1)
            
    def run(self):
        
        self.init_msgs()
        rospy.sleep(1)
        #self.init_pose()
        img_folder = './src/phm/images/'
        self.up_recording(img_folder)
        while not rospy.is_shutdown():
            
            
            
            rospy.sleep(0.1)
        
        
        
def signal_handler(signal, frame):
    print 'You pressed Ctrl+C!'
    sys.exit(0)
    
    
def main():

    signal.signal(signal.SIGINT, signal_handler)
    counter = int(sys.argv[1])
    ic = ImageCollecting('left', counter)
    ic.run()

if __name__ == '__main__':
    
    try:
        main()
    except KeyboardInterrupt:
        print "\nCtrl-C Pressed\n"
        flag = True
        sys.exit()
        
        
        

