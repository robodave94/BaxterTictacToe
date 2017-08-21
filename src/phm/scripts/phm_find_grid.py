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

class GridDetector(object):
    
    def __init__(self, arm):
        
        self.arm = arm
        self.current_task_dropping = True
        self.current_poses = None
        self.current_vision_cmd = ''
        self.rb_cmd_pub = rospy.Publisher('vision_reply', String, queue_size=10)
        self.template_imgs = {}
        #self.tmpl_width, self.tmpl_height, self.tmpl_channel = tmpl_bw_img.shape
        self.OcvBridge = CvBridge()
        #if self.tmpl_channel==3:
        #    self.tmpl_bw_img, t1, t2 = cv2.split(tmpl_bw_img)
        #elif self.tmpl_channel==1:
        #    self.tmpl_bw_img = deepcopy(tmpl_bw_img)
        #else:
        #    self.tmpl_bw_img = None
        self.GridRois = []
        self.GridRoiLocations = []
        self.GripperRect_x = [398, 662, 664, 398]
        self.GripperRect_y = [58, 72, 229, 219]
        self.Gripper_x = int(max(self.GripperRect_x)/2 + min(self.GripperRect_x)/2)
        self.Gripper_y = int(max(self.GripperRect_y)/2 + min(self.GripperRect_y)/2)
        self.RoiMaskImages = []
        
        rospy.Subscriber('robot_vision_cmd', String, self.vision_cmd_callback)
        
        
        
        
        
        left_camera_sub = rospy.Subscriber('/cameras/'+ 'left_hand_camera/image', \
                                       Image, self.left_camera_callback)
                                    
        right_camera_sub = rospy.Subscriber('/cameras/right_hand_camera/image', \
                                       Image, self.right_camera_callback)
                                    
        self.height = 600
        self.width = 960
        self.left_camera = baxter_interface.CameraController('left_hand_camera')
        
        self.right_camera = baxter_interface.CameraController('right_hand_camera')
        
        
        self.left_camera.open()
        self.left_camera.resolution = [self.width, self.height]
        self.left_camera.gain = 20
        self.right_camera.open()
        self.right_camera.resolution = [self.width, self.height]
        self.right_camera.gain = 20
        
        self.cam_calib    = 0.0025                     # meters per pixel at 1 meter
        self.cam_x_offset = 0.025 #Phm baxter left arm value                  # camera gripper offset
        self.cam_y_offset = -0.02 #Phm baxter left arm value
        
        
        self.cur_left_img = None
        self.cur_right_img = None                            
        self.LeftImageThreadLock = threading.Lock()
        self.RightImageThreadLock = threading.Lock()
                
        self.image_names = {'grid':'template_grid_white_center.png', \
                            'o':'template_o01.png', \
                            'x':'template_x05.png'}
                            
        self.image_folder = './src/phm/images/'
    
    def load_roi_location(self):
        
        self.GridRoiLocations = []
        
        file = open("./src/phm/grids_roi_location.txt", "r")
        for i in range(0, 10):
            line = file.readline()
            print "Read Line: ", line
            temp_list = []
            for j in line.split():
                
                temp_list.append(int(j))
                        
            p1 = [temp_list[0], temp_list[1]]
            p2 = [temp_list[2], temp_list[3]]
            p3 = [temp_list[4], temp_list[5]]
            p4 = [temp_list[6], temp_list[7]]
            self.GridRoiLocations.append([p1, p2, p3, p4])    
            
        print "Rois: ", self.GridRoiLocations
        print "Total Rois: ", len(self.GridRoiLocations)
        file.close()
    
    def create_roi_masks(self):
        
        self.RoiMaskImages = []
        
        for item_line in self.GridRoiLocations:
            temp_list = []
            for item in item_line:
                new_item = [item]
                temp_list.append(new_item)
            shape = np.array(item_line, np.int32)
            temp_img = np.zeros((self.height, self.width, 3), np.uint8) 
            #print shape
            cv2.fillPoly(temp_img, [shape], (255, 255, 255))
            mask_img, c2, c3 = cv2.split(temp_img)
            self.RoiMaskImages.append(mask_img)
            #cv2.imshow('current_image', temp_img)
            #cv2.waitKey(0)
        
        
    def vision_cmd_callback(self, msg):
        self.current_vision_cmd = msg.data
        self.current_task_dropping = False
        
    def add_template_images(self):
        types = self.image_names.keys()
        for type in types:
            filename = self.image_folder + self.image_names[type]
            print "add template image " + filename
            tmpl_img = cv2.imread(filename)
            tmpl_width, tmpl_height, tmpl_channel = tmpl_img.shape
            
            tmpl_bw_img = None
            if tmpl_channel==3:
                tmpl_bw_img, t1, t2 = cv2.split(tmpl_img)
            elif tmpl_channel==1:
                tmpl_bw_img = deepcopy(tmpl_img)
            else:
                tmpl_bw_img = None
                return
            
            self.template_imgs.update({type:tmpl_bw_img})
        
        return
        
    def interpret_vision_cmd(self, msg_string):
        
        msg_segments = msg_string.split(':')
        if len(msg_segments) != 3:
            return '', '', ''
        
        side = msg_segments[0]
        action = msg_segments[1]
        target = msg_segments[2]
        
        return side, action, target
        
    def init_msgs(self):
        
        left_ir_msg = message_filters.Subscriber('/robot/range/left_hand_range/state',Range)
        right_ir_msg = message_filters.Subscriber('/robot/range/right_hand_range/state',Range)
        ts = message_filters.ApproximateTimeSynchronizer([left_ir_msg, right_ir_msg], 10, 0.05)
        ts.registerCallback(self._ir_sensor_callback)
        
        self.current_ir_ranges = {'left':65.0, 'right':65.0}
        
        left_arm_msg = message_filters.Subscriber("/robot/limb/left/endpoint_state",EndpointState)
        right_arm_msg = message_filters.Subscriber("/robot/limb/right/endpoint_state",EndpointState)
        ts = message_filters.ApproximateTimeSynchronizer([left_arm_msg, right_arm_msg], 10, 0.05)
        ts.registerCallback(self._pose_callback)
    
    def get_ir_range(self, side):
        range = self.current_ir_ranges[side]
        return range
    
    def _pose_callback(self, left_msg, right_msg):
    
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
        
        
        
        #print "\nleft: \n", cp1, "\n\nnew left: \n", cp11
            
        return
    
    def _ir_sensor_callback(self, left_msg, right_msg): #def _ir_sensor_callback(self, msg, side):
        
        #print "\nLeft IR: \n", left_msg.range
        self.current_ir_ranges={'left':left_msg.range, 'right':right_msg.range}
        return
    
    def left_camera_callback(self, image):
        with self.LeftImageThreadLock:
            try:
                self.cur_left_img = self.OcvBridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
                #cv_img = deepcopy(self.cur_img)
            except Exception:
                
                print 'OH NO - IMAGE WENT WRONG!!'
                
    def right_camera_callback(self, image):
        with self.LeftImageThreadLock:
            try:
                self.cur_right_img = self.OcvBridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
                #cv_img = deepcopy(self.cur_img)
            except Exception:
                
                print 'OH NO - IMAGE WENT WRONG!!'
        
    def get_image(self, side):
        c_img = None
        if side=='left':
            with self.LeftImageThreadLock:
                c_img = deepcopy(self.cur_left_img)
        elif side == 'right':
            with self.RightImageThreadLock:
                c_img = deepcopy(self.cur_right_img)
        
        return c_img
        
       
    def contour_matching1(self, img, tmpl_bw_img, mask_img):
    
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        kernel_smooth = np.ones((5,5),np.float32)/25
        gray_img = cv2.filter2D(gray_img, -1, kernel_smooth)
        
        bw_img = cv2.adaptiveThreshold(gray_img,255,\
                                       cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                                       cv2.THRESH_BINARY,103,1)
                                       
                                       
        bw_img1 = cv2.bitwise_not(bw_img)
        
        bw_img2 = cv2.bitwise_and(bw_img1,bw_img1,mask = mask_img)#deepcopy(bw_img1)
        
        contours, hierarchy = cv2.findContours(bw_img2, cv2.RETR_TREE, \
                                               cv2.CHAIN_APPROX_SIMPLE)
        counter = 0
        
        tmpl_contours, tmpl_hierarchy = cv2.findContours \
                                                (tmpl_bw_img, \
                                                 cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) 
        
        #print "\n...Start with a new image...\n"
        matching_result = []
        for cnt in contours:

            #contour_area = cv2.contourArea(cnt)
            mom = cv2.moments(cnt)
            contour_area = mom['m00']
            #print "\nContourArea: ", contour_area, "Moments: ", contour_area1
            #if contour_area<100:
            #    matching_result.append(10.0) # a large number to fill the list
            #    continue
            empty_img = np.zeros((self.height,self.width,1), np.uint8)
            contour_img = cv2.merge((bw_img1, empty_img, empty_img)) #np.zeros((self.height,self.width,3), np.uint8)
            cv2.drawContours(contour_img, contours, counter, (255,255,255), 2)
            ret = cv2.matchShapes(cnt,tmpl_contours[0], cv2.cv.CV_CONTOURS_MATCH_I1, 0.0)
            #print ret, "\nContour Area: \n", contour_area
            if ret == 0.0 or contour_area<300:
                ret = 10.0
            matching_result.append(ret)
            #plot_img = np.zeros((self.height,self.width,3), np.uint8)
            
            counter = counter + 1
            #cv2.imshow('current_image', contour_img)
            #cv2.waitKey(0)
            #rospy.sleep(0.1)
        if matching_result != []:
            min_index = matching_result.index(min(matching_result))
            rect = cv2.minAreaRect(contours[min_index])
            angle = rect[2]
            cx = math.floor(rect[0][0])
            cy = math.floor(rect[0][1])
            print "\nMatching Result: (", matching_result[min_index], ")\nangle: ", angle
            
        
        
            empty_img = np.zeros((self.height,self.width,1), np.uint8)
            contour_img = cv2.merge((bw_img1, empty_img, empty_img))
            plot_img = cv2.merge((bw_img1,bw_img1,bw_img1)) #deepcopy(img)
            cv2.drawContours(plot_img, contours, min_index, (255,0,0), 2)
            #cv2.imshow('current_image', plot_img) #plot_img)
            #cv2.waitKey(0)
            return matching_result[min_index]
        
        return 10
        
    def contour_matching2(self, img, tmpl_bw_img, mask_img):
        
        
    
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        kernel_smooth = np.ones((5,5),np.float32)/25
        gray_img = cv2.filter2D(gray_img, -1, kernel_smooth)
        
        bw_img = cv2.adaptiveThreshold(gray_img,255,\
                                       cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                                       cv2.THRESH_BINARY,103,1)
                                       
                                       
        bw_img1 = cv2.bitwise_not(bw_img)
        
        bw_img2 = cv2.bitwise_and(bw_img1,bw_img1,mask = mask_img)#deepcopy(bw_img1)
        
        contours, hierarchy = cv2.findContours(bw_img2, cv2.RETR_TREE, \
                                               cv2.CHAIN_APPROX_SIMPLE)
        counter = 0
        
        tmpl_contours, tmpl_hierarchy = cv2.findContours \
                                                (tmpl_bw_img, \
                                                 cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) 
        tmpl_rect = cv2.minAreaRect(tmpl_contours[0])
        
        #print "\n...Start with a new image...\n"
        matching_result = []
        for cnt in contours:

            #contour_area = cv2.contourArea(cnt)
            mom = cv2.moments(cnt)
            contour_area = mom['m00']
            #print "\nContourArea: ", contour_area, "Moments: ", contour_area1
            #if contour_area<100:
            #    matching_result.append(10.0) # a large number to fill the list
            #    continue
            empty_img = np.zeros((self.height,self.width,1), np.uint8)
            contour_img = cv2.merge((bw_img1, empty_img, empty_img)) #np.zeros((self.height,self.width,3), np.uint8)
            cv2.drawContours(contour_img, contours, counter, (255,255,255), 2)
            ret = cv2.matchShapes(cnt,tmpl_contours[0], cv2.cv.CV_CONTOURS_MATCH_I1, 0.0)
            #print ret, "\nContour Area: \n", contour_area
            
            if ret == 0.0 or contour_area<300:
                ret = 10.0
            matching_result.append(ret)
            #plot_img = np.zeros((self.height,self.width,3), np.uint8)
            
            counter = counter + 1
            #cv2.imshow('current_image', contour_img)
            #cv2.waitKey(0)
            #rospy.sleep(0.1)
        if matching_result != []:
            min_index = matching_result.index(min(matching_result))
            rect = cv2.minAreaRect(contours[min_index])
            tmpl_rect_area = tmpl_rect[1][0]*tmpl_rect[1][1]
            obj_rect_area = rect[1][0]*rect[1][1]
            
            angle = rect[2]
            cx = math.floor(rect[0][0])
            cy = math.floor(rect[0][1])
            print "\nMatching Result: (", matching_result[min_index], ")\nangle: ", angle
            
        
        
            empty_img = np.zeros((self.height,self.width,1), np.uint8)
            contour_img = cv2.merge((bw_img1, empty_img, empty_img))
            plot_img = cv2.merge((bw_img1,bw_img1,bw_img1)) #deepcopy(img)
            #cv2.drawContours(plot_img, contours, min_index, (255,0,0), 2)
            #cv2.imshow('current_image', plot_img) #plot_img)
            #cv2.imwrite('current_img.png', img)
            #cv2.waitKey(0)
            
            if (obj_rect_area/tmpl_rect_area)>1.2:
                print "Object too big..."
                return 10, rect
            return matching_result[min_index], rect
        
        return 10, []

    
    def contour_match(self, img, tmpl_bw_img, mask_img, rect):
        
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        kernel_smooth = np.ones((5,5),np.float32)/25
        gray_img = cv2.filter2D(gray_img, -1, kernel_smooth)
        
        bw_img = cv2.adaptiveThreshold(gray_img,255,\
                                       cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                                       cv2.THRESH_BINARY,103,1)
                                    
        bw_img1 = cv2.bitwise_not(bw_img, mask = mask_img)
        #cv2.imshow('current_image', bw_img1)
        #cv2.waitKey(10)
        bw_img2 = deepcopy(bw_img1)
        contours, hierarchy = cv2.findContours(bw_img2, cv2.RETR_TREE, \
                                               cv2.CHAIN_APPROX_SIMPLE)
        
        tmpl_contours, tmpl_hierarchy = cv2.findContours \
                                                (tmpl_bw_img, \
                                                 cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)                                    
                                            
        counter = 0
        matching_result = []
        plot_img = cv2.merge((bw_img1, bw_img1, bw_img1))
        for cnt in contours:
            
            mom = cv2.moments(cnt)
            contour_area = mom['m00']
            
            ret = cv2.matchShapes(cnt,tmpl_contours[0], cv2.cv.CV_CONTOURS_MATCH_I1, 0.0)
            #print ret, "\nContour Area: \n", contour_area
            if ret == 0.0 or contour_area<300:
                ret = 10.0
            matching_result.append([ret,cnt])
            
            counter = counter + 1
        sorted_list = sorted(matching_result,key=lambda x: x[0])
        new_list = sorted_list[0:5]
        if new_list[0][0]>0.1:
            print "Can't find item", new_list[0][0]
            #cv2.imshow('current_image', plot_img)
            #cv2.waitKey(10)
            return None
        dist_list = []
        new_rect = None
        #plot_img = deepcopy(img)
        
        if rect == None:
            print "None Image"
            new_rect = cv2.minAreaRect(new_list[0][1])
            cv2.drawContours(plot_img, [new_list[0][1]], 0, (255,0,0), 2)
        
        else:
            print "Real Images"
            cx0 = rect[0][0]
            cy0 = rect[0][1]
            
            for item in new_list:
                
                cnt = item[1]
                new_rect1 = cv2.minAreaRect(cnt)
                cx1 = new_rect1[0][0]
                cy1 = new_rect1[0][1]
                dist = math.sqrt((cx0-cx1)*(cx0-cx1) + (cy0-cy1)*(cy0-cy1))
                dist_list.append([dist, new_rect1, cnt])
        
            
            #min_index = matching_result.index(min(matching_result))
            #print "size of dist list", len(dist_list)
            sorted_dist_list = sorted(dist_list,key=lambda x: x[0])
            new_rect = sorted_dist_list[0][1]
            cv2.drawContours(plot_img, [sorted_dist_list[0][2]], 0, (255,0,0), 2)
        
        #cv2.imshow('current_image', plot_img)
        #cv2.waitKey(10)
        #sorted_dist_list 
        #rect = cv2.minAreaRect(contours[min_index])
        #angle = rect[2]
        #cx = math.floor(rect[0][0])
        #cy = math.floor(rect[0][1])
        
        return new_rect
        
    #def get_new_rect(self, rect):
    
    
    
        
    def integer_box(self, box, width, height):
        
        
        x0 = math.floor(box[0][0])
        y0 = math.floor(box[0][1])
        
        x1 = math.floor(box[1][0])
        y1 = math.floor(box[1][1])
        
        x2 = math.floor(box[2][0])
        y2 = math.floor(box[2][1])
        
        x3 = math.floor(box[3][0])
        y3 = math.floor(box[3][1])
            
        int_box = np.array([ [x0, y0], [x1, y1], [x2, y2], [x3, y3] ], 'int32')
        
        for point in int_box:
            if point[0]>width:
                point[0] = width
                
            if point[1]>height:
                point[1] = height
                
            if point[0]<0:
                point[0] = 0
                
            if point[1]<0:
                point[1] = 0 
        
        
        return int_box
        
    def track_contour(self, side, object_type):
        img = self.get_image(side)
        if img == None:
            rospy.sleep(0.05)
            return
        
        height, width, channel = img.shape
        mask_img = np.ones((height,width,1), np.uint8)
        
        tmpl_bw_img = self.template_imgs[object_type]
        rect = self.contour_match(img, tmpl_bw_img, mask_img, None)
        #print rect
        counter = 0
        result_x = 0
        result_y = 0
        task_dropping = self.current_task_dropping
        print "Task Dropping Status: ", task_dropping
        while (not task_dropping) and (counter <30):
            
            img = self.get_image(side)
            if img == None:
                rospy.sleep(0.05)
                continue
            
            #new_h = rect[1][0]+rect[1][0]/2.0
            #new_w = rect[1][1]+rect[1][1]/2.0
            #center = (rect[0][0], rect[0][1])
            #size = (new_h, new_w)
            #new_rect = (center, size, rect[2])
            #box = cv2.cv.BoxPoints(new_rect)
            #int_box = self.integer_box(box, width, height)
            #print type(box), box
            #temp_img = np.zeros((height,width,3), np.uint8)
            #cv2.rectangle(temp_img, tuple(int_box[0]), tuple(int_box[2]), (255, 255, 255), -1, 8, 0)
            #mask_img, g, r = cv2.split(temp_img)
            #new_img = cv2.bitwise_and(img,img,mask = mask_img)
            
            rect1 = self.contour_match(img, tmpl_bw_img, mask_img, rect)
            rect = rect1
            print rect
            cur_pose = self.current_poses[side]
            x = cur_pose.pose.position.x
            y = cur_pose.pose.position.y
            z = cur_pose.pose.position.z
            ox = cur_pose.pose.orientation.x
            oy = cur_pose.pose.orientation.y
            oz = cur_pose.pose.orientation.z
            ow = cur_pose.pose.orientation.w
            table_z = -0.20-0.096 ## Hard coded 
            block_height = 0.03
            if rect != None:
                #x, y = self.pixel_to_baxter([rect[0][0], rect[0][1]], self.get_ir_range(side))
                x, y = self.pixel_to_baxter([rect[0][0], rect[0][1]], math.fabs(z-table_z)-block_height)
                print "Current Z: ", z
                print "Distance IR: ", self.get_ir_range(side), " Distance Z: ", math.fabs(z-table_z)-block_height
                result_x = result_x + x
                result_y = result_y + y
                counter = counter + 1
                print x, y
                #self.rb_cmd_pub.publish(msg_string)
            #cv2.imshow('current_image', new_img)
            #cv2.waitKey(10)
            rospy.sleep(0.05)
            
        
        self.current_task_dropping = True
        result_x = round(result_x/30, 4)
        result_y = round(result_y/30, 4)
        msg_string = side + ':' + \
                     object_type + \
                     ':' + str(result_x) + ',' + str(result_y) + \
                     ',' + str(round(self.get_ir_range(side), 4))
        self.rb_cmd_pub.publish(msg_string)
        return  result_x/30, result_y/30
    
    def track_contour1(self, side, object_type):
        img = self.get_image(side)
        if img == None:
            rospy.sleep(0.05)
            return
        
        height, width, channel = img.shape
        mask_img = np.ones((height,width,1), np.uint8)
        
        tmpl_bw_img = self.template_imgs[object_type]
        rect = self.contour_match(img, tmpl_bw_img, mask_img, None)
        #print rect
        counter = 0
        result_x = 0
        result_y = 0
        task_dropping = self.current_task_dropping
        counter = 0
        print "Task Dropping Status: ", task_dropping
        while (not task_dropping) and (not task_dropping):
            
            img = self.get_image(side)
            if img == None:
                rospy.sleep(0.05)
                continue
            
            #new_h = rect[1][0]+rect[1][0]/2.0
            #new_w = rect[1][1]+rect[1][1]/2.0
            #center = (rect[0][0], rect[0][1])
            #size = (new_h, new_w)
            #new_rect = (center, size, rect[2])
            #box = cv2.cv.BoxPoints(new_rect)
            #int_box = self.integer_box(box, width, height)
            #print type(box), box
            #temp_img = np.zeros((height,width,3), np.uint8)
            #cv2.rectangle(temp_img, tuple(int_box[0]), tuple(int_box[2]), (255, 255, 255), -1, 8, 0)
            #mask_img, g, r = cv2.split(temp_img)
            #new_img = cv2.bitwise_and(img,img,mask = mask_img)
            
            rect1 = self.contour_match(img, tmpl_bw_img, mask_img, rect)
            rect = rect1
            print rect
            cur_pose = self.current_poses[side]
            x = cur_pose.pose.position.x
            y = cur_pose.pose.position.y
            z = cur_pose.pose.position.z
            ox = cur_pose.pose.orientation.x
            oy = cur_pose.pose.orientation.y
            oz = cur_pose.pose.orientation.z
            ow = cur_pose.pose.orientation.w
            
            block_height = 0.03
            dx = 0
            dy = 0
            angle = 0.0
            if rect != None:
                #x, y = self.pixel_to_baxter([rect[0][0], rect[0][1]], self.get_ir_range(side))
                #x, y = self.pixel_to_baxter([rect[0][0], rect[0][1]], math.fabs(z-table_z)-block_height)
                dx = rect[0][0]-self.Gripper_x
                dy = rect[0][1]-self.Gripper_y
                angle = rect[2]
                counter = counter + 1
                print x, y, angle
                
            if counter>10:
                task_dropping = True
                #self.rb_cmd_pub.publish(msg_string)
            #cv2.imshow('current_image', new_img)
            #cv2.waitKey(10)
            rospy.sleep(0.05)
            
        
        self.current_task_dropping = True
        
        
        return  dx, dy, angle
    
    def track_contour2(self, side, object_type, mask):
        img = self.get_image(side)
        if img == None:
            rospy.sleep(0.05)
            return
        
        height, width, channel = img.shape
        mask_img = np.ones((height,width,1), np.uint8)
        
        tmpl_bw_img = self.template_imgs[object_type]
        rect = self.contour_match(img, tmpl_bw_img, mask_img, None)
        #print rect
        counter = 0
        result_x = 0
        result_y = 0
        result_angle = 0.0
        task_dropping = self.current_task_dropping
        print "Task Dropping Status: ", task_dropping
        while (not task_dropping) and (counter <5):
            
            img = self.get_image(side)
            if img == None:
                rospy.sleep(0.05)
                continue
            
            #new_h = rect[1][0]+rect[1][0]/2.0
            #new_w = rect[1][1]+rect[1][1]/2.0
            #center = (rect[0][0], rect[0][1])
            #size = (new_h, new_w)
            #new_rect = (center, size, rect[2])
            #box = cv2.cv.BoxPoints(new_rect)
            #int_box = self.integer_box(box, width, height)
            #print type(box), box
            #temp_img = np.zeros((height,width,3), np.uint8)
            #cv2.rectangle(temp_img, tuple(int_box[0]), tuple(int_box[2]), (255, 255, 255), -1, 8, 0)
            #mask_img, g, r = cv2.split(temp_img)
            #new_img = cv2.bitwise_and(img,img,mask = mask_img)
            
            rect1 = self.contour_match(img, tmpl_bw_img, mask_img, rect)
            rect = rect1
            print rect
            cur_pose = self.current_poses[side]
            x = cur_pose.pose.position.x
            y = cur_pose.pose.position.y
            z = cur_pose.pose.position.z
            ox = cur_pose.pose.orientation.x
            oy = cur_pose.pose.orientation.y
            oz = cur_pose.pose.orientation.z
            ow = cur_pose.pose.orientation.w
            table_z = -0.20-0.096 ## Hard coded 
            block_height = 0.05
            if rect != None:
                #x, y = self.pixel_to_baxter([rect[0][0], rect[0][1]], self.get_ir_range(side))
                x, y = self.pixel_to_baxter([rect[0][0], rect[0][1]], math.fabs(z-table_z)-block_height)
                print "Current Z: ", z
                print "Distance IR: ", self.get_ir_range(side), " Distance Z: ", math.fabs(z-table_z)-block_height
                result_x = result_x + x
                result_y = result_y + y
                result_angle = result_angle + rect[2]
                counter = counter + 1
                print x, y
                #self.rb_cmd_pub.publish(msg_string)
            #cv2.imshow('current_image', new_img)
            #cv2.waitKey(10)
            rospy.sleep(0.05)
            
        
        self.current_task_dropping = True
        result_x = round(result_x/5, 4)
        result_y = round(result_y/5, 4)
        result_angle = round(result_angle/5, 4)
        msg_string = side + ':' + \
                     object_type + \
                     ':' + str(result_x) + ',' + str(result_y) + \
                     ',' + str(result_angle) + \
                     ',' + str(round(self.get_ir_range(side), 4))
        self.rb_cmd_pub.publish(msg_string)
        return  result_x, result_y, result_angle


    def pixel_to_baxter(self, px, dist):
        
        x1 = self.current_poses[self.arm].pose.position.x
        y1 = self.current_poses[self.arm].pose.position.y
        print "\nCurrent Pose x, y:", x1, y1, "\n"
        print "Current Cx, Cy: ", px[0], px[1], "\n"
        print "Current Distance: ", dist, "\n"
        x = ((px[1] - (self.height / 2)) * self.cam_calib * dist) \
           + self.cam_x_offset 
        y = ((px[0] - (self.width / 2)) * self.cam_calib * dist) \
           + self.cam_y_offset 
        print "Current dx, dy: ", x, y 
        print "Target x, y: ", -x+x1, -y+y1
        #print "Target x, y: ", x+x1, y+y1
        return -x+x1, -y+y1
    
    def put_to_grid(self, x, y):
        pass
    
    
    def dt_matching(self, img):
        
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        kernel_smooth = np.ones((5,5),np.float32)/25
        gray_img = cv2.filter2D(gray_img, -1, kernel_smooth)
        
        bw_img = cv2.adaptiveThreshold(gray_img,255,\
                                       cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
                                       cv2.THRESH_BINARY,103,1)
                                       
                                       
        bw_img1 = cv2.bitwise_not(bw_img)
        
        bw_img2 = deepcopy(bw_img1)
        contours, hierarchy = cv2.findContours(bw_img2, cv2.RETR_TREE, \
                                               cv2.CHAIN_APPROX_SIMPLE)
        counter = 0
        
        print "\n...Start with a new image...\n"
        matching_result = []
        for cnt in contours:

            #contour_area = cv2.contourArea(cnt)
            mom = cv2.moments(cnt)
            contour_area = mom
            #if contour_area<100:
            #    matching_result.append(10.0) # a large number to fill the list
            #    continue
            empty_img = np.zeros((self.height,self.width,1), np.uint8)
            contour_img = cv2.merge((bw_img1, empty_img, empty_img)) #np.zeros((self.height,self.width,3), np.uint8)
            cv2.drawContours(contour_img, contours, counter, (255,255,255), 2)
            ret = cv2.matchShapes(cnt,self.tmpl_contours[0], cv2.cv.CV_CONTOURS_MATCH_I3, 0.0)
            #print ret, "\nContour Area: \n", contour_area
            if ret == 0.0:
                ret = 10.0
            matching_result.append(ret)
            #plot_img = np.zeros((self.height,self.width,3), np.uint8)
            
            counter = counter + 1
            #cv2.imshow('current_image', contour_img)
            #cv2.waitKey(0)
            #rospy.sleep(0.1)
    
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
        
    def move_to(self, x, y, z, ox, oy, oz, ow, arm):
        msg_string = arm + ':move_to:' + \
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
        self.rb_cmd_pub.publish(msg_string)
        
        return
    
    def move_arm(self, x, y, z, arm):
        
        msg_string = arm + ':move:' + \
                      str(x) + \
                      ',' + \
                      str(y) + \
                      ',' + \
                      str(z) + \
                      ',' + \
                      str(0.0) + \
                      ',' + \
                      str(0.0) + \
                      ',' + \
                      str(0.0) + \
                      ',' + \
                      str(0.0)
        self.rb_cmd_pub.publish(msg_string)
        
        return
    def init_arm_angle(self, arm):
        msg_string = arm + ':init_angle:0.0,0.0,0.0,0.0,1.0,0.0,0.0'
            
        self.rb_cmd_pub.publish(msg_string)
        
        return
        
    def cv_show_img(self, img, key_pressed):
        
        #cv2.imshow('current_image', img)
        #key = cv2.waitKey(10)
        while key != key_pressed:
            key = cv2.waitKey(10)
    
    def record_grid_rois(self):
        if self.GridRoiLocations == []:
            print "Grid Roi Location not Loaded..."
            return
        
        for i in range(0,9):
            pass
        
    
    def check_grid(self, side, ids):

        if ids == []:
            print "Error in check_grid: id is not correctr"
            return
        grid_status = []
        task_dropping = False
        
        roi_mask_images = []
        for id in ids:
            mask_img = self.RoiMaskImages[id]
            roi_mask_images.append(mask_img)
        
        while (not task_dropping):
            
            task_dropping = self.current_task_dropping
            img = self.get_image(side)
            if img == None:
                rospy.sleep(0.05)
                continue
            for mask_img in roi_mask_images:
                
                roi_img = cv2.bitwise_and(img,img,mask = mask_img)
                result_list = []
                object_names = ['x', 'o'] #self.template_imgs.keys()
                for object_name in object_names:
                    if object_name != 'grid':
                        tmpl_img = self.template_imgs[object_name]
                        result = self.contour_matching1(img, tmpl_img, mask_img)
                        result_list.append(result)
                
                min_index = result_list.index(min(result_list))
                if result_list[min_index]<0.1:
                    grid_status.append(object_names[min_index])
                    print "Find object: ", object_names[min_index]
##                    #empty_img = np.zeros((self.height,self.width,1), np.uint8)
##                    contour_img = cv2.merge((bw_img1, empty_img, empty_img))
##                    plot_img = cv2.merge((bw_img1,bw_img1,bw_img1)) #deepcopy(img)
##                    cv2.drawContours(plot_img, contours, min_index, (255,0,0), 2)
                else:
                    grid_status.append("b")
            
            task_dropping = True
            print "Grid Status", grid_status
            msg_string = side + ':grid_status:'
            for status in grid_status:
                msg_string = msg_string + status + ' '
            
            print "grid checking reply: ", msg_string
            self.rb_cmd_pub.publish(msg_string)
            
            #cv2.imshow('current_image', img)
            #cv2.waitKey(10)
            #rospy.sleep(0.05)
    
    def check_grid1(self, side, ids):

        if ids == []:
            print "Error in check_grid: id is not correctr"
            return
        grid_status = []
        grid_rect_list = []
        xy_list = []
        task_dropping = False
        
        roi_mask_images = []
        for id in ids:
            mask_img = self.RoiMaskImages[id]
            roi_mask_images.append(mask_img)
        
        while (not task_dropping):
            
            task_dropping = self.current_task_dropping
            img = self.get_image(side)
            if img == None:
                rospy.sleep(0.05)
                continue
            
            for mask_img in roi_mask_images:
                
                roi_img = cv2.bitwise_and(img,img,mask = mask_img)
                result_list = []
                rect_list = []
                object_names = ['x', 'o'] #self.template_imgs.keys()
                for object_name in object_names:
                    if object_name != 'grid':
                        tmpl_img = self.template_imgs[object_name]
                        result, rect = self.contour_matching2(img, tmpl_img, mask_img)
                        result_list.append(result)
                        rect_list.append(rect)
                cur_pose = self.current_poses[side]
                x = cur_pose.pose.position.x
                y = cur_pose.pose.position.y
                z = cur_pose.pose.position.z
                ox = cur_pose.pose.orientation.x
                oy = cur_pose.pose.orientation.y
                oz = cur_pose.pose.orientation.z
                ow = cur_pose.pose.orientation.w
                table_z = -0.079-0.096 ## Hard coded 
                
                block_height = 0.05
                
                msg_string = side + ':grid_status:'
                
                min_index = result_list.index(min(result_list))
                if result_list[min_index]<0.15:
                    grid_status.append(object_names[min_index])
                    rect = rect_list[min_index]
                    grid_rect_list.append(rect)
                    print "Find object: ", object_names[min_index]
                    
                    if rect != None or rect != []:
                        #x, y = self.pixel_to_baxter([rect[0][0], rect[0][1]], self.get_ir_range(side))
                        x1, y1 = self.pixel_to_baxter([rect[0][0], rect[0][1]], math.fabs(z-table_z)-block_height)
                        xy_list.append([x1, y1])
                        #print "Current Z: ", z
                        #print "Distance IR: ", self.get_ir_range(side), " Distance Z: ", math.fabs(z-table_z)-block_height
                        
                        #print x, y
                        
                    #empty_img = np.zeros((self.height,self.width,1), np.uint8)
##                    contour_img = cv2.merge((bw_img1, empty_img, empty_img))
##                    plot_img = cv2.merge((bw_img1,bw_img1,bw_img1)) #deepcopy(img)
##                    cv2.drawContours(plot_img, contours, min_index, (255,0,0), 2)
##                    cv2.imshow('current_image', img)
##                    cv2.waitKey(0)
                else:
                    
                    grid_status.append("b")
                    xy_list.append([10000.0, 10000.0])
                    
            
            task_dropping = True
            
            
            for status in grid_status:
                msg_string = msg_string + status + ' '
            
            msg_string = msg_string + ':'
            for xy in xy_list:
                
                msg_string = msg_string + str(xy[0]) + ',' + str(xy[1])
            
            
            print "grid checking reply: ", msg_string
            self.rb_cmd_pub.publish(msg_string)
            
            #cv2.imshow('current_image', img)
            #cv2.waitKey(10)
            #rospy.sleep(0.05)
    
            
    def fine_tune_location(self, side, object_type):
        
        img = self.get_image(side)
        
        if img == None:
            rospy.sleep(0.05)
            return
        
        dx, dy, angle = self.track_contour1(side, object_type)
        print "Fine Tune Ongoing: ", dx, dy, angle
        dx_string = str(dx)
        dy_string = str(dy)
        angle_string = str(round(angle, 4))
        
        msg_string = side + ':' + \
                     object_type + \
                     ':' + dx_string + ',' + dy_string + \
                     ',' + angle_string
        self.rb_cmd_pub.publish(msg_string)
        return  dx,  dy, angle
        

    def run(self):
        
        print "Loading Roi Locations..."
        self.load_roi_location()
        print "Create Roi Mask Images..."
        self.create_roi_masks()
        print "Vision Detecting and Tracking Starts"
        while not rospy.is_shutdown():
            
            c_cmd = self.current_vision_cmd
            if c_cmd == '':
                #self.current_vision_cmd = ''
                rospy.sleep(0.1)
                continue
            self.current_vision_cmd = ''
            side, action, target = self.interpret_vision_cmd(c_cmd)
            print "Cmd : ", c_cmd
            if target == 'grid':
                
                self.track_contour(side, 'grid')
                
            elif action == 'detect' and target == 'x':
                self.track_contour(side, 'x')
            
            elif action == 'detect' and target == 'o':
                self.track_contour(side, 'o')
            
            elif action == 'check':
                
                #ids_string = target.split(',')
                #ids = [int(i) for i in ids_string]
                id = int(target)
                print "Check Grid Id: ", id
                if id not in [0, 1, 2, 3, 4, 5, 6, 7, 8]:
                    print "Check Grid id not correct..."
                    rospy.sleep(0.1)
                    continue
                self.check_grid(side, [id])
                pass
            
            elif action == 'check1':
                
                #ids_string = target.split(',')
                #ids = [int(i) for i in ids_string]
                id = int(target)
                print "Check Grid Id: ", id
                if id not in [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]:
                    print "Check Grid id not correct..."
                    rospy.sleep(0.1)
                    continue
                self.check_grid1(side, [id])
                pass
            
            elif target == 'record':
                
                self.record_grid_rois()
            elif action=='fine_tune' and target == 'x':
                print "Fine tune position"
                self.fine_tune_location(side, 'x')
            elif action=='fine_tune' and target == 'o':
                print "Fine tune position"
                self.fine_tune_location(side, 'o')
            rospy.sleep(0.1)
            
    
        
flag = False
def main():
    
    #cv2.namedWindow('current_image')
    signal.signal(signal.SIGINT, signal_handler)
    #template_img = cv2.imread(sys.argv[1])
    arm = 'left'
    gd = GridDetector(arm)
    
    rospy.init_node("phm_find_grid")
    gd.init_msgs()
    
    gd.add_template_images()
    gd.run()
    #gd.init_arm_angle('left')
    #img = gd.get_image()
    #angle = 0 
    #cx = 0
    #cy = 0
    #if img != None:
    #    cx, cy, angle = gd.contour_matching(img)
    #x, y = gd.pixel_to_baxter([cx, cy], gd.get_ir_range('left'))
    #gd.move_arm(0.0, 0.3, 0.0, 'left')
    #rospy.sleep(2)
    #gd.move_arm(round(x, 2), round(y, 2), 0.0, 'right')
    #gd.move_to(-round(x, 2), -round(y, 2), 0.0, 0.0, 0.0, 0.0, 0.0, 'left')
    #rospy.sleep(5)
    
    '''
    while not rospy.is_shutdown() and not flag:
        
        #img = gd.get_image()
        #if img != None:
        #    cx, cy, angle = gd.contour_matching(img)
            #x, y = gd.pixel_to_baxter([cx, cy], gd.get_ir_range('left'))

            #gd.move_to(-round(x, 2), -round(y, 2), 0.0, 0.0, 0.0, 0.0, 0.0, left)

        gd.track_contour(arm, 'grid')
        
        rospy.sleep(0.1)
    '''

def signal_handler(signal, frame):
    print 'You pressed Ctrl+C!'
    sys.exit(0)


if __name__ == '__main__':
    
    try:
        main()
    except KeyboardInterrupt:
        print "\nCtrl-C Pressed\n"
        flag = True
        sys.exit()
        

    
    


