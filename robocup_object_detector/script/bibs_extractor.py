#!/usr/bin/env python
import sys
import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from robocup_msgs.msg import RobocupObject,Robot,SoccerBall
import rospkg
import dlib
import rospy
from dynamic_reconfigure.server import Server
from robocup_object_detector.cfg import Params_object_detectorConfig
from intersect_plane import Intersector
import csv_reader
import os
import tf

class rect:
    def __init__(self,x,y,w,h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

class bibs:
    def __init__(self,name,h_th_low,h_th_up,min_bibs_area,is_friend):
        self.intersector = Intersector()
        self.camera_frame = rospy.get_param(rospy.get_name() + "/camera_frame")
        self.view_angle_width = 0
        self.view_angle_height = 0
        self.bibs_height = 0
        self.height_robot = 0
        self.width_robot = 0
        self.h_th_low = h_th_low
        self.h_th_up = h_th_up
        self.min_bibs_area = min_bibs_area
        self.name = name
        self.rects = []
        self.r = 0
        self.g = 0
        self.b = 0
        self.max_robot_detection_range = 0
        self.min_robot_detection_range = 0
        self.bibs_min_aspect_ratio = 0
        self.is_friend = is_friend
        if is_friend == True:
            self.marker_pub = rospy.Publisher(rospy.get_name() + "/friend/marker",Marker,queue_size=10)
        else:
            self.marker_pub = rospy.Publisher(rospy.get_name() + "/enemy/marker",Marker,queue_size=10)
        self.robot_marker = Marker()
        self.robot_marker.action = self.robot_marker.ADD
        self.robot_marker.id = 0
        self.robot_marker.type = 3
        self.robot_marker.lifetime = rospy.Duration(1.0)
        self.robot_marker.color.r = 0
        self.robot_marker.color.g = 1
        self.robot_marker.color.b = 0
        self.robot_marker.color.a = 1
        q = tf.transformations.quaternion_from_euler(0,0,0)
        self.robot_marker.pose.orientation.x = q[0]
        self.robot_marker.pose.orientation.y = q[1]
        self.robot_marker.pose.orientation.z = q[2]
        self.robot_marker.pose.orientation.w = q[3]

    def remove_small_area(self,img):
        img_tmp = np.copy(img)
        mask = np.copy(img)
        mask[:,:] = 0
        contours, hierarchy = cv2.findContours(img_tmp,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        contours = list(filter(lambda c:cv2.contourArea(c) > self.min_bibs_area, contours))
        cv2.drawContours(mask,contours,-1,(255),-1)
        return mask

    def extract(self,img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        if self.h_th_up >= self.h_th_low:
            ret,h_low = cv2.threshold(h,self.h_th_low/2,255,cv2.THRESH_BINARY)
            ret,h_top = cv2.threshold(h,self.h_th_up/2,255,cv2.THRESH_BINARY_INV)
            mask = cv2.bitwise_and(h_low,h_top)
            mask = self.remove_small_area(mask)
            return mask
        else:
            ret,h_low = cv2.threshold(h,self.h_th_low/2,255,cv2.THRESH_BINARY_INV)
            ret,h_top = cv2.threshold(h,self.h_th_up/2,255,cv2.THRESH_BINARY)
            mask = cv2.bitwise_and(h_low,h_top)
            mask = cv2.bitwise_not(mask)
            mask = self.remove_small_area(mask)
            return mask

    def get_bibs_area(self,mask):
        img = np.copy(mask)
        contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        self.rects = []
        for contour in contours:
            x,y,w,h = cv2.boundingRect(contour)
            if h >= w*self.bibs_min_aspect_ratio:
                self.rects.append(rect(x,y,w,h))

    def set_color(self,r,g,b):
        self.r = r
        self.g = g
        self.b = b
        self.robot_marker.color.r = r/255.0
        self.robot_marker.color.g = g/255.0
        self.robot_marker.color.b = b/255.0

    def set_param(self,config):
        self.view_angle_width = config.view_angle_width
        self.view_angle_height = config.view_angle_height
        self.bibs_height = config.bibs_height
        self.height_robot = config.height_robot
        self.width_robot = config.width_robot
        self.max_robot_detection_range = config.max_robot_detection_range
        self.min_robot_detection_range = config.min_robot_detection_range
        self.bibs_min_aspect_ratio = config.bibs_min_aspect_ratio

    def draw(self,img):
        num_rects = len(self.rects)
        for i in range(num_rects):
            x = self.rects[i].x
            y = self.rects[i].y
            w = self.rects[i].w
            h = self.rects[i].h
            cv2.rectangle(img,(int(x),int(y)),(int(x+w),int(y+h)),(self.b,self.g,self.r),2)
            if self.is_friend == True:
                cv2.putText(img, "friend", (int(x)-30,int(y)-4), cv2.FONT_ITALIC, 1, (self.b,self.g,self.r), 1, cv2.CV_AA)
            if self.is_friend == False:
                cv2.putText(img, "enemy", (int(x)-30,int(y)-4), cv2.FONT_ITALIC, 1, (self.b,self.g,self.r), 1, cv2.CV_AA)

    def draw_3d(self,img):
        robot_msg = []
        robot = Robot()
        if len(img.shape) == 3:
            height, width, channels = img.shape[:3]
        else:
            height, width = img.shape[:2]
            channels = 1
        num_rects = len(self.rects)
        for i in range(num_rects):
            d_x = 1.0*self.rects[i].x + 0.5 * self.rects[i].w
            d_y = 1.0*self.rects[i].y + 0.5 * self.rects[i].h
            x = (d_x - width/2)/width*2
            y = (d_y - height/2)/height*2

            yaw = np.deg2rad(x * self.view_angle_width)# + np.pi
            pitch = np.deg2rad(y * self.view_angle_height) + np.pi
            q = tf.transformations.quaternion_from_euler(yaw, pitch, 0)
            p = PoseStamped()
            p.header.stamp = rospy.Time(0)
            p.header.frame_id = self.camera_frame
            p.pose.position.x = 0
            p.pose.position.y = 0
            p.pose.position.z = 0
            p.pose.orientation.w = q[0]
            p.pose.orientation.x = q[1]
            p.pose.orientation.y = q[2]
            p.pose.orientation.z = q[3]
            g = self.intersector.run(p,1)
            if g is not None:
                self.robot_marker.header.frame_id = "base_footprint"
                self.robot_marker.header.stamp = rospy.Time(0)
                yaw = np.arctan(g.point.y/g.point.x)
                theta = np.deg2rad((1.0*self.rects[i].h/height) * self.view_angle_height)
                robot_range = self.bibs_height/(np.tan(theta))
                if self.max_robot_detection_range > robot_range and self.min_robot_detection_range < robot_range:
                    self.robot_marker.pose.position.x = robot_range * np.cos(yaw)
                    self.robot_marker.pose.position.y = robot_range * np.sin(yaw)
                    self.robot_marker.pose.position.z = self.height_robot/2
                    self.robot_marker.scale.x = self.width_robot
                    self.robot_marker.scale.y = self.width_robot
                    self.robot_marker.scale.z = self.height_robot
                    self.marker_pub.publish(self.robot_marker)
                    robot.point.point = self.robot_marker.pose.position
                    robot.point.header = self.robot_marker.header
                    robot.point.point.z = 0
                    if self.is_friend == True:
                        robot.role = robot.FRIEND
                    else:
                        robot.role = robot.ENEMY
                    robot.radius = self.width_robot
                    robot.height = self.height_robot
                    robot.detection_type = robot.DETECTED
                    robot_msg.append(robot)
            return robot_msg

class bibs_extractor:
    def __init__(self):
        self.bridge = CvBridge()
        reader = csv_reader.csv_reader()
        reader.read()
        self.bibs_friend = reader.bibs_friend
        self.bibs_friend.set_color(0,0,255)
        self.bibs_enemy = reader.bibs_enemy
        self.bibs_enemy.set_color(255,0,0)
        #self.bibs_friend_pub = rospy.Publisher(rospy.get_name() + "/bibs/friend/image",Image,queue_size=10)
        #self.bibs_enemy_pub = rospy.Publisher(rospy.get_name() + "/bibs/enemy/image",Image,queue_size=10)

    def extract(self,img):
        src = np.copy(img)
        img = self.bibs_friend.extract(src)
        self.bibs_friend.get_bibs_area(img)
        #self.bibs_friend_pub.publish(self.bridge.cv2_to_imgmsg(img, "mono8"))
        img = self.bibs_enemy.extract(src)
        self.bibs_enemy.get_bibs_area(img)
        #self.bibs_enemy_pub.publish(self.bridge.cv2_to_imgmsg(img, "mono8"))


    def set_param(self,config):
        self.bibs_friend.set_param(config)
        self.bibs_enemy.set_param(config)
