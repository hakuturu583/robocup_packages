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
import bibs_extractor
import ball_detector

class ball_detector:
    def __init__(self):
        rospack = rospkg.RosPack()
        SvmFile = rospack.get_path("robocup_object_detector")+"/data/soccer_ball/detector.svm"
        self.ball_detection_range = 0
        self.camera_frame = rospy.get_param(rospy.get_name() + "/camera_frame")
        self.detector = dlib.simple_object_detector(SvmFile)
        self.diameter_soccer_ball = 0.15
        self.intersector = Intersector()
        self.view_angle_height = 0
        self.view_angle_width = 0
        self.marker_pub = rospy.Publisher(rospy.get_name() + "/soccer_ball/marker",Marker,queue_size=10)
        self.ball_marker = Marker()
        self.ball_marker.action = self.ball_marker.ADD
        self.ball_marker.id = 0
        self.ball_marker.type = self.ball_marker.SPHERE
        self.ball_marker.lifetime = rospy.Duration(1.0)
        self.ball_marker.color.r = 0
        self.ball_marker.color.g = 1
        self.ball_marker.color.b = 0
        self.ball_marker.color.a = 1
        self.ball_marker.pose.orientation.x = 0
        self.ball_marker.pose.orientation.y = 0
        self.ball_marker.pose.orientation.z = 0
        self.ball_marker.pose.orientation.w = 1

    def detect(self,img):
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        dets = self.detector(rgb)
        return dets

    def draw(self,dets,img):
        if len(dets):
            d = dets[0]
            cv2.rectangle(img, (d.left(), d.top()), (d.right(), d.bottom()), (0, 255, 0), 2)
            cv2.putText(img, "ball", (d.left()-5,d.top()-4), cv2.FONT_ITALIC, 1, (0, 255, 0), 1, cv2.CV_AA)

    def draw_3d(self,dets,img):
        if len(img.shape) == 3:
            height, width, channels = img.shape[:3]
        else:
            height, width = img.shape[:2]
            channels = 1
        if len(dets):
            d = dets[0]
            d_x = 1.0*(d.left() + d.right()/2)
            d_y = 1.0*(d.top() + d.bottom()/2)
            x = (d_x - width/2)/width
            y = (d_y - height/2)/height
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
            g = self.intersector.run(p,self.diameter_soccer_ball/2)
            ball_msg = []
            ball = SoccerBall()
            if g is not None:
                ball_range = np.sqrt(g.point.x * g.point.x + g.point.y * g.point.y)
                if ball_range < self.ball_detection_range:
                    self.ball_marker.header = g.header
                    self.ball_marker.header.stamp = rospy.Time(0)
                    self.ball_marker.pose.position = g.point
                    self.ball_marker.scale.x = self.diameter_soccer_ball
                    self.ball_marker.scale.y = self.diameter_soccer_ball
                    self.ball_marker.scale.z = self.diameter_soccer_ball
                    self.marker_pub.publish(self.ball_marker)
                    ball.point = g
                    ball.radius = self.diameter_soccer_ball/2
                    ball.detection_type = ball.DETECTED
                    ball_msg.append(ball)
            return ball_msg

    def set_param(self,config):
        self.diameter_soccer_ball = config.diameter_soccer_ball
        self.view_angle_width = config.view_angle_width
        self.view_angle_height = config.view_angle_height
        self.ball_detection_range = config.ball_detection_range
