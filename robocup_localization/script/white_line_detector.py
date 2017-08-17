#!/usr/bin/env python
import sys
import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from robocup_msgs.msg import WhiteLines,Circle,StraightLine
import tf2_ros
import tf
import math

class line_detector:
  def __init__(self):
      self.image_pub = rospy.Publisher(rospy.get_name()+"/detected_image",Image, queue_size=1)
      self.white_lines_pub = rospy.Publisher(rospy.get_name()+"/white_lines",WhiteLines, queue_size=1)
      self.bridge = CvBridge()
      self.tf_buffer = tf2_ros.Buffer()
      self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
      top_view_topic = rospy.get_param(rospy.get_name()+'/top_view_topic')
      self.camera_frame_name = rospy.get_param(rospy.get_name()+'/camera_frame_name')
      self.h_max = rospy.get_param(rospy.get_name()+'/h_max',180)
      self.s_max = rospy.get_param(rospy.get_name()+'/s_max',50)
      self.v_min = rospy.get_param(rospy.get_name()+'/v_min',150)
      self.expansion_rate = rospy.get_param(rospy.get_name()+'/expansion_rate',0.004)
      self.kernel_size = rospy.get_param(rospy.get_name()+'/kernel_size',25)
      self.hough_circle_param_1 = rospy.get_param(rospy.get_name()+'/hough_circle_param_1',50)
      self.hough_circle_param_2 = rospy.get_param(rospy.get_name()+'/hough_circle_param_2',12)
      self.hough_circle_min_radius = rospy.get_param(rospy.get_name()+'/hough_circle_min_radius',40)
      self.hough_circle_max_radius = rospy.get_param(rospy.get_name()+'/hough_circle_max_radius',0)
      self.hough_line_min_length = rospy.get_param(rospy.get_name()+'/hough_line_min_length',100)
      self.hough_line_max_gap = rospy.get_param(rospy.get_name()+"/max_line_gap",10)
      self.hough_line_th = rospy.get_param(rospy.get_name()+"/hough_line_th",30)
      self.hough_circle_th = rospy.get_param(rospy.get_name()+"/hough_circle_th",30)
      self.image_sub = rospy.Subscriber(top_view_topic,Image,self.callback)
  def callback(self,data):
      try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
      except CvBridgeError as e:
          print(e)
      white_min = np.array([0, 0, self.v_min],np.uint8)
      white_max = np.array([self.h_max, self.s_max, 255],np.uint8)
      hsv_img = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
      cv_image = cv2.inRange(hsv_img, white_min, white_max)
      if self.kernel_size != 0:
          kernel = np.ones((self.kernel_size,self.kernel_size),np.uint8)
          cv_image = cv2.erode(cv_image,kernel,iterations = 1)
      circles = cv2.HoughCircles(cv_image,cv2.cv.CV_HOUGH_GRADIENT,1,self.hough_circle_th,param1=self.hough_circle_param_1,param2=self.hough_circle_param_2,minRadius=self.hough_circle_min_radius,maxRadius=self.hough_circle_max_radius)
      lines = cv2.HoughLinesP(cv_image,1,np.pi/180,self.hough_line_th,minLineLength=self.hough_line_min_length,maxLineGap=self.hough_line_max_gap)
      detected_image = cv2.cvtColor(cv_image,cv2.COLOR_GRAY2BGR)
      now = rospy.Time.now()
      try:
          trans = self.tf_buffer.lookup_transform('base_footprint', self.camera_frame_name, rospy.Time())
          self.euler = tf.transformations.euler_from_quaternion((trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w))
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
          pass
      height, width = cv_image.shape[:2]
      self.white_lines_msg = WhiteLines()
      self.white_lines_msg.header.stamp = now
      self.white_lines_msg.header.frame_id = "base_footprint"
      if circles is not None:
          circles = np.uint16(np.around(circles))
          for i in circles[0,:]:
              circle_msg = Circle()
              cv2.circle(detected_image,(i[0],i[1]),i[2],(0,0,255),2)
              theta = -math.atan(float(i[0]-width/2)/float(height-i[1])) - self.euler[2]
              r = math.sqrt(float(i[0]-width/2)*float(i[0]-width/2)+float(height-i[1])*float(height-i[1])) * self.expansion_rate
              circle_msg.center.x = r * math.cos(theta)
              circle_msg.center.y = r * math.sin(theta)
              circle_msg.center.z = 0
              circle_msg.radius = i[2] * self.expansion_rate
              self.white_lines_msg.circles.append(circle_msg)
      if lines is not None:
          for x1,y1,x2,y2 in lines[0]:
              cv2.line(detected_image,(x1,y1),(x2,y2),(255,0,0),2)
              line_msg = StraightLine()
              theta = -math.atan(float(x1-width/2)/float(height-y1)) - self.euler[2]
              r = math.sqrt(float(x1-width/2)*float(x1-width/2)+float(height-y1)*float(height-y1)) * self.expansion_rate
              line_msg.start_point.x = r * math.cos(theta)
              line_msg.start_point.y = r * math.sin(theta)
              line_msg.start_point.z = 0
              theta = -math.atan(float(x2-width/2)/float(height-y2)) - self.euler[2]
              r = math.sqrt(float(x2-width/2)*float(x2-width/2)+float(height-y2)*float(height-y2)) * self.expansion_rate
              line_msg.end_point.x = r * math.cos(theta)
              line_msg.end_point.y = r * math.sin(theta)
              line_msg.end_point.z = 0
              self.white_lines_msg.lines.append(line_msg)
      try:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(detected_image, "bgr8"))
      except CvBridgeError as e:
          print(e)
      self.white_lines_pub.publish(self.white_lines_msg)

if __name__ == '__main__':
    rospy.init_node('white_line_detector', anonymous=True)
    detector = line_detector()
    rospy.spin()
