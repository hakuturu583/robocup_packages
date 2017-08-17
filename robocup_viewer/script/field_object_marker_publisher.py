#!/usr/bin/env python
import rospy
import numpy as np
import math
from jsk_recognition_msgs.msg import PolygonArray
from geometry_msgs.msg import PolygonStamped,Point32
from robocup_msgs.msg import WhiteLines,StraightLine,Circle

class field_object_publisher:
    def __init__(self):
        self.polygons_pub = rospy.Publisher(rospy.get_name()+"/polygons", PolygonArray, queue_size=1)
        self.white_lines_sub = rospy.Subscriber(rospy.get_name()+"/white_lines", WhiteLines, self.callback, queue_size=1)

    def circle_to_polygon_stamped(self,circle,header):
        polygon_stamped = PolygonStamped()
        polygon_stamped.header = header
        for i in range(360):
            point = Point32()
            point.x = circle.center.x + circle.radius * math.sin(np.pi*2*(float(i)/360.0))
            point.y = circle.center.y + circle.radius * math.cos(np.pi*2*(float(i)/360.0))
            point.z = 0
            polygon_stamped.polygon.points.append(point)
        return polygon_stamped

    def line_to_polygon_stamped(self,line,header):
        polygon_stamped = PolygonStamped()
        polygon_stamped.header = header
        for i in range(360):
            point = Point32()
            point.x = (line.end_point.x - line.start_point.x)/359.0*i + line.start_point.x
            point.y = (line.end_point.y - line.start_point.y)/359.0*i + line.start_point.y
            point.z = 0
            polygon_stamped.polygon.points.append(point)
        return polygon_stamped

    def callback(self,msg):
        polygon_array = PolygonArray()
        polygon_array.header = msg.header
        for circle in msg.circles:
            polygon_array.polygons.append(self.circle_to_polygon_stamped(circle,msg.header))
        for line in msg.lines:
            polygon_array.polygons.append(self.line_to_polygon_stamped(line,msg.header))
        self.polygons_pub.publish(polygon_array)

if __name__ == '__main__':
    rospy.init_node('field_object_publisher', anonymous=True)
    publisher = field_object_publisher()
    rospy.spin()
