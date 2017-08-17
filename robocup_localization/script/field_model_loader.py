#!/usr/bin/env python
import xml.etree.ElementTree as ET
import rospkg
import rospy
from robocup_msgs.msg import WhiteLines,Circle,StraightLine

class model_loader:
    def __init__(self):
        self.field_model_pub = rospy.Publisher(rospy.get_name()+"/field_model",WhiteLines,queue_size=1)
        rospack = rospkg.RosPack()
        tree = ET.parse(rospack.get_path('robocup_localization')+'/data/field_model.xml')
        root = tree.getroot()
        settings_info = root.find(".//settings")
        objects_info = root.find(".//objects")
        self.field_model_msg = WhiteLines()
        self.model_frame = ""
        try:
            self.model_frame = settings_info.find(".//frame_id").text
        except:
            rospy.logerr("failed to load frame_id")
        self.field_model_msg.header.frame_id = self.model_frame
        for object_info in objects_info:
            if object_info.attrib["type"] == "circle":
                circle_object = Circle()
                circle_object.radius = float(object_info.attrib["radius"])
                circle_object.center.x = float(object_info.attrib["center_x"])
                circle_object.center.y = float(object_info.attrib["center_y"])
                self.field_model_msg.circles.append(circle_object)
            if object_info.attrib["type"] == "line":
                line_object = StraightLine()
                line_object.start_point.x = float(object_info.attrib["start_x"])
                line_object.start_point.y = float(object_info.attrib["start_y"])
                line_object.start_point.z = 0
                line_object.end_point.x = float(object_info.attrib["end_x"])
                line_object.end_point.y = float(object_info.attrib["end_y"])
                line_object.end_point.z = 0
                self.field_model_msg.lines.append(line_object)

    def publish(self):
        self.field_model_msg.header.stamp = rospy.Time.now()
        self.field_model_pub.publish(self.field_model_msg)

if __name__ == '__main__':
    rospy.init_node('field_model_loader', anonymous=True)
    loader = model_loader()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        loader.publish()
        rate.sleep()
