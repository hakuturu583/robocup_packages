#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
import numpy as np
from tf.transformations import euler_from_quaternion
from math import sin, cos
import tf

class Intersector(object):
    def __init__(self):
        self.plane_frame = rospy.get_param('~plane_frame')
        self.table_pose  = PoseStamped()
        self.tfl         = tf.TransformListener()
        self.table_pose.pose.orientation.w = 1.0
        self.table_pose.header.frame_id = self.plane_frame

    def run(self,pose):
        intersection = self.cast_ray(pose,self.table_pose, self.tfl)
        if intersection:
            return intersection

    def cast_ray(self, pose, plane, tfl):
        # assume the plane passes through table.pose
        # and the vector table.pose.x,table.pose.y,table.pose.z+1 is a
        # normal for the table in its frame

        # point
        q = np.array([
            plane.pose.position.x,
            plane.pose.position.y,
            plane.pose.position.z
        ])

        # normal
        m = q + [0,0,1]

        try:
            tfl.waitForTransform(plane.header.frame_id, pose.header.frame_id, rospy.Time(0), rospy.Duration.from_sec(5))
            pose.header.stamp = rospy.Time(0)
            pose_transformed = tfl.transformPose(plane.header.frame_id, pose)
        except tf.Exception, e:
            rospy.logerr('trouble with tf lookup')
            rospy.logerr(e.message)
            return False

        # origin vector
        p = np.array([pose_transformed.pose.position.x, pose_transformed.pose.position.y, pose_transformed.pose.position.z])
        quat = (
            pose_transformed.pose.orientation.x,
            pose_transformed.pose.orientation.y,
            pose_transformed.pose.orientation.z,
            pose_transformed.pose.orientation.w
        )
        ax,ay,az = euler_from_quaternion(quat)
        # direction vector
        # a pose is basically spherical coordinates, so convert to cartesian
        d = np.array([
            -cos(az)*cos(ay),
            -sin(az)*cos(ay),
             sin(ay)
        ])
        print np.dot(q-p,m),np.dot(d,m)
        # intersection
        t = np.dot(q-p,m) / np.dot(d,m)
        if t < 0: # some normal must be flipped since t is normally > 0
            v = PointStamped()
            v.header = plane.header
            v.point.x, v.point.y, v.point.z = p + t*d
            return v
        return False

if __name__ == '__main__':
    intersector = Intersector()
