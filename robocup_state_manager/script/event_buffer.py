#!/usr/bin/env python
from robocup_msgs.msg import Event
import rospy
import copy

class event_buffer:
    def __init__(self):
        self.buf = []
        self.is_reading = False
        rospy.Subscriber('/event',Event,self.recieve,queue_size=10)

    def clear(self):
        while True:
            if self.is_reading == False:
                break
        self.buf = []
    def read(self):
        self.is_reading = True
        data = copy.copy(self.buf)
        self.is_reading = False
        return data

    def recieve(self,data):
        self.buf.append(data)
