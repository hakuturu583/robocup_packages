#!/usr/bin/env python
import rospy
import smach
import smach_ros
from robocup_msgs.msg import PlayingState,Event
import rospkg

state_msg = PlayingState()
state_pub = rospy.Publisher("/state",PlayingState,queue_size=10)

class state_base(smach.State):
    def __init__(self,outcomes,transitions):
        self.transitions = {}
        for i in range(len(outcomes)):
            self.transitions.update({outcomes[i]:transitions[i]})
        smach.State.__init__(self, outcomes=outcomes)
        self.state_dict = {}
        self.event_dict = {}
        self.read_state_message()
        #self.read_event_message()

    def get_state_name(self):
        return self.__class__.__name__

    def query_state_ID(self):
        return self.state_dict[self.__class__.__name__]

    def read_state_message(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path("robocup_msgs")
        path = path + "/msg/PlayingState.msg"
        f = open(path)
        lines = f.readlines()
        for line in lines:
            line = line.strip()
            line = line.split("uint8")[1]
            line = line.split("=")
            if len(line) == 2:
                line[0] = line[0].replace(" ","")
                self.state_dict.update({line[0]:int(line[1])})

    #def read_event_message(self):
    #    rospack = rospkg.RosPack()
    #    path = rospack.get_path("robocup_msgs")
    #    path = path + "/msg/Event.msg"
    #    f = open(path)
    #    lines = f.readlines()
    #    for line in lines:
    #        line = line.strip()
    #        line = line.split("uint8")[1]
    #        line = line.split("=")
    #        if len(line) == 2:
    #            line[0] = line[0].replace(" ","")
    #            self.event_dict.update({line[0]:int(line[1])})

    def publish(self):
        #rospy.logerr(self.__class__.__name__)
        #rospy.logwarn(self.state_dict)
        state_msg.ID = self.query_state_ID()
        state_pub.publish(state_msg)

if __name__ == '__main__':
    walk_to_ball = WALK_TO_BALL(['FindBall_Succeed','FindBall_Failed'],['WALK_TO_BALL','FINDING_BALL']);
