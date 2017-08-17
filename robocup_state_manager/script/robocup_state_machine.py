#!/usr/bin/env python
import rospy
import smach
import smach_ros
import state
from actionlib import *
from actionlib_msgs.msg import *

class stae_machine(smach.StateMachine):
    def add_state(self,state):
        self.add(state.get_state_name(),state ,state.transitions)
        self.trans = state.transitions

    def add_state_machine(self,name,trans):
        smach.StateMachine.add(name, self, trans)

# main
def main():
    rospy.init_node('smach_example_state_machine')
    # Create a SMACH state machine
    sm_robocup = stae_machine(outcomes=['GAME_FINISHED'])
    #find_ball = state.FIND_BALL(['BALL_DETECTED','BALL_LOST'],['WALK_TO_BALL','WALK_RANDOM'])
    #walk_random = state.WALK_RANDOM(['BALL_DETECTED','BALL_LOST'],['WALK_TO_BALL','FIND_BALL'])
    #walk_to_ball = state.WALK_TO_BALL(['BALL_DETECTED','BALL_LOST'],['WALK_TO_BALL','FIND_BALL'])
    find_ball = state.FIND_BALL(['BALL_DETECTED','BALL_LOST','SOMETHING_ON_ROBOT_LEFT','SOMETHING_ON_ROBOT_RIGHT','SOMETHING_ON_ROBOT_FRONT'],['WALK_TO_BALL','WALK_RANDOM','TURN_RIGHT','TURN_LEFT','GO_BACK'])
    walk_random = state.WALK_RANDOM(['BALL_DETECTED','BALL_LOST','SOMETHING_ON_ROBOT_LEFT','SOMETHING_ON_ROBOT_RIGHT','SOMETHING_ON_ROBOT_FRONT'],['WALK_TO_BALL','FIND_BALL','TURN_RIGHT','TURN_LEFT','GO_BACK'])
    walk_to_ball = state.WALK_TO_BALL(['BALL_DETECTED','BALL_LOST','SOMETHING_ON_ROBOT_LEFT','SOMETHING_ON_ROBOT_RIGHT','SOMETHING_ON_ROBOT_FRONT'],['WALK_TO_BALL','FIND_BALL','TURN_RIGHT','TURN_LEFT','GO_BACK'])
    turn_left = state.TURN_LEFT(['SUCCESS'],['GO_BACK'])
    turn_right = state.TURN_RIGHT(['SUCCESS'],['GO_BACK'])
    go_back = state.GO_BACK(['SUCCESS'],['FIND_BALL'])
    # Open the container
    with sm_robocup:
        # Add states to the container
        sm_robocup.add_state(find_ball)
        sm_robocup.add_state(walk_to_ball)
        sm_robocup.add_state(walk_random)
        sm_robocup.add_state(turn_left)
        sm_robocup.add_state(turn_right)
        sm_robocup.add_state(go_back)
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm_robocup, '/GAME_START')
    sis.start()
    # Execute SMACH plan
    outcome = sm_robocup.execute()
    rospy.spin()

if __name__ == '__main__':
    main()
