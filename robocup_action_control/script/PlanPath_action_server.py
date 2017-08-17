#! /usr/bin/env python
import rospy
import actionlib
import robocup_action_control.msg
import threading
from geometry_msgs.msg import PoseStamped,Pose2D
from humanoid_nav_msgs.srv import PlanFootsteps
from nav_msgs.msg import Path
from humanoid_nav_msgs.msg import StepTarget

class PlanPath_Action():
    _feedback = robocup_action_control.msg.PlanPathFeedback()
    _result   = robocup_action_control.msg.PlanPathResult()
    path = Path()
    steps = StepTarget()
    success = True
    finished = False

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, robocup_action_control.msg.PlanPathAction, execute_cb=self.execute_cb)
        th_planning_client = threading.Thread(target=self.planning_client, name="th_planning_client")
        self._as.start()

    def planning_client(self, initial_pose, goal):
        rospy.wait_for_service('plan_footsteps')
        try:
            plan_footsteps_client = rospy.ServiceProxy('add_two_ints', AddTwoInts)
            ans = plan_footsteps_client(start, goal)
            self._result.path.steps = ans.footsteps
            self._result.path.path = ans.footsteps
            self._result.path.success = ans.success
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
            self._result.path.steps = StepTarget()
            self._result.path.path = Path()
            self._result.path.success = False
        self.finished = True

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(10)
        th_planning_client.start()
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            duration = now - start_time
            self._feedback.duration = duration
            self._as.publish_feedback(self._feedback)
            if self.finished == True:
                self.finished == False
                break
            r.sleep()
        self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('find_ball')
    PlanPath_Action(rospy.get_name())
    rospy.spin()
