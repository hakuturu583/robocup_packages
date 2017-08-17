import state_base
import rospy
import event_buffer

buf = event_buffer.event_buffer()

class WALK_TO_BALL(state_base.state_base):
    def execute(self, userdata):
        rospy.sleep(1)
        self.publish()
        event_list = buf.read()
        return 'BALL_DETECTED'

class FIND_BALL(state_base.state_base):
    def execute(self, userdata):
        rospy.sleep(1)
        self.publish()
        event_list = buf.read()
        return 'BALL_LOST'

class WALK_RANDOM(state_base.state_base):
    def execute(self, userdata):
        rospy.sleep(1)
        self.publish()
        event_list = buf.read()
        return 'BALL_LOST'

class TURN_LEFT(state_base.state_base):
    def execute(self, userdata):
        rospy.sleep(1)
        self.publish()
        event_list = buf.read()
        return 'SUCCESS'

class TURN_RIGHT(state_base.state_base):
    def execute(self, userdata):
        rospy.sleep(1)
        self.publish()
        event_list = buf.read()
        return 'SUCCESS'

class GO_BACK(state_base.state_base):
    def execute(self, userdata):
        rospy.sleep(1)
        self.publish()
        event_list = buf.read()
        return 'SUCCESS'
