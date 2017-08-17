#!/usr/bin/env python
import rospy
from naoqi import *
from geometry_msgs.msg import Twist,Pose2D
from std_msgs.msg import Bool
from robocup_msgs.msg import GameState,Event
from robocup_action_control.cfg import naoqi_robocup_walker_dynamic_paramsConfig
import dynamic_reconfigure.server

class robocup_walker():
    def __init__(self):
        self.walkr_enable = True
        self.nao_ip = rospy.get_param('/nao_ip')
        self.roscore_ip = rospy.get_param('/roscore_ip')
        self.broker = ALBroker("broker",self.roscore_ip,9559,self.nao_ip,9559)
        self.motionProx = ALProxy("ALMotion")
        self.postureProx = ALProxy("ALRobotPosture")
        self.memoryProx = ALProxy("ALMemory")
        try:
            self.lifeProx = ALProxy("ALAutonomousLife")
        except:
            import traceback
            traceback.print_exc()
        self.server = dynamic_reconfigure.server.Server(naoqi_robocup_walker_dynamic_paramsConfig,self.config_callback)
        self.postureProx.goToPosture("StandInit",0.5)
        self.is_penalized = False
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel',Twist,self.callback_vel,queue_size=1)
        self.cmd_pose_sub = rospy.Subscriber('/cmd_pose',Pose2D,self.callback_pose,queue_size=5)
        self.cmd_stop_sub = rospy.Subscriber('/stop',Bool,self.callback_stop,queue_size=1)
        self.event_sub = rospy.Subscriber('/event',Event,self.callback_event,queue_size=1)
        self.event_sub = rospy.Subscriber('/game_state',GameState,self.callback_game_state,queue_size=1)

    def callback_vel(self,data):
        #rospy.logwarn(self.is_penalized)
        #rospy.loginfo(self.walkr_enable)
        if self.walkr_enable == True and self.is_penalized == False:
            #self.motionProx.stopMove()
            self.motionProx.move(data.linear.x,data.linear.y,data.angular.z,self.move_config)

    def callback_pose(self,data):
        if self.walkr_enable == True and self.is_penalized == False:
            self.motionProx.post.moveTo(data.x,data.y,data.theta,self.move_config)

    def callback_game_state(self,data):
        if data.penalty_info.penalty == data.penalty_info.PENALTY_NONE:
            self.is_penalized = False
            self.postureProx.stopMove()
        else:
            self.is_penalized = True
            self.motionProx.stopMove()
            self.postureProx.goToPosture("StandInit",0.5)

    def callback_event(self,data):
        if data.id == data.ROBOT_FALLS:
            self.walkr_enable = False
            self.postureProx.goToPosture("StandInit",0.5)
            self.walkr_enable = True

    def callback_stop(self,data):
        self.motionProx.post.stopMove()
        if self.UseWholeBodyBalancer == True:
            self.motionProx.wbFootState("Plane","LLeg")
            self.motionProx.wbFootState("Plane","RLeg")
        self.postureProx.post.goToPosture("StandInit",0.5)

    def update_controller(self):
        self.motionProx.setSmartStiffnessEnabled(self.UseSmartStifness)
        self.motionProx.wbEnable(self.UseWholeBodyBalancer)
        if self.KillAutonomousLife == True:
            try:
                self.lifeProx.stopAll()
                self.lifeProx.setState("disabled")
            except:
                import traceback
                traceback.print_exc()
        else:
            try:
                self.lifeProx.setState("solitary")
            except:
                import traceback
                traceback.print_exc()
        try:
            rospy.loginfo("allife state:" + self.lifeProx.getState())
        except:
            import traceback
            traceback.print_exc()

    def set_init_params(self):
        self.MaxStepX = 0.080
        self.MaxStepY = 0.160
        self.MaxStepTheta = 0.524
        self.MaxStepFrequency = 1.0
        self.StepHeight = 0.040
        self.TorsoWx = 0
        self.TorsoWy = 0
        self.UseSmartStifness = True
        self.UseWholeBodyBalancer = True
        self.KillAutonomousLife = True
        self.move_config = [["MaxStepX",self.MaxStepX],
                            ["MaxStepY",self.MaxStepY],
                            ["MaxStepTheta",self.MaxStepTheta],
                            ["MaxStepTheta",self.MaxStepTheta],
                            ["MaxStepFrequency",self.MaxStepFrequency],
                            ["StepHeight",self.StepHeight],
                            ["TorsoWx",self.TorsoWx],
                            ["TorsoWy",self.TorsoWy]]

    def config_callback(self,config,level):
        self.MaxStepX = config.MaxStepX
        self.MaxStepY = config.MaxStepY
        self.MaxStepTheta = config.MaxStepTheta
        self.MaxStepFrequency = config.MaxStepFrequency
        self.StepHeight = config.StepHeight
        self.TorsoWx = config.TorsoWx
        self.TorsoWy = config.TorsoWy
        self.UseSmartStifness = config.UseSmartStifness
        self.UseWholeBodyBalancer = config.UseWholeBodyBalancer
        self.KillAutonomousLife = config.KillAutonomousLife

        self.move_config = [["MaxStepX",self.MaxStepX],
                            ["MaxStepY",self.MaxStepY],
                            ["MaxStepTheta",self.MaxStepTheta],
                            ["MaxStepTheta",self.MaxStepTheta],
                            ["MaxStepFrequency",self.MaxStepFrequency],
                            ["StepHeight",self.StepHeight],
                            ["TorsoWx",self.TorsoWx],
                            ["TorsoWy",self.TorsoWy]]
        self.update_controller()
        return config

if __name__ == '__main__':
    rospy.init_node('naoqi_robocup_walker', anonymous=False)
    walker = robocup_walker()
    rospy.spin()
    walker.motionProx.stopMove()
