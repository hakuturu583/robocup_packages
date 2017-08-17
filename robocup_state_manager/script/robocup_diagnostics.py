#!/usr/bin/env python
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from robocup_msgs.msg import Event,PlayingState
from naoqi_bridge_msgs.msg import FadeRGB
from std_msgs.msg import String,ColorRGBA
import numpy as np

class diagnostics_mamager:
    def __init__(self):
        self.event_sub = rospy.Subscriber('/event',Event,self.event_callback,queue_size=10)
        self.diagnostic = rospy.Subscriber('/diagnostics_agg',DiagnosticArray,self.diagnostic_callback,queue_size=10)
        #self.playing_state_sub = rospy.Subscriber('/state',PlayingState,self.playing_state_callback,queue_size=10)
        self.speech_msg = String()
        self.led_msg = FadeRGB()
        self.load_colors()
        self.led_msg.fade_duration = rospy.Duration(0)
        self.state_updated_time = rospy.Time.now()
        self.ear_led_msg = FadeRGB()
        self.ear_led_msg.fade_duration = rospy.Duration(0)
        self.error_state_before = 0
        self.led_pub = rospy.Publisher('/fade_rgb',FadeRGB,queue_size=10)

    def diagnostic_callback(self,data):
        for state in data.status:
            if state.name == "/Nao" and state.level != self.error_state_before:
                if state.level == state.ERROR:
                    self.led_msg.color = self.color_red
                    self.led_msg.led_name = "FaceLeds"
                    self.led_pub.publish(self.led_msg)
                if state.level == state.WARN:
                    self.led_msg.color = self.color_yellow
                    self.led_msg.led_name = "FaceLeds"
                    self.led_pub.publish(self.led_msg)
                if state.level == state.OK:
                    self.led_msg.color = self.color_white
                    self.led_msg.led_name = "FaceLeds"
                    self.led_pub.publish(self.led_msg)
                state.level = self.error_state_before
            if state.name == "/Nao/Battery":
                for value in state.values:
                    if value.key == "nao_power: Battery":
                        try:
                            percent = value.value.split("(")[1].split("%")[0]
                            rospy.loginfo("Battery " + str(percent) + "% remaining")
                            self.light_ear_led(float(percent))
                        except:
                            pass

    def light_ear_led(self,percent):
        num_lights = int(percent/10)
        for i in range(1,10):
            if i <= num_lights:
                self.ear_led_msg.color = self.color_blue
                self.ear_led_msg.led_name = "RightEarLed" + str(i)
                self.led_pub.publish(self.ear_led_msg)
                self.ear_led_msg.led_name = "LeftEarLed" + str(i)
                self.led_pub.publish(self.ear_led_msg)
            else:
                self.ear_led_msg.color = self.color_black
                self.ear_led_msg.led_name = "RightEarLed" + str(i)
                self.led_pub.publish(self.ear_led_msg)
                self.ear_led_msg.led_name = "LeftEarLed" + str(i)
                self.led_pub.publish(self.ear_led_msg)

    def event_callback(self,data):
        if data.level == Event.ERROR:
            rospy.logerr("ERROR at:" + data.node_info + " :" + data.event_name)
        if data.level == Event.WARNING:
            rospy.logwarn("WARNING at:" + data.node_info + " :" + data.event_name)

    def load_colors(self):
        self.color_red = ColorRGBA()
        self.color_red.r = 1
        self.color_red.g = 0
        self.color_red.b = 0
        self.color_red.a = 1
        self.color_green = ColorRGBA()
        self.color_green.r = 0
        self.color_green.g = 1
        self.color_green.b = 0
        self.color_green.a = 1
        self.color_blue = ColorRGBA()
        self.color_blue.r = 0
        self.color_blue.g = 0
        self.color_blue.b = 1
        self.color_blue.a = 1
        self.color_yellow = ColorRGBA()
        self.color_yellow.r = 1
        self.color_yellow.g = 1
        self.color_yellow.b = 0
        self.color_yellow.a = 1
        self.color_purple = ColorRGBA()
        self.color_purple.r = 1
        self.color_purple.g = 0
        self.color_purple.b = 1
        self.color_purple.a = 1
        self.color_cyan = ColorRGBA()
        self.color_cyan.r = 0
        self.color_cyan.g = 1
        self.color_cyan.b = 1
        self.color_cyan.a = 1
        self.color_white = ColorRGBA()
        self.color_white.r = 1
        self.color_white.g = 1
        self.color_white.b = 1
        self.color_white.a = 1
        self.color_black = ColorRGBA()
        self.color_black.r = 0
        self.color_black.g = 0
        self.color_black.b = 0
        self.color_black.a = 0

if __name__ == '__main__':
    rospy.init_node('robocup_diagnostics', anonymous=False)
    diagnostics = diagnostics_mamager()
    rospy.spin()
