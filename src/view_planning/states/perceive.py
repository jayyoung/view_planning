#! /usr/bin/env python
import rospy
import smach
import smach_ros
import json
import math

from std_msgs.msg import *
from sensor_msgs.msg import *


class ScitosPerception(smach.State):
    #
    #
    # Adapted fron Lars Kunze's code, originally for VIPER at:
    # https://github.com/kunzel/viper_ros/blob/y4/src/viper_ros/navigation.py
    #
    #

    def __init__(self):
        smach.State.__init__(self,outcomes=['SUCCEEDED','ABORTED','PREEMPTED'])
        rospy.loginfo("Done")
        self.first_call = True

    def execute(self, data):
        rospy.loginfo("Beginning perceive action")

        if self.preempt_requested():
            self.service_preempt()
            return 'PREEMPTED'

        #
        # INCLUDE CALLS TO PERCEPTION HERE
        #


        rospy.loginfo("Perception completed")

        #userdata.percentage_complete  = (float((self.plan_length - len(self.views))) / float(self.plan_length))  * 100
        return 'SUCCEEDED'
