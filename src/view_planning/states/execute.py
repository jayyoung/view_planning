#! /usr/bin/env python
import rospy
import smach
import smach_ros
import json
import math

from std_msgs.msg import *
from sensor_msgs.msg import *


class ScitosExecutive(smach.State):
    #
    #
    # Adapted fron Lars Kunze's code, originally for VIPER at:
    # https://github.com/kunzel/viper_ros/blob/y4/src/viper_ros/navigation.py
    #
    #

    def __init__(self):
        smach.State.__init__(self,outcomes=['SUCCEEDED','ABORTED','PREEMPTED'], input_keys=['views'], output_keys=['percentage_complete'])
        rospy.loginfo("Done")
        self.first_call = True

    def execute(self, data):
        rospy.loginfo("Beginning execute action")

        if self.preempt_requested():
            self.service_preempt()
            return 'PREEMPTED'

        if(self.first_call):
            self.views = data.views
            self.plan_length = len(data.views)
            self.first_call = False


        ## remember to go to shutdown state if we're out of views #
        if(self.views == []):
            return 'PREEMPTED'

        next_view = self.views.pop(0)
        #  remember to add in the target poses we're going to move the robot to
        #  here, ptu and position

        percentage_complete  = (float((self.plan_length - len(self.views))) / float(self.plan_length))  * 100
        data.percentage_complete = percentage_complete
        rospy.loginfo("PERCENTAGE OF VIEWS COMPLETE: " + str(percentage_complete))
        return 'SUCCEEDED'
