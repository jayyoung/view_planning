#! /usr/bin/env python
import rospy
import smach
import smach_ros
import json
import math

from std_msgs.msg import *
from sensor_msgs.msg import *

class ScitosPlan(smach.State):
    #
    #
    # Adapted fron Lars Kunze's code, originally for VIPER at:
    # https://github.com/kunzel/viper_ros/blob/y4/src/viper_ros/navigation.py
    #
    #

    def __init__(self):
        smach.State.__init__(self,outcomes=['SUCCEEDED','ABORTED','PREEMPTED'],
        input_keys=['waypoint','target_nav_roi_id','target_surface_roi_id','max_num_of_views'],
        output_keys=['views'])

    def execute(self, data):
        rospy.loginfo("Beginning plan action")

        if self.preempt_requested():
            self.service_preempt()
            return 'PREEMPTED'
            

        rospy.loginfo("Planning completed")
        data.views = ["1","2","3"]

        #userdata.percentage_complete  = (float((self.plan_length - len(self.views))) / float(self.plan_length))  * 100
        return 'SUCCEEDED'
