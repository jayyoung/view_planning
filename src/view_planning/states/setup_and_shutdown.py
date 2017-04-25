#! /usr/bin/env python
import rospy
import smach
import smach_ros
import json
import math

from std_msgs.msg import *
from sensor_msgs.msg import *

class ScitosSetup(smach.State):
    #
    #
    # Adapted fron Lars Kunze's code, originally for VIPER at:
    # https://github.com/kunzel/viper_ros/blob/y4/src/viper_ros/navigation.py
    #
    #

    def __init__(self):
        smach.State.__init__(self,outcomes=['SUCCEEDED','ABORTED','PREEMPTED'],
        input_keys=['waypoint','target_nav_roi_id','target_surface_roi_id','max_num_of_views'])

    def execute(self, data):
        rospy.loginfo("Beginning Setup action")
        rospy.loginfo("Waypoint: " + data.waypoint)
        rospy.loginfo("Nav ROI: " + data.target_nav_roi_id)
        rospy.loginfo("Surface ROI: " + data.target_surface_roi_id)
        rospy.loginfo("Max Num Views: " + str(data.max_num_of_views))




        if self.preempt_requested():
            self.service_preempt()
            return 'PREEMPTED'

        #data.views = []

        #userdata.percentage_complete  = (float((self.plan_length - len(self.views))) / float(self.plan_length))  * 100
        return 'SUCCEEDED'


class ScitosShutdown(smach.State):
    #
    #
    # Adapted fron Lars Kunze's code, originally for VIPER at:
    # https://github.com/kunzel/viper_ros/blob/y4/src/viper_ros/navigation.py
    #
    #

    def __init__(self):
        smach.State.__init__(self,outcomes=['SUCCEEDED','ABORTED','PREEMPTED'])

    def execute(self, data):
        rospy.loginfo("Beginning shutdown action")

        if self.preempt_requested():
            self.service_preempt()
            return 'PREEMPTED'

        #userdata.percentage_complete  = (float((self.plan_length - len(self.views))) / float(self.plan_length))  * 100
        return 'SUCCEEDED'
