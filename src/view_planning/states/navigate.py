#! /usr/bin/env python
import rospy
import smach
import smach_ros
import json
import math
from actionlib import *
from actionlib.msg import *
from scitos_ptu.msg import PtuGotoAction,PtuGotoGoal
from strands_navigation_msgs.msg import MonitoredNavigationAction, MonitoredNavigationGoal
from std_msgs.msg import *
from sensor_msgs.msg import *



class ScitosGoTo(smach.State):
    #
    #
    # Adapted fron Lars Kunze's code, originally for VIPER at:
    # https://github.com/kunzel/viper_ros/blob/y4/src/viper_ros/navigation.py
    #
    #

    def __init__(self):
        smach.State.__init__(self,outcomes=['SUCCEEDED','ABORTED','PREEMPTED'], input_keys=['waypoint','robot_pose','ptu_state'])
        self.nav_client = actionlib.SimpleActionClient('monitored_navigation', MonitoredNavigationAction)
        #rospy.loginfo("Wait for monitored navigation server")
        #self.nav_client.wait_for_server(rospy.Duration(60))
        #rospy.loginfo("Done")

        #self.ptu_client = actionlib.SimpleActionClient('SetPTUState', PtuGotoAction)
        #rospy.loginfo("Wait for PTU action server")
        #self.ptu_client.wait_for_server(rospy.Duration(60))
        #rospy.loginfo("Done")

    def execute(self, userdata):
        rospy.loginfo("Beginning movement action")
        if self.preempt_requested():
            self.service_preempt()
            return 'PREEMPTED'

        return 'SUCCEEDED'

        pose = userdata.robot_pose
        rospy.loginfo("ROBOT MOVING TO: x: %f y: %f", pose.position.x, pose.position.y)
      	goal = MonitoredNavigationGoal()
        goal.action_server = 'move_base'
      	goal.target_pose.header.frame_id = 'map'
      	goal.target_pose.header.stamp = rospy.get_rostime() #rospy.Time.now()
      	goal.target_pose.pose = pose

      	self.nav_client.send_goal(goal)
      	self.nav_client.wait_for_result()
        res = self.nav_client.get_result()

        if(res.outcome is "succeeded"):
            rospy.loginfo("Successfully moved to location")
        else:
            rospy.logerr("Could not complete movement action, aborting")
            return 'ABORTED'

        if self.preempt_requested():
            self.service_preempt()
            return 'PREEMPTED'

        ptu_state = userdata.ptu_state
        goal = PtuGotoGoal()
        goal.pan = math.degrees(ptu_state.position[ptu_state.name.index('pan')])
        goal.tilt = math.degrees(ptu_state.position[ptu_state.name.index('tilt')])
        goal.pan_vel = ptu_state.velocity[ptu_state.name.index('pan')] * 100
        goal.tilt_vel = ptu_state.velocity[ptu_state.name.index('tilt')] * 100
        rospy.loginfo("SETTING PTU STATE TO: pan: %f tilt: %f", goal.pan, goal.tilt)
        self.ptu_client.send_goal(goal)
        self.ptu_client.wait_for_result()
        rospy.loginfo("PTU Movement Succeeded")

        return 'SUCCEEDED'
