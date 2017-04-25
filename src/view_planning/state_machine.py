#! /usr/bin/env python
import rospy
import smach
import smach_ros
import sys

import actionlib
from actionlib_msgs.msg import *

##from move_base_msgs.msg import *
#from strands_navigation_msgs.msg import *

#from geometry_msgs.msg import Polygon
#from geometry_msgs.msg import Point32
#from geometry_msgs.msg import Pose

from states.plan import ScitosPlan
from states.perceive import ScitosPerception
from states.navigate import ScitosGoTo
from states.execute import ScitosExecutive
from states.setup_and_shutdown import ScitosShutdown,ScitosSetup


import threading

class ViewPlanMachine(smach.StateMachine):
    def __init__(self):
        smach.StateMachine.__init__(self, outcomes=['SUCCEEDED','ABORTED','PREEMPTED'])
        self.setup           = ScitosSetup()
        self.view_planning   = ScitosPlan()
        self.executive       = ScitosExecutive()
        self.perception      = ScitosPerception()
        self.goto            = ScitosGoTo()
        self.shutdown        = ScitosShutdown()

        with self:
            smach.StateMachine.add('Setup', self.setup,
                                   transitions={'SUCCEEDED': 'PlanViews',
                                                'ABORTED':'Shutdown',
                                                'PREEMPTED':'Shutdown'})

            smach.StateMachine.add('PlanViews', self.view_planning,
                                   transitions={'SUCCEEDED': 'ExecuteView',
                                                'ABORTED':'Shutdown',
                                                'PREEMPTED':'Shutdown'})

            smach.StateMachine.add('ExecuteView', self.executive,
                                   transitions={'SUCCEEDED': 'GoTo',
                                                'ABORTED':'Shutdown',
                                                'PREEMPTED':'Shutdown'})

            smach.StateMachine.add('GoTo', self.goto,
                                   transitions={'SUCCEEDED': 'Perception',
                                                'ABORTED':'ExecuteView',
                                                'PREEMPTED':'Shutdown'})

            smach.StateMachine.add('Perception', self.perception,
                                   transitions={'SUCCEEDED':'ExecuteView',
                                                'ABORTED':'Shutdown',
                                                'PREEMPTED':'Shutdown'})

            smach.StateMachine.add('Shutdown', self.shutdown,
                                   transitions={'SUCCEEDED':'SUCCEEDED',
                                                'ABORTED':'ABORTED',
                                                'PREEMPTED':'PREEMPTED'})

if __name__ == '__main__':
    rospy.init_node('sm_test', anonymous = False)

    s = ViewPlanMachine()
    s.userdata.waypoint = "Butts"
    s.userdata.target_nav_roi_id = "1"
    s.userdata.target_surface_roi_id = "2"
    s.userdata.max_num_of_views = 5


    smach_thread = threading.Thread(target = s.execute)
    smach_thread.start()
