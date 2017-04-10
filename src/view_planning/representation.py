#!/usr/bin/env/python
import numpy as np
import shapely
from shapely.geometry import *
from shapely import affinity
import roslib
import rospy
from visualization_msgs.msg import Marker,MarkerArray
import geometry_msgs
from soma_msgs.msg import SOMAObject,SOMAROIObject
from soma_manager.srv import *
import random
from nav_areas import *
from view_frustrum import *
from random import randint
import tf
import math

class RobotViewState():
    def __init__(self):
        pass

    def generate_random(self,nav_area_generator):
        pt = nav_area_generator.get_random_points_in_area()[0]

        origin = [pt.x,pt.y,1.75]
        width = 0.7
        height = 0.7
        length = 1.5
        v = ViewFrustum(origin,[origin,
        (origin[0]+length,origin[1]+width,origin[2]+height),
        (origin[0]+length,origin[1]+-width,origin[2]+-height),
        (origin[0]+length,origin[1]+width,origin[2]+-height),
        (origin[0]+length,origin[1]+(-width),origin[2]+height)])

        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = "/map"
        ps.pose.position.x = pt.x
        ps.pose.position.y = pt.y
        ps.pose.position.z = 1.75

        qt = geometry_msgs.msg.Quaternion()
        yaw = random.uniform(0, 2*math.pi)
        deg = math.degrees(yaw)
        q = list(tf.transformations.quaternion_about_axis(yaw, (0,0,1)))
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]

        v.pan(deg)

        # returns the posestamped and the frsutrum polygon #
        return ps,v


if __name__ == '__main__':
    rospy.init_node('sm_test', anonymous = False)
    n = NavAreaGenerator("2")
    na = n.generate_from_soma_roi()


    view_root_publisher = rospy.Publisher("/view_points", Marker,queue_size=5)
    marker_publisher = rospy.Publisher("/frust_points", Marker,queue_size=5)
    pose_publisher = rospy.Publisher("/frust_pose", geometry_msgs.msg.PoseStamped,queue_size=5)

    for k in range(20):

        print("uhh")
        view = RobotViewState()
        pose,frustrum = view.generate_random(na)

        pose_publisher.publish(pose)

        centroid_marker = Marker()
        centroid_marker.header.frame_id = "/map"
        centroid_marker.type = Marker.SPHERE
        centroid_marker.header.stamp = rospy.Time.now()
        centroid_marker.pose.position.x = pose.pose.position.x
        centroid_marker.pose.position.y = pose.pose.position.y
        centroid_marker.pose.position.z = 1.75
        centroid_marker.scale.x = 0.5
        centroid_marker.scale.y = 0.5
        centroid_marker.scale.z = 0.5
        centroid_marker.color.a = 1.0
        centroid_marker.color.r = 0.0
        centroid_marker.color.g = 1.0
        centroid_marker.color.b = 0.0

        view_root_publisher.publish(centroid_marker)

        marker_publisher.publish(frustrum.get_visualisation())

        rospy.sleep(2)
