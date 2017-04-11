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

    def generate_from_data(self,position,orientation,pan,tilt):
        origin = [position[0],position[1],position[2]]
        width = 0.7
        height = 0.7
        length = 1.5
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = "/map"
        ps.pose.position.x = position[0]
        ps.pose.position.y = position[1]
        ps.pose.position.z = position[2]


    def generate_random(self,nav_area_generator):
        #rospy.loginfo("-- Generating View --")
        self.view_pose = nav_area_generator.get_random_points_in_area()[0]

        origin = [self.view_pose.x,self.view_pose.y,1.75]
        width = 0.7
        height = 0.7
        length = 1.5

        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = "/map"
        ps.pose.position.x =self.view_pose.x
        ps.pose.position.y =self.view_pose.y
        ps.pose.position.z = 1.75

        qt = geometry_msgs.msg.Quaternion()
        yaw = random.uniform(0, 2*math.pi)
        deg = math.degrees(yaw)

        v = ViewFrustum(origin,[origin,
        (origin[0]+length,origin[1]+width,origin[2]+height),
        (origin[0]+length,origin[1]+-width,origin[2]+-height),
        (origin[0]+length,origin[1]+width,origin[2]+-height),
        (origin[0]+length,origin[1]+(-width),origin[2]+height)],deg)

        q = list(tf.transformations.quaternion_about_axis(yaw, (0,0,1)))
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]
        # returns the posestamped and the frsutrum polygon #
        return ps,v

class ViewFitnessEvaluator():
    def __init__(self):
        # calculates for some given view parameters
        # what the overlap is with the voxel map as a proportion of possible
        # distance from voxel centroid
        # other things? like if the view is in a tabu region?

        pass
    def evaluate(self,view,vmap):
        # calculate how well this view overlaps the points in the map #
        frust = view[0]
        pose = view[1]

        overlapping_points = 0
        for point in vmap.points:
            if(frust.contains(Point(point))):
                overlapping_points+=1
        degree_of_overlap = float(overlapping_points)/float(len(vmap.points))

        # calculate the robot distance of this view from the centroid of the map #
        points = np.asarray(vmap.points)
        length = points.shape[0]
        sum_x = np.sum(points[:, 0])
        sum_y = np.sum(points[:, 1])
        sum_z = np.sum(points[:, 2])
        centroid = np.asarray([sum_x/length, sum_y/length, sum_z/length])
        centroid_point = Point(centroid)

        cp = np.asarray([pose.pose.position.x,pose.pose.position.y,pose.pose.position.z])
        dist_to_centroid = np.linalg.norm(centroid-cp)
    #    rospy.loginfo([degree_of_overlap,dist_to_centroid])
        return degree_of_overlap,dist_to_centroid,frust.pan_angle




class VoxelMap():
    def __init__(self):
        # the segmented objects #
        # should alow us to access each point
        # and tell us what the centroid of the map is
        self.points = []
        pass

    def generate_from_octomap(self,octomap):
        pass

    def generate_dummy(self,pos):
        p = [pos[0],pos[1],pos[2]]
        self.points.append(p)



if __name__ == '__main__':
    rospy.init_node('sm_test', anonymous = False)
    n = NavAreaGenerator("2",0.4)
    na = n.generate_from_soma_roi()


    view_root_publisher = rospy.Publisher("/view_points", Marker,queue_size=5)
    marker_publisher = rospy.Publisher("/frust_points", Marker,queue_size=5)
    pose_publisher = rospy.Publisher("/frust_pose", geometry_msgs.msg.PoseStamped,queue_size=5)

    for k in range(5):
        view = RobotViewState()
        pose,frustrum = view.generate_random(na)
        frustrum.pan(90)
        pose_publisher.publish(pose)

        centroid_marker = Marker()
        centroid_marker.header.frame_id = "/map"
        centroid_marker.type = Marker.SPHERE
        centroid_marker.header.stamp = rospy.Time.now()
        centroid_marker.pose.position.x = pose.pose.position.x
        centroid_marker.pose.position.y = pose.pose.position.y
        centroid_marker.pose.position.z = 1.75
        centroid_marker.scale.x = n.inflation_radius
        centroid_marker.scale.y = n.inflation_radius
        centroid_marker.scale.z = n.inflation_radius
        centroid_marker.color.a = 1.0
        centroid_marker.color.r = 0.0
        centroid_marker.color.g = 1.0
        centroid_marker.color.b = 0.0

        view_root_publisher.publish(centroid_marker)

        marker_publisher.publish(frustrum.get_visualisation())

        rospy.sleep(1)
