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


    def generate_random(self,nav_area):
        #rospy.loginfo("-- Generating View --")
        self.view_data = nav_area.get_random_points_in_area()[0]

        origin = [self.view_data.x,self.view_data.y,1.75]

        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = "/map"
        ps.pose.position.x = self.view_data.x
        ps.pose.position.y = self.view_data.y
        ps.pose.position.z = 1.75

        qt = geometry_msgs.msg.Quaternion()
        yaw = random.uniform(0, 2*math.pi)
        deg = math.degrees(yaw)
        tlt = 0

        v = ViewFrustum()

        ## MUST ALWAYS TRANSLATE BEFORE ROTATE ##
        v.translate(origin)
        v.reset()
        v.panTo(deg)
        v.translate(origin)

        v.pan_angle = 0
        v.tilt_angle = 0
        roi_hull = nav_area.obs_polygon
        view_hull = Polygon(v.raw_points)
        #print(view_hull)
        if(roi_hull.intersects(view_hull)):
            pass
        else:
            #print("init view doesn't intersect the obs polygon")
            return self.generate_random(nav_area)

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
            if(frust.point_in_hull(np.array(point))):
                overlapping_points+=1
        degree_of_overlap = float(overlapping_points)/float(len(vmap.points))
        #print("pan: " + str(frust.pan_angle))

        #print("tilt: " + str(frust.pan_angle))
        #print("overlapping points:" + str(overlapping_points))
        #print("total points: " + str(len(vmap.points)))
        #print("degree: " + str(degree_of_overlap))

        # calculate the robot distance of this view from the centroid of the map #

        points_centroid = vmap.get_centroid()
        frust_centroid = self.get_cent(frust.raw_points)
        print("points centroid:"+str(points_centroid))
        print("frust centroid:"+str(frust_centroid))

        #centroid_point = Point(centroid)
        #cp = np.asarray([pose.pose.position.x,pose.pose.position.y,pose.pose.position.z])


        dist_to_centroid = np.linalg.norm(points_centroid-frust_centroid)
        print(dist_to_centroid)

        return degree_of_overlap,dist_to_centroid
        #,abs(frust.pan_angle)

    def get_cent(self,arr):
        arr = np.array(arr)
        length = arr.shape[0]
        sum_x = np.sum(arr[:, 0])
        sum_y = np.sum(arr[:, 1])
        sum_z = np.sum(arr[:, 2])
        return sum_x/length, sum_y/length, sum_z/length


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

    def calc_centroid(self):
        points = np.asarray(self.points)
        length = points.shape[0]
        sum_x = np.sum(points[:, 0])
        sum_y = np.sum(points[:, 1])
        sum_z = np.sum(points[:, 2])
        self.centroid = np.asarray([sum_x/length, sum_y/length, sum_z/length])

    def get_centroid(self):
        return self.centroid

    def get_visualisation(self):
        centroid_marker = Marker()
        centroid_marker.header.frame_id = "/map"
        centroid_marker.type = Marker.SPHERE_LIST
        centroid_marker.header.stamp = rospy.Time.now()
        for k in self.points:
            px = geometry_msgs.msg.Point()
            px.x = k[0]
            px.y = k[1]
            px.z = k[2]
            centroid_marker.points.append(px)
        centroid_marker.scale.x = 0.1
        centroid_marker.scale.y = 0.1
        centroid_marker.scale.z = 0.1
        centroid_marker.color.a = 1.0
        centroid_marker.color.r = 0.0
        centroid_marker.color.g = 1.0
        centroid_marker.color.b = 0.0
        return centroid_marker

if __name__ == '__main__':
    rospy.init_node('sm_test', anonymous = False)
    n = NavAreaGenerator("2","1",0.4)
    na = n.generate_from_soma_roi()


    marker_publisher = rospy.Publisher("/view_planner/candidate_frustrum_geometry", Marker,queue_size=5)
    pose_publisher = rospy.Publisher("/view_planner/candidate_robot_pose", geometry_msgs.msg.PoseStamped,queue_size=5)
    frust_pose_publisher = rospy.Publisher("/view_planner/candidate_frustrum_pose", geometry_msgs.msg.PoseStamped,queue_size=5)


    for k in range(25):
        view = RobotViewState()
        pose,frustrum = view.generate_random(na)
        for i in range(5):
            frustrum.pan(5)
            pose_publisher.publish(pose)
            frust_pose_publisher.publish(frustrum.pose)
            marker_publisher.publish(frustrum.get_visualisation())
            rospy.sleep(0.1)

        pose_publisher.publish(pose)
        frust_pose_publisher.publish(frustrum.pose)

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

        #view_root_publisher.publish(centroid_marker)

        marker_publisher.publish(frustrum.get_visualisation())

        rospy.sleep(1)
