#!/usr/bin/env/python
import numpy as np
import shapely
from shapely.geometry import *
from shapely import affinity
import roslib
import rospy
from visualization_msgs.msg import Marker,MarkerArray
import geometry_msgs
import copy
import tf
import math
import geometry_msgs
import sys
import numpy
import numpy.random
from numpy import zeros, ones, arange, asarray, concatenate
from scipy.optimize import linprog

from scipy.spatial import ConvexHull

class BinaryPoint(shapely.geometry.Point):
    def __init__(self,pos):
        super(BinaryPoint,self).__init__(pos)
        self.visit_counter = 0

    def visited(self):
        return self.visit_counter == 0

def tilt(frust,angle):
    print(frust)
    new_coords = []
    for k in frust.exterior.coords:
        x = k[0]
        y = k[1]
        z = k[2]
        pk = (x,z,z)
        pk = affinity.rotate(Point(pk),angle,v.origin)
        pk = Point(pk)
        p = geometry_msgs.msg.Point()
        p.x = v.origin[0]+pk.x
        p.y = v.origin[1]+y
        p.z = v.origin[2]+pk.y
        k = (p.x,p.y,p.z)
        new_coords.append(k)
    new = ViewFrustum(frust.origin,new_coords)
    print(new)
    return new

def pan(frust,angle):
    print(frust)
    new_coords = []
    for k in frust.exterior.coords:
        x = k[0]
        y = k[1]
        z = k[2]
        pk = (x,y,z)
        pk = affinity.rotate(Point(pk),angle,v.origin)
        pk = Point(pk)
        p = geometry_msgs.msg.Point()
        p.x = pk.x
        p.y = pk.y
        p.z = pk.z
        k = (p.x,p.y,p.z)
        new_coords.append(k)
    new = ViewFrustum(frust.origin,new_coords)
    print(new)
    return new

class ViewFrustum(shapely.geometry.Polygon):

    def __init__(self,origin,vertices,default_pan_angle=0,default_tilt_angle=0):
        super(ViewFrustum,self).__init__(vertices)
        self.raw_vertices = vertices # vertices always in defauult, 0,0 pos #
        self.origin = origin
        #self.init_tilt_angle = default_tilt_angle
        #self.init_pan_angle = default_pan_angle
        self.pan_angle = default_pan_angle
        self.tilt_angle = default_tilt_angle
        self.agg_angle = 0
        self.pose = geometry_msgs.msg.PoseStamped()
        self.pose.header.frame_id = "/map"
        self.pose.pose.position.x = origin[0]
        self.pose.pose.position.y = origin[1]
        self.pose.pose.position.z = 1.75

        frust_orientation_q = list(tf.transformations.quaternion_about_axis(math.radians(self.pan_angle), (0,0,1)))
        frust_quat = geometry_msgs.msg.Quaternion()
        self.pose.pose.orientation.x = frust_orientation_q[0]
        self.pose.pose.orientation.y = frust_orientation_q[1]
        self.pose.pose.orientation.z = frust_orientation_q[2]
        self.pose.pose.orientation.w = frust_orientation_q[3]

    def point_in_hull(self,pnt):
        '''
        Given a set of points that defines a convex hull, uses simplex LP to determine
        whether point lies within hull.
        `hull_points` -- (N, 3) array of points defining the hull
        `pnt` -- point array of shape (3,)
        '''
        hull_points = np.array(self.raw_vertices)
        N = hull_points.shape[0]
        c = ones(N)
        A_eq = concatenate((hull_points, ones((N,1))), 1).T   # rows are x, y, z, 1
        b_eq = concatenate((pnt, (1,)))
        result = linprog(c, A_eq=A_eq, b_eq=b_eq)
        if result.success and c.dot(result.x) == 1.:
            return True
        return False



    def __deepcopy__(self, memo):
        nvs = ViewFrustum(copy.deepcopy(self.origin),copy.deepcopy(self.raw_vertices),copy.deepcopy(self.pan_angle),copy.deepcopy(self.tilt_angle))
        nvs.pose = copy.deepcopy(self.pose)
        nvs.agg_angle = self.agg_angle
        #nvs.pan_angle = self.pan_angle
        #nvs.tilt_angle = self.tilt_angle
        return nvs

    def pan(self,angle):
        #if(angle < 0):
        #    if(self.pan_angle-angle < -45):
        #        return
        #else:
        #    if(self.pan_angle+angle > 45):
        #        return
        new_coords = []
        self.pan_angle += angle
        for k in self.exterior.coords:
            x = k[0]
            y = k[1]
            z = k[2]
            pk = (x,y,z)
            pk = affinity.rotate(Point(pk),angle,self.origin)
            pk = Point(pk)
            p = geometry_msgs.msg.Point()
            p.x = pk.x
            p.y = pk.y
            p.z = pk.z
            k = (p.x,p.y,p.z)
            new_coords.append(k)
        self.raw_vertices = new_coords

        super(ViewFrustum,self).__init__(new_coords)

        self.agg_angle += angle

        self.pose = geometry_msgs.msg.PoseStamped()
        self.pose.header.frame_id = "/map"
        self.pose.pose.position.x = self.origin[0]
        self.pose.pose.position.y = self.origin[1]
        self.pose.pose.position.z = 1.75

        frust_orientation_q = list(tf.transformations.quaternion_about_axis(math.radians(self.agg_angle), (0,0,1)))
        frust_quat = geometry_msgs.msg.Quaternion()

        self.pose.pose.orientation.x = frust_orientation_q[0]
        self.pose.pose.orientation.y = frust_orientation_q[1]
        self.pose.pose.orientation.z = frust_orientation_q[2]
        self.pose.pose.orientation.w = frust_orientation_q[3]

    def tilt(self,angle):
        new_coords = []
        self.tilt_angle = angle
        for k in self.exterior.coords:
            x = k[0]
            y = k[1]
            z = k[2]
            pk = (x,z,z)
            pk = affinity.rotate(Point(pk),angle,self.origin)
            pk = Point(pk)
            p = geometry_msgs.msg.Point()
            p.x = pk.x
            p.y = y
            p.z = pk.y
            k = (p.x,p.y,p.z)
            new_coords.append(k)
        super(ViewFrustum,self).__init__(new_coords)

    def get_visualisation(self,colour="blue"):
        points_list = Marker()
        points_list.header.frame_id = "/map"
        points_list.type = Marker.LINE_LIST
        points_list.scale.x = 0.05
        points_list.scale.y = 0.1
        points_list.scale.z = 0.1
        points_list.color.a = 1.0

        if(colour is "blue"):
            points_list.color.r = 0.0
            points_list.color.g = 0.0
            points_list.color.b = 1.0

        if(colour is "red"):
            points_list.color.r = 1.0
            points_list.color.g = 0.0
            points_list.color.b = 0.0

        if(colour is "green"):
            points_list.color.r = 0.0
            points_list.color.g = 1.0
            points_list.color.b = 0.0



        #v.pan(3)
        #v.tilt(5)
        #print(v.contains(b))
        pts = []
        for k in self.exterior.coords:
            p = geometry_msgs.msg.Point()
            p.x = k[0]
            p.y = k[1]
            p.z = k[2]

            o = geometry_msgs.msg.Point()
            o.x = self.origin[0]
            o.y = self.origin[1]
            o.z = self.origin[2]
            points_list.points.append(o)
            points_list.points.append(p)
            pts.append(p)

        points_list.points.append(pts[1])
        points_list.points.append(pts[3])

        points_list.points.append(pts[1])
        points_list.points.append(pts[4])

        points_list.points.append(pts[2])
        points_list.points.append(pts[3])

        points_list.points.append(pts[2])
        points_list.points.append(pts[4])
        return points_list




if __name__ == '__main__':
    rospy.init_node('sm_test', anonymous = False)
    b = BinaryPoint([1,0,2])
    width = 0.7
    height = 0.7
    length = 1.5
    origin = [0,0,1.75]
    v = ViewFrustum(origin,[origin,
    (origin[0]+length,origin[1]+width,origin[2]+height),
    (origin[0]+length,origin[1]+-width,origin[2]+-height),
    (origin[0]+length,origin[1]+width,origin[2]+-height),
    (origin[0]+length,origin[1]+(-width),origin[2]+height)])
    target_points_publisher = rospy.Publisher("/view_points", Marker,queue_size=5)

    marker_publisher = rospy.Publisher("/view_planner/candidate_frustrum_geometry", Marker,queue_size=5)
    pose_publisher = rospy.Publisher("/view_planner/candidate_robot_pose", geometry_msgs.msg.PoseStamped,queue_size=5)
    frust_pose_publisher = rospy.Publisher("/view_planner/candidate_frustrum_pose", geometry_msgs.msg.PoseStamped,queue_size=5)

    for k in range(10):
        #m = MarkerArray()
        #v.pan(-3)
        points_list = v.get_visualisation()
        marker_publisher.publish(points_list)
        frust_pose_publisher.publish(v.pose)
        print("in shapley polygon:"+ str(v.contains(b)))
        ich = pnt_in_cvex_hull_2(np.array(v.raw_vertices),np.array([b.x,b.y,b.z]))
        print("in convex hull: " + str(ich))
        centroid_marker = Marker()
        centroid_marker.header.frame_id = "/map"
        centroid_marker.type = Marker.SPHERE
        centroid_marker.header.stamp = rospy.Time.now()
        centroid_marker.pose.position.x = b.x
        centroid_marker.pose.position.y = b.y
        centroid_marker.pose.position.z = b.z
        centroid_marker.scale.x = 0.1
        centroid_marker.scale.y = 0.1
        centroid_marker.scale.z = 0.1
        centroid_marker.color.a = 1.0
        centroid_marker.color.r = 1.0
        centroid_marker.color.g = 0.0
        centroid_marker.color.b = 0.0
        target_points_publisher.publish(centroid_marker)
        #print(m)
        #p_rot = affinity.rotate(p,67,v.origin)
        #print(p_rot)
        rospy.sleep(0.3)
