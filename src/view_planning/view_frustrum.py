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
        self.raw_vertices = vertices
        self.origin = origin
        self.init_tilt_angle = default_tilt_angle
        self.init_pan_angle = default_pan_angle
        self.pan(default_pan_angle)
        self.tilt(default_tilt_angle)
        self.pan_angle = 0
        self.tilt_angle = 0

    def __deepcopy__(self, memo):
        return ViewFrustum(copy.deepcopy(self.origin),copy.deepcopy(self.raw_vertices),copy.deepcopy(self.init_pan_angle),copy.deepcopy(self.init_tilt_angle))

    def pan(self,angle):
        new_coords = []
        self.pan_angle = angle
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
        super(ViewFrustum,self).__init__(new_coords)

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

    def get_visualisation(self):
        points_list = Marker()
        points_list.header.frame_id = "/map"
        points_list.type = Marker.LINE_LIST
        points_list.scale.x = 0.05
        points_list.scale.y = 0.1
        points_list.scale.z = 0.1
        points_list.color.a = 1.0
        points_list.color.r = 0.0
        points_list.color.g = 0.0
        points_list.color.b = 1.0
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
    b = BinaryPoint([1,0,1.75])
    width = 0.7
    height = 0.7
    length = 1.5
    origin = [0,0,1.75]
    v = ViewFrustum(origin,[origin,
    (origin[0]+length,origin[1]+width,origin[2]+height),
    (origin[0]+length,origin[1]+-width,origin[2]+-height),
    (origin[0]+length,origin[1]+width,origin[2]+-height),
    (origin[0]+length,origin[1]+(-width),origin[2]+height)])

    marker_publisher = rospy.Publisher("/frust_points", Marker,queue_size=5)
    targ_publisher = rospy.Publisher("/targ_points", Marker,queue_size=5)

    for k in range(10):
        #m = MarkerArray()
        v.pan(3)
        points_list = v.get_visualisation()
        marker_publisher.publish(points_list)
        print(v.contains(b))

    #    centroid_marker = Marker()
    #    centroid_marker.header.frame_id = "/map"
    #    centroid_marker.type = Marker.SPHERE
    #    centroid_marker.header.stamp = rospy.Time.now()
    #    centroid_marker.pose.position.x = b.x
    #    centroid_marker.pose.position.y = b.y
    #    centroid_marker.pose.position.z = b.z
    #    centroid_marker.scale.x = 0.1
    #    centroid_marker.scale.y = 0.1
    #    centroid_marker.scale.z = 0.1
    #    centroid_marker.color.a = 1.0
    #    centroid_marker.color.r = 1.0
    #    centroid_marker.color.g = 0.0
    #    centroid_marker.color.b = 0.0
    #    targ_publisher.publish(centroid_marker)
        #print(m)
        #p_rot = affinity.rotate(p,67,v.origin)
        #print(p_rot)
        rospy.sleep(1)
