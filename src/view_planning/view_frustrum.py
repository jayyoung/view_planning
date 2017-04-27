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
import tf2_msgs
import math
import geometry_msgs
import sys
import numpy
import numpy.random
from numpy import zeros, ones, arange, asarray, concatenate
from scipy.optimize import linprog
import os
from scipy.spatial import ConvexHull
import random
from random import randint
import math3d as m3d


class BinaryPoint(shapely.geometry.Point):
    def __init__(self,pos):
        super(BinaryPoint,self).__init__(pos)
        self.visit_counter = 0

    def visited(self):
        return self.visit_counter == 0

class OldViewFrustum(shapely.geometry.Polygon):

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
        self.tilt_angle += angle
        #print("b"+str(self.raw_vertices[0]))

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
        #new_coords[0] = self.origin
        self.raw_vertices = new_coords
        #print("a" + str(self.raw_vertices[0]))

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


class ViewFrustum():
    def __init__(self):
        #self.ntl,self.ntr,self.nbl,self.nbr,self.ftl,self.ftr,self.fbl,self.fbr = Point()
        #self.near_d,self.far_d,self.ratio,self.angle,self.tang = 0
        #self.nw,self.nh,self.fw,self.fh = 0
        #self.value = 0
        self.origin = [0,0,1.75]

        self.position = [0,0,1.75]

        self.structure = {}

        self.pan_angle = 0

        self.tilt_angle = 0
        self.reset()

    def reset(self):
        frustum_near = 0.8;
        frustum_far = 2.5;
        frustum_angle = 40.5;
        frustum_ratio = 1.333;

        p = np.array([0.0,0.0,0.0])
        l = np.array([1.0,0.0,0.0])
        u = np.array([0.0,0.0,1.0])
        self.setCamInternals(frustum_angle,frustum_ratio,frustum_near,frustum_far)
        self.setUpPlanesFromPose(p,l,u)

    def point_in_hull(self,pnt):
        '''
        Given a set of points that defines a convex hull, uses simplex LP to determine
        whether point lies within hull.
        `hull_points` -- (N, 3) array of points defining the hull
        `pnt` -- point array of shape (3,)
        '''
        hull_points = np.array(self.raw_points)
        N = hull_points.shape[0]
        c = ones(N)
        A_eq = concatenate((hull_points, ones((N,1))), 1).T   # rows are x, y, z, 1
        b_eq = concatenate((pnt, (1,)))
        result = linprog(c, A_eq=A_eq, b_eq=b_eq)
        if result.success and c.dot(result.x) == 1.:
            return True
        return False


    def setCamInternals(self,angle,ratio,near_d,far_d):
        self.ratio = ratio
        self.angle = angle
        self.near_d = near_d
        self.far_d = far_d
        ANG2RAD = 3.14159265358979323846/180.0

        self.tang = math.tan(self.angle * ANG2RAD * 0.5)
        self.nh = self.near_d * self.tang
    	self.nw = self.nh * self.ratio
    	self.fh = self.far_d  * self.tang
    	self.fw = self.fh * self.ratio

    def normalise(self,vector):
        #print("starting")
        #print(vector)
        length = math.sqrt(vector[0]*vector[0]+vector[1]*vector[1]+vector[2]*vector[2])
        #length = len(vector)
        #print(length)
        #print("normalising..")
        if(length):
            vector[0] /= length
            vector[1] /= length
            vector[2] /= length
        #print(vector)
        return np.array(vector)


    def setUpPlanesFromPose(self,p,l,u):
        #z = p-l
        z = [0.0,0.0,0.0]
        z[0] = p[0]-l[0]
        z[1] = p[1]-l[1]
        z[2] = p[2]-l[2]
        z = self.normalise(z)
        #print(z)

        #x = u * z
        #print("NP RESULT:"+str(x))
        x = mul(u,z)
        x = self.normalise(x)
        x = np.array(x)

        #y = z * x
        y = mul(z,x)
        y = np.array(y)

        nc = p-z*self.near_d
        fc = p - z * self.far_d

        ntl = nc + y * self.nh - x * self.nw
        ntr = nc + y * self.nh + x * self.nw

        nbl = nc - y * self.nh - x * self.nw
        nbr = nc - y * self.nh + x * self.nw

        ftl = fc + y * self.fh - x * self.fw
        ftr = fc + y * self.fh + x * self.fw

        fbl = fc - y * self.fh - x * self.fw
        fbr = fc - y * self.fh + x * self.fw

        self.raw_points = [ntl,ntr,nbl,nbr,ftl,ftr,fbl,fbr]
        self.points = []
        self.points.append(vec2point(ntl))
        self.points.append(vec2point(ntr))

        self.points.append(vec2point(nbl))
        self.points.append(vec2point(nbr))

        self.points.append(vec2point(ftl))
        self.points.append(vec2point(ftr))

        self.points.append(vec2point(fbl))
        self.points.append(vec2point(fbr))

        # these are PLANES
        self.structure['TOP'] = np.array([ntr,ntl,ftl])
        self.structure['BOTTOM'] = np.array([nbl,nbr,fbr])
        self.structure['LEFT'] = np.array([ntl,nbl,fbl])

        self.structure['RIGHT'] = np.array([nbr,ntr,fbr])
        self.structure['NEARP'] = np.array([ntl,ntr,nbr])
        self.structure['FARP'] = np.array([ftr,ftl,fbl])

    def pan(self,angle):
        self.reset()

        r = m3d.Orientation.new_axis_angle([0,0,1],math.radians(angle))
        new_points = []
        for k in self.points:
            v = m3d.Vector(k.x,k.y,k.z)
            ne = r * v
            k.x = ne[0]
            k.y = ne[1]
            k.z = ne[2]
            new_points.append(np.array([ne[0],ne[1],ne[2]]))
        self.pan_angle = angle
        self.raw_points = new_points

        self.translate(self.position)

    def tilt(self,angle):
        self.reset()

        r = m3d.Orientation.new_axis_angle([0,1,0],math.radians(angle))
        new_points = []
        for k in self.points:
            v = m3d.Vector(k.x,k.y,k.z)
            ne = r * v
            k.x = ne[0]
            k.y = ne[1]
            k.z = ne[2]
            new_points.append(np.array([ne[0],ne[1],ne[2]]))
        self.tilt_angle = angle
        self.raw_points = new_points

        self.translate(self.position)

    def translate(self,pos):
        if(pos != self.position):
            self.reset()
        self.position = pos
        new_points = []
        for k in self.points:
            #print("PRE TRANSFORM: " + str(k))
            k.x = k.x+pos[0]
            k.y = k.y+pos[1]
            k.z = k.z+pos[2]
            new_points.append(np.array([k.x,k.y,k.z]))
        self.raw_points = new_points
            #print("POST TRANSFORM: " + str(k))

    def get_visualisation(self,colour="blue"):
        marker1 = Marker()
        marker1.header.frame_id = "/map"
        marker1.type = marker1.LINE_LIST
        marker1.action = marker1.ADD
        marker1.scale.x = 0.05
        marker1.color.a = 1

        if(colour is "blue"):
            marker1.color.r = 0.0
            marker1.color.g = 0.0
            marker1.color.b = 1.0
        if(colour is "red"):
            marker1.color.r = 1.0
            marker1.color.g = 0.0
            marker1.color.b = 0.0
        if(colour is "green"):
            marker1.color.r = 0.0
            marker1.color.g = 1.0
            marker1.color.b = 0.0

    #    marker1.pose.orientation = pose.orientation
    #    marker1.pose.position = pose.position

        marker1.points.append(self.points[0])
        marker1.points.append(self.points[1])

        marker1.points.append(self.points[2])
        marker1.points.append(self.points[3])

        marker1.points.append(self.points[0])
        marker1.points.append(self.points[2])

        marker1.points.append(self.points[1])
        marker1.points.append(self.points[3])

        marker1.points.append(self.points[4])
        marker1.points.append(self.points[5])

        marker1.points.append(self.points[6])
        marker1.points.append(self.points[7])

        marker1.points.append(self.points[4])
        marker1.points.append(self.points[6])

        marker1.points.append(self.points[5])
        marker1.points.append(self.points[7])

        marker1.points.append(self.points[0])
        marker1.points.append(self.points[4])

        marker1.points.append(self.points[2])
        marker1.points.append(self.points[6])

        marker1.points.append(self.points[1])
        marker1.points.append(self.points[5])

        marker1.points.append(self.points[3])
        marker1.points.append(self.points[7])

        return marker1

def mul(a,b):
    x = [0.0,0.0,0.0]
    x[0] = a[1]*b[2]-a[2]*b[1]
    x[1] = a[2]*b[0]-a[0]*b[2]
    x[2] = a[0]*b[1]-a[1]*b[0]
    return x


def vec2point(point):
    p = geometry_msgs.msg.Point()
    p.x = point[0]
    p.y = point[1]
    p.z = point[2]
    return p


if __name__ == '__main__':
    rospy.init_node('sm_test', anonymous = False)

    pubfrustum = rospy.Publisher('/frustums', Marker,queue_size=25)

    b = BinaryPoint([1,0,2])

    origin = [0,0,1.75]
    target = [0,7,1.75]
    v = ViewFrustum()
    v.translate(target)

    target_points_publisher = rospy.Publisher("/view_points", Marker,queue_size=5)

    marker_publisher = rospy.Publisher("/view_planner/candidate_frustrum_geometry", Marker,queue_size=5)
    pose_publisher = rospy.Publisher("/view_planner/candidate_robot_pose", geometry_msgs.msg.PoseStamped,queue_size=5)
    frust_pose_publisher = rospy.Publisher("/view_planner/candidate_frustrum_pose", geometry_msgs.msg.PoseStamped,queue_size=5)
    print("ORIGIN:" + str(origin))
    pan_amt = 1
    for k in range(20):
        #m = MarkerArray()
        v.pan(pan_amt)
        pan_amt += 2
        #v.translate(target)
        #v.pan(2)
        points_list = v.get_visualisation()
        marker_publisher.publish(points_list)
    #    frust_pose_publisher.publish(v.pose)
        print("in shapley polygon:"+ str(v.point_in_hull(np.array([1,0,2]))))
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
