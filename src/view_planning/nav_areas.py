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
from nav_msgs.msg import OccupancyGrid

class NavArea():
    def __init__(self,nav_poly,obs_poly,generator):
        self.generator = generator
        self.nav_polygon = nav_poly
        self.obs_polygon = obs_poly

    def get_random_points_in_area(self,num=1):
     (minx, miny, maxx, maxy) = self.nav_polygon.bounds
     pts = []
     for k in range(num):
         while True:
             p = Point(random.uniform(minx, maxx), random.uniform(miny, maxy))
             c = p.buffer(self.generator.inflation_radius)
             if self.nav_polygon.contains(c):
                add = True
                #print("CIRCLE")
                for k in c.exterior.coords:
                     grid_x = int((k[0] - self.generator.costmap.info.origin.position.x) / self.generator.costmap.info.resolution)
                     grid_y = int((k[1] - self.generator.costmap.info.origin.position.y) / self.generator.costmap.info.resolution)
                     costmap_point = self.generator.costmap.data[grid_x + self.generator.costmap.info.width * grid_y]
                     #print(costmap_point)
                     if(costmap_point != 0):
                         add = False
                         break
                if(add):
                    pts.append(p)
                    break
     return pts


class NavAreaGenerator():
    def __init__(self,nav_roi_id,observation_roi_id,inflation_radius):
        rospy.loginfo("Created Nav Area Generator with inflation radius " + str(inflation_radius))
        self.nav_roi_id = nav_roi_id
        self.observation_roi_id = observation_roi_id
        self.inflation_radius = inflation_radius
        soma_query = rospy.ServiceProxy('soma/query_rois',SOMAQueryROIs)

        query = SOMAQueryROIsRequest()
        query.returnmostrecent = True
        query.roiids = [self.nav_roi_id]
        response = soma_query(query)
        self.nav_roi = response.rois[-1]

        query = SOMAQueryROIsRequest()
        query.returnmostrecent = True
        query.roiids = [self.observation_roi_id]
        response = soma_query(query)
        self.observation_roi = response.rois[-1]

        rospy.loginfo("Getting Map")
        msg = rospy.wait_for_message("/move_base/global_costmap/costmap", OccupancyGrid , timeout=10.0)
        self.costmap = msg

    # gets a soma roi, turns it into a polygon and then a navarea #

    def get_polygon(self,roi):
        points = roi.posearray.poses
        points_2d = []
        for point in points:
            points_2d.append([point.position.x,point.position.y])
        if(len(points_2d) <= 2):
            rospy.logerr("SOMa Region isn't a polygon, aborting!")
            return None
        polygon = Polygon(points_2d)
        return polygon

    def generate_from_soma_roi(self):
        nav_polygon = self.get_polygon(self.nav_roi)
        obs_polygon = self.get_polygon(self.observation_roi)

        na = NavArea(nav_polygon,obs_polygon,self)
        na.nav_roi_id = self.nav_roi_id
        na.obs_roi_id = self.observation_roi_id
        return na

if __name__ == '__main__':

    #
    n = NavAreaGenerator("2","1")
    na = n.generate_from_soma_roi()
    print(na.get_random_points_in_area()[0])
