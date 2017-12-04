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
from deap import base
from deap import creator
from deap import tools
from deap import algorithms
from nav_areas import *
from view_frustrum import *
from random import randint,shuffle
import tf
import math
import copy
from representation import RobotViewState
from representation import ViewFitnessEvaluator
from representation import VoxelMap
import sys
from numpy import (array, dot, arccos, clip)
from numpy.linalg import norm
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import scitos_ptu.msg

class DummyFitness():
    def __init__(self):
        self.values = ()
        self.valid = True

class ViewIndividual(list):
    def __init__(self, *args):
        list.__init__(self, *args)
        #self.fitness = DummyFitness()
        #self.fitness = creator.FitnessMulti
        # use this to store the output of a fitness evaluation #

class ViewSequenceOptimiser():

    def __init__(self,nav_roi,obs_roi,inflation_radius,voxel_map):
        self.nav_generator = NavAreaGenerator(nav_roi,obs_roi,inflation_radius)
        self.successful_init = False
        if(self.nav_generator.successful_init is True):
            pass
        else:
            rospy.logerr("Unable to locate one of the target ROIs.")
            return
        self.nav_area = self.nav_generator.generate_from_soma_roi()
        self.fitness_evaluator = ViewFitnessEvaluator()
        self.voxel_map = voxel_map
        self.successful_init = True
        self.tabu_list = []

    def generate_view(self):
        dv = RobotViewState()
        robot_pose,frust = dv.generate_random(self.nav_area)
        return [frust,robot_pose]

    def mutate_pan(self,ind):
        pan_new = random.randint(-30,30)
        if(pan_new > 30 or pan_new < -30):
            return 0
        #print("PANNING WITH: " + str(pan_new))
        #print("FOR TOTAL: " + str(view[0].pan_angle+pan_new))
        #ind[0].panBy(pan_new)
        #if(ind[0].intersects(self.nav_area.obs_polygon)):
        return pan_new
        #else:
        #    ind[0].pan(-pan_new)
        #    return self.mutate_pan(ind)

    def mutate_tilt(self,ind):
        tilt_new = random.randint(-30,30)
        if(tilt_new > 40 or tilt_new < -40):
            return 0
        return tilt_new


    def mutate_view(self,ind):
        print("mutating view")

        origin = [ind[1].pose.position.x,ind[1].pose.position.y,ind[1].pose.position.z]
        #ind[0].translate(origin)
        #ind[0].reset()
        pa = self.mutate_pan(ind)
        ta = self.mutate_tilt(ind)
        #ind[0].applyPanTilt(pa,ta)
        #ind[0].translate(origin)

        # this is WITHOUT Being translated to the
        # orientation of the pose yet
        # this is why we get weird pans like 9 and it's meant to be 45 or something
        v = ViewFrustum()
        v.translate(origin)
        v.reset()
        #v.panTo(ind[0].init_pose_orientation_pan+pa)
        v.applyPanTilt(ind[0].init_pose_orientation_pan+pa,ind[0].init_pose_orientation_tilt+ta)
        v.translate(origin)

    #    v.reset()
    #    v.panTo(pa)
    #    v.translate(origin)
        v.init_pose_orientation_pan = ind[0].init_pose_orientation_pan+pa
        v.init_pose_orientation_tilt = ind[0].init_pose_orientation_tilt+ta

        v.init_pose_orientation_pan_offset = ind[0].init_pose_orientation_pan_offset+pa
        v.init_pose_orientation_tilt_offset = ind[0].init_pose_orientation_tilt_offset+ta


        ind[0] = v


        return ind


    def mutate_pose(self,ind):
        print("mutating pose")
        mutation_intensity = 4
        print("old position: " + str(ind[1].pose.position))
        nx = ind[1].pose.position.x+random.uniform(-mutation_intensity,mutation_intensity)
        ny = ind[1].pose.position.y+random.uniform(-mutation_intensity,mutation_intensity)
        nz = ind[1].pose.position.z
        (minx, miny, maxx, maxy) = self.nav_area.nav_polygon.bounds
        max_retries = 5
        while(True):
            if(max_retries <= 0):
                #print("FAILURE")
                return ind
            else:
                max_retries-=1
            p = Point(nx,ny)
            if(self.nav_area.nav_polygon.contains(p)):
                add = True
                #print("CIRCLE")
                c = p.buffer(self.nav_area.generator.inflation_radius)
                for k in c.exterior.coords:
                     grid_x = int((k[0] - self.nav_area.generator.costmap.info.origin.position.x) / self.nav_area.generator.costmap.info.resolution)
                     grid_y = int((k[1] - self.nav_area.generator.costmap.info.origin.position.y) / self.nav_area.generator.costmap.info.resolution)
                     costmap_point = self.nav_area.generator.costmap.data[grid_x + self.nav_area.generator.costmap.info.width * grid_y]
                     if(costmap_point != 0):
                        # print("NO GOOD")
                         add = False
                         break
                if(add):
                    break
            else:
                nx = ind[1].pose.position.x+random.uniform(-mutation_intensity,mutation_intensity)
                ny = ind[1].pose.position.y+random.uniform(-mutation_intensity,mutation_intensity)
                nz = ind[1].pose.position.z

        #print("SUCCESS!")
        int_frust_pan = ind[0].pan_angle
        int_frust_tilt = ind[0].tilt_angle

        origin = [nx,ny,1.75]
        ps = geometry_msgs.msg.PoseStamped()
        ps.header.frame_id = "/map"
        ps.pose.position.x = nx
        ps.pose.position.y = ny
        ps.pose.position.z = 1.75

        qt = geometry_msgs.msg.Quaternion()
        yaw = random.uniform(0, 2*math.pi)
        deg = math.degrees(yaw)
        tlt = 0

        # this is just setting up the new view, not mutating the frustum #
        v = ViewFrustum()
        v.translate(origin)
        v.reset()

        v.panTo(deg)
        v.tiltTo(int_frust_tilt)

        v.pan_angle = 0
        v.tilt_angle = 0
        v.init_pose_orientation_pan = deg
        #v.pan(int_frust_pan)

        #roi_hull = self.nav_area.obs_polygon
        #view_hull = Polygon(v.raw_points)
        #print(view_hull)
        #if(roi_hull.intersects(view_hull)):
        #    pass
        #else:
            #print("init view doesn't intersect the obs polygon")
        #    return self.mutate_pose(ind)

        q = list(tf.transformations.quaternion_about_axis(yaw, (0,0,1)))
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]

        accept = self.tabu_check(ps.pose)
        if(accept):
            pass
        else:
            return self.mutate_pose(ind)
        # returns the posestamped and the frsutrum polygon #
        ind[0] = v
        ind[1] = ps
        print("new position: " + str(ind[1].pose.position))
        return ind

    def tabu_check(self,ps):
        for k in self.tabu_list:
            dist = numpy.linalg.norm(np.array([ps.position.x,ps.position.y])-
            np.array([k.position.x,k.position.y]))
            if(dist < (self.nav_generator.inflation_radius)*1.5):
                return False
        return True


    def tabu_register(self,ps):
        self.tabu_list.append(ps)



    def evaluate_view(self,individual):
        return self.fitness_evaluator.evaluate(individual,self.voxel_map,self.prior_view)

    def optimise(self,prior_view,vmap):
        if(not self.successful_init):
            rospy.logerr("System is not prepared. Quitting")
            return
        target_points_publisher = rospy.Publisher("/view_points", Marker,queue_size=5)
        target_points_publisher.publish(self.voxel_map.get_visualisation())
        self.prior_view = prior_view

        marker_publisher = rospy.Publisher("/view_planner/candidate_frustrum_geometry", Marker,queue_size=5)
        pose_publisher = rospy.Publisher("/view_planner/candidate_robot_pose", geometry_msgs.msg.PoseStamped,queue_size=5)
        #frust_pose_publisher = rospy.Publisher("/view_planner/candidate_frustrum_pose", geometry_msgs.msg.PoseStamped,queue_size=5)

        rospy.loginfo("Beginning Genetic Planning")
        CXPB, MUTPB, NGEN, POPSIZE = 0.5, 0.2, 50, 250


        creator.create("FitnessMulti", base.Fitness, weights=(0.6, -0.6, -1.0))
        creator.create("Individual", ViewIndividual, fitness=creator.FitnessMulti)


        toolbox = base.Toolbox()
        rospy.loginfo("Creating Individual Representation")
        toolbox.register("view_creation", self.generate_view)
        toolbox.register("individual", tools.initIterate, creator.Individual, toolbox.view_creation)
        toolbox.register("population", tools.initRepeat, ViewIndividual, toolbox.individual)
        rospy.loginfo("Done")

        rospy.loginfo("Setting Up Operators")
        toolbox.register("evaluate", self.evaluate_view)
        toolbox.register("select", tools.selBest)
        toolbox.register("mutate_view", self.mutate_view)
        toolbox.register("mutate_pose", self.mutate_pose)

        rospy.loginfo("Done")
        rospy.loginfo("Creating initial population and evaluating its fitness")

        pop = toolbox.population(n=POPSIZE)


        # Evaluate the entire population
        fitnesses = list(map(toolbox.evaluate, pop))
        print("INITIAL FITNESSES")
        for ind, fit in zip(pop, fitnesses):
            print(ind)
            ind.fitness.values = fit


        #print("TOP TEN:")
        #top_ten = toolbox.select(pop, 100)
        ##for k in top_ten:
        #    print(k.fitness)

        rospy.loginfo("Done")
        rospy.loginfo("-- Beginning Evolutionary Planning -- ")
        for g in range(NGEN):
            print("-- Generation %i --" % g)

            # Select the next generation individuals
            offspring = toolbox.select(pop, len(pop))
            # Clone the selected individuals
            offspring = list(map(toolbox.clone, offspring))
        #    print("cloned pop:")
        #    for k in offspring:
                #print(k.fitness.values)
        #        print(k.fitness)
            ###################### MUTATION ######################
            for mutant in offspring:
                if random.random() < MUTPB:
                    toolbox.mutate_pose(mutant)
                    toolbox.mutate_view(mutant)
                    #print("deleting")
                    del mutant.fitness.values



            # Evaluate the individuals with an invalid fitness
            invalid_ind = []
            for k in offspring:
                if hasattr(k.fitness, 'values'):
                    pass
                else:
                    invalid_ind.append(k)

            print("INVALID DUDES:" + str(len(invalid_ind)))
            fitnesses = map(toolbox.evaluate, invalid_ind)
            for ind, fit in zip(invalid_ind, fitnesses):
                #print("FIXING UP INVALID DUDE")
                ind.fitness.values = fit
                marker_publisher.publish(ind[0].get_visualisation("blue"))
                pose_publisher.publish(ind[1])
                #frust_pose_publisher.publish(ind[0].pose)
                #rospy.sleep(0.1)

            #    print("NEW FITNESS: " + str(fit))


            print("  Evaluated %i invalid individuals" % len(invalid_ind))

            # The population is entirely replaced by the offspring
            pop[:] = offspring
            best_ind = tools.selBest(pop, 1)[0]
            print("Fitness values for best of this generation: %s" % list(best_ind.fitness.values))
            #pose_publisher.publish(best_ind[1])
            target_points_publisher.publish(self.voxel_map.get_visualisation())
            marker_publisher.publish(best_ind[0].get_visualisation("green"))
            pose_publisher.publish(best_ind[1])
            #frust_pose_publisher.publish(best_ind[0].pose)

            #rospy.sleep(0.5)



        print("-- End of evolution --")
        best_ind = tools.selBest(pop, 1)[0]
        print("FINAL Best individual parameters: %s" % (best_ind))
        best_frust = best_ind[0]
        print("BEST PAN: "+str(best_frust.init_pose_orientation_pan_offset))
        print("BEST TILT: "+str(best_frust.init_pose_orientation_tilt_offset))

        print("publishing goal")


        test_targ_pub = rospy.Publisher('/move_base_simple/goal', geometry_msgs.msg.PoseStamped, queue_size=10)
        rospy.loginfo("Waiting for move_base action server...")

        vmap_cent = vmap.get_centroid()

        print("vmap centroid:"+str(vmap_cent))
        robot_pose = np.asarray([best_ind[1].pose.position.x,best_ind[1].pose.position.y,best_ind[1].pose.position.z])
        magnitude_vmap = np.sqrt(vmap_cent[0]*vmap_cent[0] + vmap_cent[1]*vmap_cent[1] + vmap_cent[2]*vmap_cent[2])
        magnitude_rp = np.sqrt(robot_pose[0]*robot_pose[0] + robot_pose[1]*robot_pose[1] + robot_pose[2]*robot_pose[2])

        angle = np.arccos(np.dot(vmap_cent,robot_pose)/(magnitude_vmap*magnitude_rp))
        #angle = angle*180/np.pi
        new_angle = 30
        angle_diff = 180 - abs(abs(angle - new_angle) - 180);

        print("ANGLE OF BEST VIEW TO VMAP CENTROID: " + str(angle))
        print("ANGLE DIFF: " + str(angle_diff))

        print("Fitness values for these parameters: %s" % list(best_ind.fitness.values))
        for i in range(20):
            pose_publisher.publish(best_ind[1])
            marker_publisher.publish(best_ind[0].get_visualisation("red"))
            #frust_pose_publisher.publish(best_ind[0].pose)
            #rospy.sleep(0.1)

        #print("BEST PAN: "+str(best_frust.init_pose_orientation_pan_offset))
        #print("BEST TILT: "+str(best_frust.init_pose_orientation_tilt_offset))

        # robot pose, pan, tilt
        return best_ind[1].pose,best_frust.init_pose_orientation_pan_offset,best_frust.init_pose_orientation_tilt_offset

def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


class ViewSequenceExecutor():
    def __init__(self,sequence):
        self.sequence = sequence


    def send_movement_goal(self,pose):

        move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose
        move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")
        move_base.send_goal(goal)
        rospy.loginfo("Moving robot...")
        finished_within_time = move_base.wait_for_result(rospy.Duration(120))
        if not finished_within_time:
            move_base.cancel_goal()
            rospy.loginfo("Timed out achieving goal")
        else:
            rospy.loginfo("Goal succeeded!")


    def reset_ptu(self):
        print("attempting to reset PTU")
        ptuClient = actionlib.SimpleActionClient('ResetPtu',scitos_ptu.msg.PtuResetAction)
        ptuClient.wait_for_server()
        print("sending")
        goal = scitos_ptu.msg.PtuResetGoal()
        ptuClient.send_goal(goal)
        ptuClient.wait_for_result()

    def send_ptu_goal(self,pan,tilt):

        #
        #
        # DEBUG, REMOVE LATER
        #
        if(tilt > 40):
            tilt = 40
        #
        # DEBUG, REMOVE LATER
        #
        print("sending ptu goal: " + str(pan) + " : " + str(tilt))
        ptuClient = actionlib.SimpleActionClient('SetPTUState',scitos_ptu.msg.PtuGotoAction)
        ptuClient.wait_for_server()
        print("sending")
        goal = scitos_ptu.msg.PtuGotoGoal()
        goal.tilt = tilt
        goal.tilt_vel = 30
        goal.pan = pan
        goal.pan_vel = 30
        ptuClient.send_goal(goal)
        ptuClient.wait_for_result()

    def do_perception(self):
        print("PERCEIVING!")
        rospy.sleep(10)


    def execute(self):
        print("executing sequence of:" + str(len(self.sequence)) + " views")
        for view in self.sequence:
            print("---- VIEW ----")
            #print("VIEW ANGLE: " + str(view[3]))
            print(view)
            pose = view[0]
            pan = view[1]
            tilt = view[2]
            self.reset_ptu()
            self.send_movement_goal(pose)
            self.send_ptu_goal(pan,tilt)
            self.do_perception()
            self.reset_ptu()
            print("--- COMPLETE ---")
            rospy.sleep(5)


class ViewSequenceController():
    def __init__(self,nav_roi_id,observation_roi_id,num_views,target_map):
        if(target_map is None):
            # generate a dummy map for debugging
            self.generate_dummy_target_map()
        else:
            self.generate_target_map_from_input(target_map)

        self.optimiser = ViewSequenceOptimiser(nav_roi=nav_roi_id,obs_roi=observation_roi_id,inflation_radius=0.4,voxel_map=self.vmap)

        last_view = None
        view_sequence = []
        for view_id in range(num_views):
            new_view = self.optimiser.optimise(last_view,self.vmap)
            self.optimiser.tabu_register(new_view[0])
            last_view = new_view
            view_sequence.append(new_view)

        view_sequence = self.order_view_by_distance_travelled(view_sequence)

        self.executor = ViewSequenceExecutor(view_sequence)
        self.executor.execute()


    def trav_dist(self,a,b):
            print("TRAV DIST:")
            print(a)
            dist = numpy.linalg.norm(np.array([a[0].position.x,a[0].position.y])-np.array([b[0].position.x,b[0].position.y]))
            print("DISTANCE:" + str(dist))
            return dist

    def measure_dist_travelled(self,candidate_views):
        a_to_b = self.trav_dist(candidate_views[0],candidate_views[1])
        b_to_c = self.trav_dist(candidate_views[1],candidate_views[2])
        return a_to_b+b_to_c


        #
        # this is a giant hack but i have to leave in 10 minutes so lets
        # just see if it works and then fix it up later.
        # TODO: MAKE THIS BETTER
        #
    def order_view_by_distance_travelled(self,views):
        tabu = []
        best_permutation = None
        best_perm_val = 100000
        permuted_views = views
        print("ORDERING THESE VIEWS BY DIST")
        print(views[0])
        print("READY!")
        for i in range(100):
            random.shuffle(permuted_views)
            print("PERMUTED:")
            print(permuted_views)
            if(permuted_views in tabu):
                continue
            tabu.append(permuted_views)
            perm_val = self.measure_dist_travelled(permuted_views)
            if(perm_val < best_perm_val):
                best_perm_val = perm_val
                best_permutation = permuted_views
        return best_permutation

    def order_views_by_angle(self,views):
        ns = []
        for v in views:
            view_pose = np.asarray([v[0].position.x,v[0].position.y])
            vmap_cent = np.asarray([self.vmap.get_centroid()[0],self.vmap.get_centroid()[1]])
            print("VIEW POSE:"+str(view_pose))
            print("CENT POSE: "+str(vmap_cent))
            a = angle_between(view_pose,vmap_cent)
            angle1 = np.rad2deg(a)
            print("ANGLE IN RADS:"+str(a))
            l = list(v)
            l.append(a)
            nv = tuple(l)
            print("NEW TUPLE:")
            print(nv)
            ns.append(nv)

        newviews = sorted(ns, key=lambda x: x[2], reverse=False)

        print("VIEW ORDER:")
        for k in newviews:
            print("POSITION:"+str(k[0]))
            print("ORIENTATION:"+str(k[1]))
            print("ANGLE:"+str(k[3]))

        return newviews


    def generate_target_map_from_input(self,map):
        self.vmap = VoxelMap()
        # do some things
        self.vmap.calc_centroid()

    def generate_dummy_target_map(self):
        self.vmap = VoxelMap()
        #tum kitchen
        #self.vmap.generate_dummy([1.121,-1.564,0.9])
        #self.vmap.generate_dummy([0.845,-2.106,0.9])
        #self.vmap.generate_dummy([0.845,-1.406,0.9])
        # aloof env
        self.vmap.generate_dummy([8.78944,3.57644,0.9]) # mouse
        self.vmap.generate_dummy([8.96862,3.11941,0.9]) # keyboard
        self.vmap.generate_dummy([8.79891,2.58882,0.9]) # mug
        self.vmap.generate_dummy([9.2274,3.08814,0.9]) # mug
        self.vmap.calc_centroid()


if __name__ == '__main__':
    rospy.init_node('sm_test', anonymous = False)

#    vmap = VoxelMap()
    #tum kitchen
    #vmap.generate_dummy([1.121,-1.564,0.9])
    #vmap.generate_dummy([0.845,-2.106,0.9])
    #vmap.generate_dummy([0.845,-1.406,0.9])
    # aloof env
#    vmap.generate_dummy([8.78944,3.57644,0.9]) # mouse
#    vmap.generate_dummy([8.96862,3.11941,0.9]) # keyboard
#    vmap.generate_dummy([8.79891,2.58882,0.9]) # mug
#    vmap.generate_dummy([9.2274,3.08814,0.9]) # mug
#    vmap.calc_centroid()

    v = ViewSequenceController("1","2",3,None)




#    while(True):
#        msg = rospy.wait_for_message("/clicked_point", geometry_msgs.msg.PointStamped , timeout=65.0)
#        robot_pose = np.asarray([msg.point.x,msg.point.y])
#        vmap_cent = np.asarray([0.937,-1.692])
#
#        angle1 = angle_between(vmap_cent,robot_pose)
#        print("RADS ANGLE 1: " + str(angle1))
#        angle1 = np.rad2deg(angle1)
#        print("DEG ANGLE 1: " + str(angle1))
