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
from random import randint
import tf
import math

from representation import RobotViewState
from representation import ViewFitnessEvaluator
from representation import VoxelMap

class ViewIndividual(list):
    def __init__(self, *args):
        list.__init__(self, *args)
        # use this to store the output of a fitness evaluation #

class ViewSequenceOptimiser():
    def __init__(self,target_roi,inflation_radius,voxel_map):
        self.nav_generator = NavAreaGenerator(target_roi,inflation_radius)
        self.nav_area = self.nav_generator.generate_from_soma_roi()
        self.fitness_evaluator = ViewFitnessEvaluator()
        self.voxel_map = voxel_map

    def generate_view(self):
        dv = RobotViewState()
        robot_pose,frust = dv.generate_random(self.nav_area)

        # flattens the view into its important components #
        # namely a position,orientation and a pan/tilt for the ptu #
        # packed into a list of floats #
        ind = ViewIndividual()
        # first three components: x,y,z world pos
        #ind.append(robot_pose.pose.position.x)
        #ind.append(robot_pose.pose.position.y)
        #ind.append(robot_pose.pose.position.z)

        # next four components: x,y,z,w Quaternion rotation
        #ind.append(robot_pose.pose.orientation.x)
        #ind.append(robot_pose.pose.orientation.y)
        #ind.append(robot_pose.pose.orientation.z)
        #ind.append(robot_pose.pose.orientation.w)


        # final two components, pan and tilt for ptu
        #ind.append(frust.pan_angle)
        #ind.append(frust.tilt_angle)
        # these will be 0 and 0, because they just face the way the
        # orientation is facing for now
        # can mutate them later i guess? #
        data = {}
        data['frust'] = frust
        data['pose'] = robot_pose
        ind.append(data)
        return ind

    def evaluate_view(self,individual):
        return self.fitness_evaluator.evaluate(individual,self.voxel_map)

    def optimise(self):
        rospy.loginfo("Beginning Genetic Planning")
        CXPB, MUTPB, NGEN, POPSIZE = 0.5, 0.2, 250, 100
        creator.create("FitnessMulti", base.Fitness, weights=(1.0, -1.0))
        creator.create("Individual", ViewIndividual, fitness=creator.FitnessMulti)
        toolbox = base.Toolbox()
        rospy.loginfo("Creating Individual Representation")
        toolbox.register("view_creation", self.generate_view)
        toolbox.register("individual", tools.initIterate, ViewIndividual, toolbox.view_creation)
        toolbox.register("population", tools.initRepeat, ViewIndividual, toolbox.individual)
        rospy.loginfo("Done")

        rospy.loginfo("Setting Up Operators")
        toolbox.register("evaluate", self.evaluate_view)
        toolbox.register("select", tools.selTournament, tournsize=3)
        rospy.loginfo("Done")
        rospy.loginfo("Creating initial population and evaluating its fitness")
        pop = toolbox.population(n=POPSIZE)
        # Evaluate the entire population
        fitnesses = list(map(toolbox.evaluate, pop))
        for ind, fit in zip(pop, fitnesses):
            ind.fitness.values = fit


if __name__ == '__main__':
    rospy.init_node('sm_test', anonymous = False)
    #n = NavAreaGenerator("2",0.4)
    #na = n.generate_from_soma_roi()
    vmap = VoxelMap()
    vmap.generate_dummy([1.022,-2.046,1.6])


    optimiser = ViewSequenceOptimiser("2",0.4,vmap)
    optimiser.optimise()


#    view_root_publisher = rospy.Publisher("/view_points", Marker,queue_size=5)
#    marker_publisher = rospy.Publisher("/frust_points", Marker,queue_size=5)
#    pose_publisher = rospy.Publisher("/frust_pose", geometry_msgs.msg.PoseStamped,queue_size=5)
