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
import copy
from representation import RobotViewState
from representation import ViewFitnessEvaluator
from representation import VoxelMap
import sys

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

    def __init__(self,target_roi,inflation_radius,voxel_map):
        self.nav_generator = NavAreaGenerator(target_roi,inflation_radius)
        self.nav_area = self.nav_generator.generate_from_soma_roi()
        self.fitness_evaluator = ViewFitnessEvaluator()
        self.voxel_map = voxel_map

    def generate_view(self):
        dv = RobotViewState()
        robot_pose,frust = dv.generate_random(self.nav_area)
        return [frust,robot_pose]

    def mutate_view(self,view):
        pan_new = random.randint(-45,45)
        if((view[0].pan_angle+pan_new) > 45 or (view[0].pan_angle+pan_new) < -45):
            return view
        print("PANNING WITH: " + str(pan_new))
        print("FOR TOTAL: " + str(view[0].pan_angle+pan_new))
        view[0].pan(pan_new)
        return view

    def evaluate_view(self,individual):
        return self.fitness_evaluator.evaluate(individual,self.voxel_map)

    def optimise(self):

        target_points_publisher = rospy.Publisher("/view_points", Marker,queue_size=5)
        target_points_publisher.publish(self.voxel_map.get_visualisation())

        marker_publisher = rospy.Publisher("/frust_points", Marker,queue_size=5)
        pose_publisher = rospy.Publisher("/frust_pose", geometry_msgs.msg.PoseStamped,queue_size=5)

        rospy.loginfo("Beginning Genetic Planning")
        CXPB, MUTPB, NGEN, POPSIZE = 0.5, 0.2, 250, 500


        creator.create("FitnessMulti", base.Fitness, weights=(1.0, -1.0, -1.0))
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
        toolbox.register("mutate", self.mutate_view)
        rospy.loginfo("Done")
        rospy.loginfo("Creating initial population and evaluating its fitness")

        pop = toolbox.population(n=POPSIZE)
        # Evaluate the entire population
        fitnesses = list(map(toolbox.evaluate, pop))
        print("INITIAL FITNESSES")
        for ind, fit in zip(pop, fitnesses):
            print(ind)
            ind.fitness.values = fit


        print("TOP TEN:")
        top_ten = toolbox.select(pop, 100)
        for k in top_ten:
            print(k.fitness)

        rospy.loginfo("Done")
        rospy.loginfo("-- Beginning Evolutionary Planning -- ")
        for g in range(NGEN):
            print("-- Generation %i --" % g)

            # Select the next generation individuals
            offspring = toolbox.select(pop, len(pop))
            # Clone the selected individuals
            offspring = list(map(toolbox.clone, offspring))
            print("cloned pop:")
            for k in offspring:
                #print(k.fitness.values)
                print(k.fitness)
            ###################### MUTATION ######################
            for mutant in offspring:
                if random.random() < MUTPB:
                    toolbox.mutate(mutant)
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
            #    print("NEW FITNESS: " + str(fit))


            print("  Evaluated %i invalid individuals" % len(invalid_ind))

            # The population is entirely replaced by the offspring
            pop[:] = offspring
            best_ind = tools.selBest(pop, 1)[0]
            print("Fitness values for best of this generation: %s" % list(best_ind.fitness.values))
            pose_publisher.publish(best_ind[1])

            marker_publisher.publish(best_ind[0].get_visualisation())
            target_points_publisher.publish(self.voxel_map.get_visualisation())

            #rospy.sleep(0.5)



        print("-- End of evolution --")
        best_ind = tools.selBest(pop, 1)[0]
        print("FINAL Best individual parameters: %s" % (best_ind))
        best_frust = best_ind[0]
        print("BEST PAN: "+str(best_frust.pan_angle))

        print("Fitness values for these parameters: %s" % list(best_ind.fitness.values))
        pose_publisher.publish(best_ind[1])
        for i in range(20):
            marker_publisher.publish(best_ind[0].get_visualisation(False))
            rospy.sleep(0.5)

if __name__ == '__main__':
    rospy.init_node('sm_test', anonymous = False)

    vmap = VoxelMap()
    vmap.generate_dummy([1.222,-2.046,1.6])
    vmap.generate_dummy([1.121,-1.564,1.6])
    vmap.generate_dummy([1.263,-2.163,1.6])
#    creator.create("FitnessMulti", base.Fitness, weights=(1.0, -1.0))


    #print(f.values)
    #print(a.values)

    optimiser = ViewSequenceOptimiser("2",0.4,vmap)
    optimiser.optimise()
