import random

from deap import base
from deap import creator
from deap import tools
from deap import algorithms

import numpy as np

def evaluate(ind):
    # Do some hard computing on the individual
    targ = [50,22,7]
    dist_x = abs(targ[0]-ind[0])
    dist_y = abs(targ[1]-ind[1])
    dist_z = abs(targ[2]-ind[2])
    return dist_x,dist_y,dist_z

class ViewIndividual(list):
    def __init__(self, *args):
        list.__init__(self, *args)

        # fitness of individual #
        # number of points visible in view 1 (COVERAGE/POSSIBLE COVERAGE)
        # distance of view 1 to centroid of octomap
        # nav distance from initial state to view 1

        # number of points visible in view 2 (COVERAGE/POSSIBLE COVERAGE)
        # distance of view 2 to centroid of octomap
        # nav distance from view 1 to view 2

        # number of points visible in view 3 (COVERAGE/POSSIBLE COVERAGE)
        # distance of view 3 to centroid of octomap
        # nav distance from view 2 to view 3


if __name__ == '__main__':

    IND_SIZE = 3
    CXPB, MUTPB, NGEN, POPSIZE = 0.5, 0.2, 250, 100

    creator.create("FitnessMulti", base.Fitness, weights=(-1.0, -1.0, -1.0))
    creator.create("Individual", ViewIndividual, fitness=creator.FitnessMulti)

    toolbox = base.Toolbox()
    toolbox.register("attr_float", random.random)
    toolbox.register("individual", tools.initRepeat, creator.Individual, toolbox.attr_float, n=IND_SIZE)
    toolbox.register("population", tools.initRepeat, ViewIndividual, toolbox.individual)

    toolbox.register("evaluate", evaluate)
    toolbox.register("mutate", tools.mutGaussian, mu=0, sigma=1, indpb=0.1)
    toolbox.register("mate", tools.cxUniform, indpb=CXPB)
    toolbox.register("select", tools.selTournament, tournsize=3)

    pop = toolbox.population(n=POPSIZE)

    # Evaluate the entire population
    fitnesses = list(map(toolbox.evaluate, pop))
    for ind, fit in zip(pop, fitnesses):
        ind.fitness.values = fit

    print("  Evaluated %i individuals" % len(pop))

    for g in range(NGEN):
        print("-- Generation %i --" % g)

        # Select the next generation individuals
        offspring = toolbox.select(pop, len(pop))
        # Clone the selected individuals
        offspring = list(map(toolbox.clone, offspring))

        # Apply crossover and mutation on the offspring
        for child1, child2 in zip(offspring[::2], offspring[1::2]):

            # cross two individuals with probability CXPB
            if random.random() < CXPB:
                toolbox.mate(child1, child2)

                # fitness values of the children
                # must be recalculated later
                del child1.fitness.values
                del child2.fitness.values

        for mutant in offspring:

            # mutate an individual with probability MUTPB
            if random.random() < MUTPB:
                toolbox.mutate(mutant)
                del mutant.fitness.values

        # Evaluate the individuals with an invalid fitness
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit

        print("  Evaluated %i individuals" % len(invalid_ind))

        # The population is entirely replaced by the offspring
        pop[:] = offspring

        # Gather all the fitnesses in one list and print the stats
    #    fits = [ind.fitness.values[0] for ind in pop]

    #    length = len(pop)
    #    mean = sum(fits) / length
    #    sum2 = sum(x*x for x in fits)
    #    std = abs(sum2 / length - mean**2)**0.5

    #    print("  Min %s" % min(fits))
    #    print("  Max %s" % max(fits))
    #    print("  Avg %s" % mean)
    #    print("  Std %s" % std)

    print("-- End of (successful) evolution --")
    best_ind = tools.selBest(pop, 1)[0]
    print("Best individual parameters: %s" % (best_ind))
    print("Fitness values for these parameters: %s" % list(best_ind.fitness.values))
