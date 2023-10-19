import random
import numpy as np
import copy
from Utility.Tools import Tools


class Algorithm():

    def __init__(self, generation=1, mutation=0.1, crossover=0.01, criticalvalue=10, adjust_points=None,
                 range_mutation=None, weights=None, must_mutate_angle=0):
        self.generation = generation
        self.mutation = mutation
        self.crossover = crossover
        self.range_mutation = range_mutation
        self.step_size = self.range_mutation['Z1']

        self.criticalvalue = criticalvalue
        self.adjust_points = adjust_points

        self.weights = weights
        self.weight = self.weights['W1']

        self.must_mutate_angle = must_mutate_angle

        self.bestfitness = {'generation': 0, 'individual': None}
        self.bestfitness_gens = []
        self.transition_individual = {'generation': 0, 'individual': None}

        self.temp_new_point = None

    def optimize_with_strategy(self, population, obsticle, safety_radius):
        """
        start optimaze the walking path
        :param population:
        :param obsticle:
        :param safety_radius:
        :return:
        """
        add_flag = True
        add_index = 1

        for gen in range(self.generation):
            # calculate the fitness
            for individual in population.values:
                individual.fitness_Distance()
                individual.fitness_Smoothness()
                individual.combined_fitness(self.weight['a'], self.weight['b'])

            if gen != -1:
                sorted_individuals = sorted(population.values, key=lambda e: e.overal_fitness,
                                            reverse=True)  # 根据fitness将population排序

                if self.bestfitness['individual'] is None:
                    self.bestfitness['individual'] = copy.deepcopy(sorted_individuals[0])
                    self.bestfitness['generation'] = gen
                else:
                    if self.bestfitness['individual'].overal_fitness < sorted_individuals[0].overal_fitness:
                        self.bestfitness['individual'] = copy.deepcopy(sorted_individuals[0])
                        self.bestfitness['generation'] = gen
                        add_index = 1
                    else:
                        add_index += 1

                if (gen + 1) % 50 == 0 or gen == 0:
                    print(
                        f'Until gen-{gen + 1}, the best one, overal fitness = {round(self.bestfitness["individual"].overal_fitness, 8)}, fitness dis = {round(self.bestfitness["individual"].fitness_dis, 8)}, fitness smooth = {round(self.bestfitness["individual"].fitness_smooth, 8)}')

                self.bestfitness['individual'].add_points_inPath(self.adjust_points['N1'])
                self.bestfitness_gens.append(
                    {'generation': gen, 'individual': copy.deepcopy(self.bestfitness['individual'])})

                # optimize the smoothness
                if add_index > self.criticalvalue:
                    # break
                    self.bestfitness['individual'].add_points_inPath(self.adjust_points['N2'])

                    if add_flag is True:
                        self.weight = self.weights['W2']

                        self.step_size = self.range_mutation['Z2']

                        self.transition_individual['generation'] = gen
                        self.transition_individual['individual'] = copy.deepcopy(self.bestfitness['individual'])

                        # overall fitness reset
                        self.bestfitness['individual'].overal_fitness = 0
                        self.bestfitness['generation'] = 0
                        add_flag = False

                selected_parents = [self.bestfitness['individual']]  # pick up the best fitness parents

                new_generation = []  # New Generation
                while 1:
                    if len(new_generation) > len(population.values):
                        new_generation.pop()
                        continue
                    elif len(new_generation) == len(population.values):
                        break
                    else:
                        copy_selected_parents = copy.deepcopy(selected_parents)
                        new_generation.extend(copy_selected_parents)

                population.values = new_generation  # pass the new Generation to population

            for individual in population.values:
                # mutation
                for i, point in enumerate(individual.value):
                    if i == 0 or i == len(individual.value) - 1:
                        continue

                    m_rate = random.random()

                    # if the angle is greater than the setted angle, then this point must mutate
                    if self.must_mutate_angle != 0:
                        if i > 0 or i < len(individual.value) - 1:
                            v1 = [individual.value[i - 1], individual.value[i]]
                            v2 = [individual.value[i], individual.value[i + 1]]
                            angle = Tools.angle_between_vectors(v1, v2)
                            if angle > self.must_mutate_angle:
                                m_rate = 0.0

                    if m_rate <= self.mutation:
                        self.get_mutated_point(individual.value[i - 1], point, individual.value[i + 1], obsticle,
                                               safety_radius)

                        individual.value[i] = self.temp_new_point

                # crossover
                for i, point in enumerate(individual.value):
                    if i == 0 or i >= len(individual.value) - 2:
                        continue
                    c_rate = random.random()
                    if c_rate <= self.crossover:
                        is_legal_start = Tools.is_legal_point(individual.value[i - 1], individual.value[i + 1],
                                                              obsticle, safety_radius)
                        if is_legal_start is False:
                            continue
                        else:
                            is_legal_end = Tools.is_legal_point(point, individual.value[i + 2], obsticle, safety_radius)
                            if is_legal_end is False:
                                continue
                        temp_point = copy.deepcopy(point)
                        individual.value[i] = individual.value[i + 1]
                        individual.value[i + 1] = temp_point

    def get_mutated_point(self, previous_point, current_point, next_point, obsticle, safety_radius):
        """
        get a lagitimate mutated point, which does not collide with obstacle
        :param previous_point:
        :param current_point:
        :param next_point:
        :param obsticle:
        :param safety_radius:
        :return:
        """
        r1 = np.random.uniform(self.step_size[0], self.step_size[1])
        r2 = np.random.uniform(self.step_size[0], self.step_size[1])

        new_x = current_point[0] + r1
        new_y = current_point[1] + r2
        new_point = (new_x, new_y)
        is_legal_start = Tools.is_legal_point(previous_point, new_point, obsticle, safety_radius)
        if is_legal_start:
            is_legal_end = Tools.is_legal_point(new_point, next_point, obsticle, safety_radius)
            if is_legal_end:
                self.temp_new_point = new_point
                return
            else:
                self.get_mutated_point(previous_point, current_point, next_point, obsticle, safety_radius)
        else:
            self.get_mutated_point(previous_point, current_point, next_point, obsticle, safety_radius)
