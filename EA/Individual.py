import numpy as np
from Utility.Tools import *


class Individual():
    def __init__(self, departure, destination, safety_radius, obstacle):
        self.departure = departure

        self.destination = destination
        self.safety_radius = safety_radius
        self.obstacle = obstacle

        self.space = Tools.get_search_space(self.obstacle, self.departure, self.destination, self.safety_radius)

        self.value = []
        self.initialize()

        self.fitness_dis = 0.0
        self.fitness_smooth = 0.0

        self.overal_fitness = 0.0

    def add_points_inPath(self, number_of_points):
        """
        add more adjust points to adjust the smoothness of walking path
        :param number_of_points:
        :return:
        """

        if number_of_points <= len(self.value):
            return
        path_total = []
        # 点的个数， 平均分配
        remain_points = number_of_points - len(self.value)
        num_segs = len(self.value) - 1
        yushu = remain_points % num_segs
        ave_points = int(remain_points / num_segs)

        for i in range(num_segs):
            if yushu > 0:
                path = Tools.points_into_lineSegment(self.value[i], self.value[i + 1], ave_points + 1)
                yushu -= 1
                path.pop()
                path_total.extend(path)
            else:
                path = Tools.points_into_lineSegment(self.value[i], self.value[i + 1], ave_points)
                path.pop()
                path_total.extend(path)

        path_total.append(self.destination)
        self.value = path_total

    def initialize(self):

        self.value = []
        self.value.append(self.departure)

        flag = True

        nextPoint = self.get_randomPoint()
        is_legal_start = Tools.is_legal_point(self.value[0], nextPoint, self.obstacle, self.safety_radius)
        if is_legal_start:
            is_legal_end = Tools.is_legal_point(nextPoint, self.destination, self.obstacle,
                                                self.safety_radius)
            if is_legal_end:
                self.value.append(nextPoint)
                # self.value.append(self.temp_destination)
                self.value.append(self.destination)
                flag = False

        if flag:
            self.initialize()

    def get_randomPoint(self):
        x = np.random.uniform(self.space['min_right'], self.space['max_left'])
        y = np.random.uniform(self.space['min_down'], self.space['max_top'])
        return (x, y)

    def fitness_Distance(self):
        points = self.value
        # print(f"points = {points}")
        fitnessValue = 0
        for i, p in enumerate(points):
            if i < len(points) - 1:
                fitnessValue += Tools.getDistance(points[i], points[i + 1])
                # print(points[i], points[i+1])
        self.fitness_dis = (1 / fitnessValue) * 100

    def fitness_Smoothness(self):
        points = self.value
        fitnessValue = 0
        for i, p in enumerate(points):
            if i < len(points) - 2:
                v1 = [points[i], points[i + 1]]
                v2 = [points[i + 1], points[i + 2]]
                angle = Tools.angle_between_vectors(v1, v2)
                fitnessValue += angle
        self.fitness_smooth = (1 / fitnessValue) * 100

    def combined_fitness(self, weight_dis, weight_smooth):
        self.overal_fitness = weight_dis * self.fitness_dis + weight_smooth * self.fitness_smooth

# individual = Individual(7, 3, 4, 2)
