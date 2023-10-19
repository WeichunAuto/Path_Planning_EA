from EA.Individual import Individual


class Population():
    def __init__(self, departure, destination, safety_radius, obsticle, size=50):
        self.departure = departure

        self.destination = destination
        self.safety_radius = safety_radius
        self.obsticle = obsticle

        self.size = size

        self.values = self.initialize()

    def initialize(self):
        """
        initialize individuals
        :return:
        """
        individuals = []
        for i in range(self.size):
            individual = Individual(self.departure, self.destination, self.safety_radius, self.obsticle)
            individuals.append(individual)

        return individuals

# individual = Individual(7, 3, 4, 2)
