import matplotlib.pyplot as plt

from EA.Population import Population
from EA.Algorithm import *
from Utility.Obstacle import Obstacle

fig = plt.figure(figsize=(6, 4))

safety_radius = 6

# EA parameters setting
crossover_rate = 0.01
mutation_rate = 0.1
population_size = 50
max_generation = 200
must_mutate_angle = 18

critical_value = 15
adjust_points = {'N1': 7, 'N2': 30}
step_size = {'Z1': (-0.8*safety_radius, 0.8*safety_radius), 'Z2': (-0.2*safety_radius, 0.2*safety_radius)}
weights = {'W1': {'a':0.8, 'b':0.2}, 'W2': {'a':0.2, 'b':0.8}}


destination = (20, 90)
departure = (0, -40)

# draw the obstacle on map
obs = Obstacle()
obs.setObstacle_forE2()
# obs = Obstacles(position=(1, 1), sides=5, area=500)
points = obs.getObstaclePoints()
x, y = obs.getObstacleAxis()
plt.plot(x, y, color='black')

# draw a departure and destination on map
plt.scatter(destination[0], destination[1], s=28, c='red')
plt.plot(departure[0], departure[1], 'b.')

# draw safety circuit around the robot
thia = np.arange(0, 2 * np.pi, 0.01)
x_circuit = departure[0] + safety_radius * np.cos(thia)
y_circuit = departure[1] + safety_radius * np.sin(thia)
plt.plot(x_circuit, y_circuit, color='gray', linewidth=1, alpha=0.3)

# draw search space
search_space = Tools.get_search_space(obs, departure, destination, safety_radius)
plt.plot([search_space['min_right'], search_space['min_right'], search_space['max_left'], search_space['max_left'], search_space['min_right']],
         [search_space['max_top'], search_space['min_down'], search_space['min_down'], search_space['max_top'], search_space['max_top']],
         color='green', linestyle='-.', linewidth=1, alpha=0.5)

p = Population(departure, destination, safety_radius, obs, size=population_size)

ag = Algorithm(generation=max_generation, mutation=mutation_rate, crossover=crossover_rate,
               criticalvalue=critical_value, adjust_points=adjust_points, range_mutation=step_size,
               weights=weights, must_mutate_angle=18)
ag.optimize_with_strategy(p, obs, safety_radius)

best_individual = ag.bestfitness['individual']
gene_best = ag.bestfitness['generation']

print(f"\n--------------------------------\n")
print(f"after optima, gen_best = {gene_best}, the overal fitness = {best_individual.overal_fitness}, best_individual =  {best_individual.value}")
print(f"best_individual_fitness = {best_individual.fitness_dis}")

x_indi = []
y_indi = []
for indi in best_individual.value:
    x_indi.append(indi[0])
    y_indi.append(indi[1])
plt.plot(x_indi, y_indi, 'g-')
plt.scatter(x_indi, y_indi, c='b', s=5)

# plt.axis('scaled')
plt.xlabel("X")
plt.ylabel("Y")
plt.axis('equal')
fig.align_labels()
plt.grid(True)
plt.show()
plt.close()
