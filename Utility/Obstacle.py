class Obstacle():

    def __init__(self):
        p1 = (25, 5)
        p2 = (65, 15)
        p3 = (55, 38)
        p4 = (20, 45)
        p5 = (0, 40)
        p6 = (5, 12)
        p7 = (10, 5)

        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self.p4 = p4
        self.p5 = p5
        self.p6 = p6
        self.p7 = p7

    def setObstacle_forE1(self):
        self.p1 = (25, 10)
        self.p2 = (50, 20)
        self.p3 = (50, 40)
        self.p4 = (20, 45)
        self.p5 = (0, 25)
        self.p6 = (0, 12)
        self.p7 = (15, 20)

    def setObstacle_forE2(self):
        self.p1 = (25, 10)
        self.p2 = (60, 20)
        self.p3 = (50, 40)
        self.p4 = (20, 45)
        self.p5 = (0, 25)
        self.p6 = (0, 12)
        self.p7 = (10, 5)

    def setObstacle_forE3(self):
        self.p1 = (25, 5)
        self.p2 = (65, 15)
        self.p3 = (55, 38)
        self.p4 = (20, 45)
        self.p5 = (0, 40)
        self.p6 = (5, 12)
        self.p7 = (10, 5)

    def setObstacle_forE4(self):
        self.p1 = (20, 10)
        self.p2 = (45, 20)
        self.p3 = (50, 38)
        self.p4 = (20, 45)
        self.p5 = (0, 40)
        self.p6 = (0, 12)
        self.p7 = (10, 15)

    def setObstacle_forE5(self):
        self.p1 = (20, 12)
        self.p2 = (50, 20)
        self.p3 = (50, 70)
        self.p4 = (35, 80)
        self.p5 = (0, 70)
        self.p6 = (0, 12)
        self.p7 = (10, 15)

    def getObstaclePoints(self):

        return [self.p1, self.p2, self.p3, self.p4, self.p5, self.p6, self.p7]

    def getObstacleAxis(self):
        points = self.getObstaclePoints()
        x = []
        y = []
        for index, point in enumerate(points):
            x.append(point[0])
            y.append(point[1])
            if index + 1 == len(points):
                x.append(points[0][0])
                y.append(points[0][1])
        return x, y

    def getObstacleEdges(self):
        points = self.getObstaclePoints()
        edges = []
        for index, point in enumerate(points):
            edges.append([point, points[0 if index + 1 == len(points) else index + 1]])

        return edges

# points_number=5
# obs1 = Obstacle()
# print(obs1.getObstaclePoints())
