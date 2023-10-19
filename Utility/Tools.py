import random

import numpy as np
import math

class Tools():

    # Calculate the coordinate of projection point
    @staticmethod
    def np_point_on_line(startpoint, endpoint, targetpoint):
        """
        Two points determine a straight line.

        startpoint: a ndarray format, two dimational array, np.array([[6, 6]])

        endpoint: a ndarray format, two dimational array, np.array([[12, 7]])

        targetpoint: a ndarray format, two dimational array, np.array([[2, 1]])

        Return a ndarray, two dimational
        """
        ap = targetpoint - startpoint
        ab = endpoint - startpoint

        t = np.sum(np.multiply(ap, ab), axis=1, keepdims=True) / np.sum(np.multiply(ab, ab), axis=1, keepdims=True)
        result = startpoint + np.multiply(t, ab)
        return result

    # Calculate the coordinate of the intersection point of two straight lines
    @staticmethod
    def cross_point(line1, line2):
        # 取直线坐标两点的x和y值
        x1 = line1[0][0]
        y1 = line1[0][1]
        x2 = line1[1][0]
        y2 = line1[1][1]

        x3 = line2[0][0]
        y3 = line2[0][1]
        x4 = line2[1][0]
        y4 = line2[1][1]

        # L2直线斜率不存在操作
        if (x4 - x3) == 0:
            k2 = None
            b2 = 0
            x = x3

            # L1垂直于Y轴
            if x1 == x2:
                return None

            # 计算k1,由于点均为整数，需要进行浮点数转化
            k1 = (y2 - y1) * 1.0 / (x2 - x1)
            # 整型转浮点型是关键
            b1 = y1 * 1.0 - x1 * k1 * 1.0
            y = k1 * x * 1.0 + b1 * 1.0
        elif (x2 - x1) == 0:
            k1 = None
            b1 = 0
            x = x1
            k2 = (y4 - y3) * 1.0 / (x4 - x3)
            b2 = y3 * 1.0 - x3 * k2 * 1.0
            y = k2 * x * 1.0 + b2 * 1.0
        else:
            # 计算k1,由于点均为整数，需要进行浮点数转化
            k1 = (y2 - y1) * 1.0 / (x2 - x1)
            # 斜率存在操作
            k2 = (y4 - y3) * 1.0 / (x4 - x3)
            # if two straight are parallel, return none
            if k1 == k2:
                return None

            # 整型转浮点型是关键
            b1 = y1 * 1.0 - x1 * k1 * 1.0
            b2 = y3 * 1.0 - x3 * k2 * 1.0
            x = (b2 - b1) * 1.0 / (k1 - k2)
            y = k1 * x * 1.0 + b1 * 1.0
        return (x, y)


    @staticmethod
    def is_legal_point(startpoint, nextpoint, obstacle, safety_radius=2):
        is_legal = True
        edges = obstacle.getObstacleEdges()
        # print(edges)
        line2 = [startpoint, nextpoint]
        # print(f'startpoint = {startpoint}, nextpoint = {nextpoint}')
        for edge in edges:
            # 得到起始点和下一个点连成的线条与障碍物每条边 交叉点坐标， 如果交叉点落在障碍物的某一条边上，则该点非法。
            crosspoint = Tools.cross_point(edge, line2)
            # print(crosspoint)
            edge_startpoint = edge[0]
            edge_endpoint = edge[1]
            # calculate the length of edge.
            edge_lenth = round(np.sqrt((edge_endpoint[1]-edge_startpoint[1])**2 + (edge_endpoint[0]-edge_startpoint[0])**2), 8)

            # calculate the length between crosspoint and the end point of edge.
            length_cross_edgeend = np.sqrt((edge_endpoint[1] - crosspoint[1]) ** 2 + (edge_endpoint[0] - crosspoint[0]) ** 2)

            # calculate the length between crosspoint and the start point of edge.
            length_cross_edgestart = np.sqrt((edge_startpoint[1] - crosspoint[1]) ** 2 + (edge_startpoint[0] - crosspoint[0]) ** 2)

            if(round((length_cross_edgeend + length_cross_edgestart), 8) == edge_lenth): # 交点 落在 该条边上
                # 再检查交点 是否落在 startpoint与nextpoint组成的线段上
                length_start_next = Tools.getDistance(startpoint, nextpoint)
                length_cross_start = Tools.getDistance(crosspoint, startpoint)
                length_cross_next = Tools.getDistance(crosspoint, nextpoint)

                if(round((length_cross_start + length_cross_next), 5) == round(length_start_next, 5)):  # 交点在两条线段上
                    is_legal = False
                    # print("there is a cross point")
                    # print(f"crosspoint = {crosspoint}")
                    break

            min_distance = Tools.min_distance_from_pointtoline(edge_startpoint, edge_endpoint, nextpoint)
            if min_distance < safety_radius:  # 如果投影点与目标点之间的距离 小于 安全距离
                # print(f"min_distance = {min_distance}")
                is_legal = False
                break

            # 边的两点分别变成 目标点， 在 起始点 与 下一个点 组成直线上的投影
            min_distance = Tools.min_distance_from_pointtoline(startpoint, nextpoint, edge_startpoint)
            if min_distance < safety_radius:
                # print(f"min_distance = {min_distance}")
                # print(f"边的起始点作为目标点 edge_startpoint = {edge_startpoint}")
                is_legal = False
                break

            min_distance = Tools.min_distance_from_pointtoline(startpoint, nextpoint, edge_endpoint)
            if min_distance < safety_radius:
                # print(f"min_distance = {min_distance}")
                # print(f"边的终点作为目标点 edge_startpoint = {edge_endpoint}")
                is_legal = False
                break

        return is_legal


    # 返回点到线段的最短距离
    @staticmethod
    def min_distance_from_pointtoline(startpoint, endpoint, targetpoint):
        min_distance = 0
        edge_lenth = round(np.sqrt((endpoint[1] - startpoint[1]) ** 2 + (endpoint[0] - startpoint[0]) ** 2), 8)

        s_point = np.array([[startpoint[0], startpoint[1]]])
        e_point = np.array([[endpoint[0], endpoint[1]]])
        t_point = np.array([[targetpoint[0], targetpoint[1]]])

        projectpoint = Tools.np_point_on_line(s_point, e_point, t_point)[0]

        # calculate the length between projectpoint and the end point of edge.
        length_project_edgeend = np.sqrt(
            (endpoint[1] - projectpoint[1]) ** 2 + (endpoint[0] - projectpoint[0]) ** 2)

        # calculate the length between projectpoint and the start point of edge.
        length_project_edgestart = np.sqrt(
            (startpoint[1] - projectpoint[1]) ** 2 + (startpoint[0] - projectpoint[0]) ** 2)

        if (round((length_project_edgeend + length_project_edgestart), 8) == edge_lenth):  # 投影点 落在 该条边上，最短距离为 目标点 和 投影点 的距离

            min_distance = np.sqrt((targetpoint[1] - projectpoint[1]) ** 2 + (targetpoint[0] - projectpoint[0]) ** 2)
            # print("投影点在边上")
            # print(startpoint)
            # print(endpoint)
            # print(targetpoint)
            # print(f"最短距离= {min_distance}")
            return min_distance

        else: # 投影点不在该条边上，最短距离为 边的起点和终点分别与目标点的距离，其中较小者
            length_target_edgeend = round(np.sqrt((endpoint[1] - targetpoint[1]) ** 2 + (endpoint[0] - targetpoint[0]) ** 2), 8)
            length_target_edgestart = round(np.sqrt((startpoint[1] - targetpoint[1]) ** 2 + (startpoint[0] - targetpoint[0]) ** 2), 8)

            min_distance = length_target_edgeend if length_target_edgeend<length_target_edgestart else length_target_edgestart
            # print("投影点不在边上")
            # print(startpoint)
            # print(endpoint)
            # print(targetpoint)
            # print(f"最短距离= {min_distance}")

            return min_distance


    # 计算 点 到障碍物各个顶点的距离，返回最短距离的那个顶点坐标
    @staticmethod
    def min_distance_from_pointtoobs(point, obstacle):
        vertexes = obstacle.getObstaclePoints()
        # print(vertexes)
        min_distance = 0
        min_vertex = None
        for vertex in vertexes:
            distance = Tools.getDistance(point, vertex)
            if min_distance == 0 or distance < min_distance:
                min_distance = distance
                min_vertex = vertex

        # print(f"min_vertex = {min_vertex}")
        return min_vertex

    # 计算两个坐标点之间的距离，精确到小数点8位
    @staticmethod
    def getDistance(point1, point2):
        return round(np.sqrt((point1[1] - point2[1]) ** 2 + (point1[0] - point2[0]) ** 2), 8)


    # 计算临时的 目标 终点，返回坐标
    @staticmethod
    def getTempDestination(departure, destination, obstacle, safety_radius):
        temp_destination = None

        min_vertex = Tools.min_distance_from_pointtoobs(destination, obstacle)

        startpoint = np.array([[departure[0], departure[1]]])
        endpoint = np.array([[destination[0], destination[1]]])
        targetpoint = np.array([[min_vertex[0], min_vertex[1]]])

        project_point = Tools.np_point_on_line(startpoint, endpoint, targetpoint)[0]
        # print(f"project_point = {project_point}")

        x1 = departure[0]
        y1 = departure[1]

        x2 = destination[0]
        y2 = destination[1]

        x3 = project_point[0]
        y3 = project_point[1]

        # 判断投影点是否落在障碍物内（投影点和目标点之间的连线 是否 和 障碍物的边有交点，如果有交点则找与目标点最近的那个交点，并赋值给x3、y3）
        edges = obstacle.getObstacleEdges()
        line2 = [project_point, destination]
        nearest_crosspoint = 0
        nearest_crossedge = None
        min_dis_cross_to_destination = 0
        for edge in edges:
            crosspoint = Tools.cross_point(edge, line2)
            if(crosspoint == None): # 平行
                continue

            # 找到落在障碍物边上的交点
            edge_startpoint = edge[0]
            edge_endpoint = edge[1]

            # 边长
            edge_length = Tools.getDistance(edge_startpoint, edge_endpoint)

            # 交点和边一顶点 长度
            length_cross_edge_end = Tools.getDistance(crosspoint, edge_endpoint)

            # 交点和边一顶点 长度
            length_cross_edge_start = Tools.getDistance(crosspoint, edge_startpoint)
            if round((length_cross_edge_start+length_cross_edge_end), 6) != round(edge_length, 6): # 如果交点不在边上，则终止本轮循环，进入下一轮
                continue

            cross_to_destination = Tools.getDistance(crosspoint, destination)

            if min_dis_cross_to_destination == 0 or min_dis_cross_to_destination > cross_to_destination:
                min_dis_cross_to_destination = cross_to_destination
                nearest_crosspoint = crosspoint
                nearest_crossedge = edge

        dis_project_to_destination = Tools.getDistance(project_point, destination)
        # print(f"nearest_crosspoint = {nearest_crosspoint}")
        fx_random = random.uniform(1.5, 3.5)
        if(dis_project_to_destination > min_dis_cross_to_destination and project_point[0] < destination[0]): # 如果投影点落在障碍物内，则x3取最短交点坐标
            angle = Tools.angle_between_vectors(line2, nearest_crossedge)
            if angle == 90:
                temp_destination = (nearest_crosspoint[0], (nearest_crosspoint[1]+safety_radius) * fx_random)
            else: # 在最短交点与目标点组成的直线上找一个点，该点到最近线段的距离为安全距离
                safe_dis = (safety_radius / np.sin(math.radians(angle))) * fx_random
                temp_destination = Tools.find_point_c(nearest_crosspoint, destination, safe_dis)
                # print(f"temp_destination = {temp_destination}")

            return temp_destination

        # 起点和终点组成的直线 垂直于 X轴
        if x1==x2:
            temp_destination = (x3, y3+safety_radius*fx_random)
            if temp_destination[1] > y2:
                temp_destination = destination
            # print(f"temp_destination = {temp_destination}")
            return temp_destination

        # 起点和终点组成的直线 垂直于 Y轴
        if y1== y2:
            temp_destination = (x3+safety_radius*fx_random, y3)
            if temp_destination[0] > x2:
                temp_destination = destination
            return temp_destination

        # 计算k1,由于点均为整数，需要进行浮点数转化
        k = (y2 - y1) * 1.0 / (x2 - x1)
        # 整型转浮点型是关键
        b = y1 * 1.0 - x1 * k * 1.0

        # 终点在起点的右边
        if x2 > x1:
            temp_destination = Tools.find_point_c(project_point, destination, safety_radius*fx_random)
            # print(f"temp_destination = {temp_destination}")
            if temp_destination[0] > destination[0]:
                temp_destination = destination

            return temp_destination

        # 终点在起点的左边
        if x2 < x1:
            temp_destination = Tools.find_point_c(project_point, destination, safety_radius*fx_random)
            # x4 = x3 - safety_radius
            # temp_destination = (x4, k * x4 + b)
            if(temp_destination[0] < destination[0]):
                temp_destination = destination
            return temp_destination


    # 求两条方向 向量的夹角
    '''
    参数格式：
    v1 = [(0, 1), (1, 1)]
    v2 = [(1, 1), (0, 0.5)]
    '''
    @staticmethod
    def angle_between_vectors(v1, v2):
        # dx1 = v1[2] - v1[0]
        dx1 = v1[1][0] - v1[0][0]
        # dy1 = v1[3] - v1[1]
        dy1 = v1[1][1] - v1[0][1]
        # dx2 = v2[2] - v2[0]
        dx2 = v2[1][0] - v2[0][0]
        # dy2 = v2[3] - v2[1]
        dy2 = v2[1][1] - v2[0][1]

        angle1 = math.atan2(dy1, dx1)
        angle1 = angle1 * 180 / math.pi
        # print(angle1)
        angle2 = math.atan2(dy2, dx2)
        angle2 = angle2 * 180 / math.pi
        # print(angle2)
        if angle1 * angle2 >= 0:
            included_angle = abs(angle1 - angle2)
        else:
            included_angle = abs(angle1) + abs(angle2)
            if included_angle > 180:
                included_angle = 360 - included_angle

        return included_angle

    # 已知直线上有三个点A，B，C。已知A点坐标（e, f），B点坐标（m, n）,点A和点C的距离为S， 求点C的坐标。
    @staticmethod
    def find_point_c(point_A, point_B, S_AC):
        # Given points A and B and distance S between A and C
        """
        我们知道，对于在同一直线上的三个点A、B、C，由于它们在同一直线上，因此它们构成的三角形ABC是一条直线段。我们可以利用相似三角形的性质来推导出点C的坐标。

        设点C的坐标为(x, y)，则有：

        AC / AB = |y - f| / |x - e| = S / |v_AB|
        其中，|v_AB|表示向量v_AB的长度。

        我们已知v_AB = (m-e, n-f)，因此|v_AB| = sqrt((m-e)^2 + (n-f)^2)。

        将AC / AB = S / |v_AB|代入上式，得到：

        |y - f| / |x - e| = S / sqrt((m-e)^2 + (n-f)^2)
        将左侧绝对值去掉，并根据y的正负性，得到以下两个方程：

        y = f + S*(n-f) / sqrt((m-e)^2 + (n-f)^2)
        y = f - S*(n-f) / sqrt((m-e)^2 + (n-f)^2)
        将上述方程与直线方程 y = ((n-f)/(m-e)) * (x - e) + f 相联立，即可求解出点C的坐标。
        """

        A = point_A
        B = point_B
        S = S_AC

        # Calculate vector v_AB
        v_AB = (B[0] - A[0], B[1] - A[1])

        # Calculate the magnitude of v_AB
        mag_v_AB = math.sqrt(v_AB[0] ** 2 + v_AB[1] ** 2)

        # Calculate the coordinates of point C using the equation
        # y = f + S*(n-f) / mag_v_AB and y = f - S*(n-f) / mag_v_AB
        C1 = (A[0] + v_AB[0] * S / mag_v_AB, A[1] + v_AB[1] * S / mag_v_AB)
        C2 = (A[0] - v_AB[0] * S / mag_v_AB, A[1] - v_AB[1] * S / mag_v_AB)

        # Check which point is on the line AB
        # if ((C1[0] - A[0]) / v_AB[0]) == ((C1[1] - A[1]) / v_AB[1]):
        k = (B[1] - A[1]) / (B[0] - A[0])

        if k > 0:
            if C1[0] > C2[0]:
                C = C1
            else:
                C = C2

        else:
            if C1[0] > C2[0]:
                C = C2
            else:
                C = C1

        # print(S)
        # print(C1, C2)
        # print(f"The coordinates of point C are: {C}")
        return C


    # 定义搜索空间范围，障碍物上下/左右 最大与最小坐标与安全距离的2倍
    @staticmethod
    def get_search_space(obsticle, departure, destination, safety_radius):

        points = obsticle.getObstaclePoints()

        max_left = None
        min_right = None
        max_top = None
        min_down = None

        for point in points:

            if max_left is None or max_left < point[0]:
                max_left = point[0]
            if min_right is None or min_right > point[0]:
                min_right = point[0]
            if max_top is None or max_top < point[1]:
                max_top = point[1]
            if min_down is None or min_down > point[1]:
                min_down = point[1]

        if destination[0] > max_left:
            max_left = destination[0]
        if destination[0] < min_right:
            min_right = destination[0]
        if destination[1] > max_top:
            max_top = destination[1]
        if destination[1] < min_down:
            min_down = destination[1]

        if departure[0] < min_right:
            min_right = departure[0]
        if departure[0] > max_left:
            max_left = departure[0]
        if departure[1] > max_top:
            max_top = departure[1]
        if departure[1] < min_down:
            min_down = departure[1]

        space_buffer = safety_radius * 15

        return {
            'max_left': max_left + space_buffer,
            'min_right': min_right - space_buffer,
            'max_top': max_top + space_buffer,
            'min_down': min_down - space_buffer
        }


    # 在一个指定的线段之间，随机添加点。 返回该条线段上 包括起始点和终点的所有点
    @staticmethod
    def points_into_lineSegment(start_point, end_point, num_of_points):
        points = []
        points.append(start_point)

        x1 = start_point[0]
        y1 = start_point[1]

        x2 = end_point[0]
        y2 = end_point[1]

        # 计算 k,b 由于点均为整数，需要进行浮点数转化
        k = (y2 - y1) * 1.0 / (x2 - x1)
        b = y1 * 1.0 - x1 * k * 1.0

        x_array = []
        for i in range(num_of_points):
            x = np.random.uniform(x1, x2)
            x_array.append(x)

        if x2 > x1:
            x_array.sort()
        else:
            x_array.sort(reverse=True)

        for x in x_array:
            y = k * x + b
            points.append((x, y))

        points.append(end_point)

        return points



