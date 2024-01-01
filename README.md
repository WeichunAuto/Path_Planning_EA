<h1 align='center'>Multi-objective Path Planning for Autonomous Mobile Robot Based on Evolution Algorithm</h1>
<br/><br/><br/>

### Programming Languages Used for This Project
<ul>
  <li>Python</li>
</ul>

### Libs Used under Python for This Project
<ul>
  <li>numpy</li>
  <li>matplotlib</li>
</ul>

# Introduction
The path planning of the robot is to find an optimal or approximately optimal path from the starting position to the destination position without colliding with obstacles. The aim of this paper is to generate a feasible collision-free walking path with the fittest distance and the fittest smoothness from the starting point to the target destination in the limitation of the robot's field of view and no precise map of the environment. It is worth pointing out that a smooth walking path is significantly important for the stability of robots during walking [9]. To track an unsmooth walking path, a robot has to frequently switch its body among ‘stop’, ‘rotation’, and ‘turn’. It will lead to discontinuous velocity and acceleration, which will result in instability and the occurrence of falls. <br /><br>
In considering the size of the robot and obstacles in the surrounding environment, the planned feasible path should fulfill the following criteria and constraints:
<ul>
    <li>The planned feasible path should be a feasible route for the movement of the mobile robot without colliding with any obstacles.</li>
    <li>The planned feasible path should be as smooth as possible and avoid sharp turns in the feasible route.</li>
    <li>The total length of the feasible route should be as short as possible.</li>
</ul>

# Data Source
The data of different environments and obstacles will be randomly generated by Python programming and manually set.

# Limitations
<ul style="list-style-position:outside; font-size: 16px; line-height:20px;">
    <li style='height: 45px;'>The environment and the obstacles and the coordination of each vertex of obstacles are first assumed to be known by sensors, which will not be covered in this research.</li>
    <li style='height: 26px;'>Python programming will be used to simulate the feasible walking path for the robot with the evolution algorithm.</li>
</ul>

# Methodology
<p style='font-size: 16px; line-height:23px;  text-indent: 35px;'>
In this paper, in order to more efficiently plan a feasible path for the robot, the standard evolution algorithm(EA) has been modified from the below five aspects which will be described in detail in the later chapter to form an differential evolution algorithm.
</p>

<ul style="list-style-position:outside; list-style-type:decimal; font-size: 16px; line-height:20px;">
    <li style='height: 26px;'>A policy of asexual reproduction</li>
    <li style='height: 26px;'>Different numbers of "Adjustment Points $N$" with association of the critical value $g$</li>
    <li style='height: 26px;'>Different value of "Step Size $Z$" with association of the critical value $g$</li>
    <li style='height: 26px;'>Different weights allocation $W$ with association of the critical value $g$</li>
    <li style='height: 26px;'>A strategy of a must mutated point</li>
</ul>

<p style='font-size: 16px; line-height:23px;  text-indent: 35px;'>
The following chapter of describing the designed evolution algorithm involves definition of the environment, compute the search space, generate individuals and population, calculate the fitness and parent selection.
</p>

### Definition of the Environment
<p style='font-size: 16px; line-height:30px;  text-indent: 35px;'>
We have known the coordination of the robot’s departure point, as shown the blue dot in (a) of Fighre 1:
    $$P_{d}(x_{d}, y_{d})$$
</p>
<p style='font-size: 16px; line-height:30px;  text-indent: 35px;'>    
The coordination of the robot’s destination point, as shown the red dot in (a) of Fighre 1:
    $$P_{o}(x_{o}, y_{o})$$
</p>
<p style='font-size: 16px; line-height:30px;  text-indent: 35px;'>    
Once an obstacle was detected by its sensor, the coordinates of each vertex of the obstacle are:
    $$P_{v}i(x_{i},y_{i}), (i = 0,1,\ldots,n). \text{  }i \text{ is the number of each vertex of the obstacle.}$$
</p>
<p style='font-size: 16px; line-height:30px;  text-indent: 35px;'>    
And because the robot has its size, we need to define a safe radius R to represent the size, as shown the gray circle surrounded the departure point in (a) of Fighre 1.
    $$\text{Safe Radius = }R$$
</p>

### Compute the Search Space

<p style='font-size: 16px; line-height:23px;  text-indent: 35px;'> 
The search space was calculated by the position of the robot’s departure point, the position of the robot’s destination point and the position of the obstacle, which can be expressed by: 
</p>
<p style='font-size: 16px; line-height:23px;  text-indent: 35px;'> 
    $$
    \text{Search Space = }
    \begin{cases} 
        X1 = f\min(P_{d} \left(x_{d}), \;P_{o}(x_{o}), \;P_{v}i(x_{i}) \right)\ - R \times Buffer\\
        X2 = f\max(P_{d} \left(x_{d}), \;P_{o}(x_{o}), \;P_{v}i(x_{i}) \right)\ + R \times Buffer\\
        Y1 = f\min(P_{d} \left(y_{d}), \;P_{o}(y_{o}), \;P_{v}i(y_{i}) \right)\ - R \times Buffer\\
        Y2 = f\max(P_{d} \left(y_{d}), \;P_{o}(y_{o}), \;P_{v}i(y_{i}) \right)\ + R \times Buffer
    \end{cases} \quad \quad (i = 0,1,\ldots,n).
    $$
</p>
<p style='font-size: 16px; line-height:23px;  text-indent: 35px;'> 
where, $P_{v}i(x_{i})$, $P_{v}i(y_{i})$, are the $x_{i}$ and $y_{i}$ coordination of each vertex of the obstacle. $Buffer$ is a constant. (In this paper, the Value is 15).
X1, X2 are respectively parallel to the Y axis, and Y1, Y2 respectively are parallel to the X axis, and the intersection points of these 4 straight lines form the search space. As shown in (b) of Figure 1, the green dashed rectangle is the search space.
</p>
</br>
<center>
    <img style='width:700px;' src='https://github.com/WeichunAuto/Path_Planning_EA/blob/main/images/en_ss_1.png'/>
    <p align='center'>Figure 1: The environment and search space.</p>
</center>

### Generate Individuals and Population
<p style='font-size: 16px; line-height:23px;  text-indent: 35px;'> 
The first generation of individuals are randomly generated by randomly determining a temporary point to form a population, where the temporary point should fulfill the below constraint:
</p>

<ul style="list-style-position:outside; font-size: 16px; line-height:20px;">
    <li style='height: 45px;'>In considering the safe radius $R$ of the robot, a walking path consists of a departure point, a temporary point, and a destination point, which cannot collide with obstacles.</li>
</ul>

<p style='font-size: 16px; line-height:23px;  text-indent: 35px;'> 
Suppose the coordination of the temporary point is: 
  $$P_{t}(x_{t}, y_{t})\;, \quad P_{t} \in {Search \;Space }$$
</p>

<p style='font-size: 16px; line-height:23px;  text-indent: 35px;'> 
According to the straight line equation formula, the straight line segment equation composed of the departure point and the temporary point, and the straight line segment equation composed of the temporary point and the destination point, which can be expressed as:
    $$Line 1 = A_{1}x + B_{1}y + C_{1}$$
    $$Line 2 = A_{2}x + B_{2}y + C_{2}$$
In order to prove the temporary point satisfies the above constraint, we need to prove the minimum distance from the obstacle to $Line1$ and $Line2$ is greater than the safe radius $R$, and the minimum distance from the obstacle to $Line1$ and $Line2$ can be converted to the minimum distance from one of the vertexes of the obstacle to $Line1$ and $Line2$. 
</p>

<p style='font-size: 16px; line-height:23px;  text-indent: 35px;'> 
    By using the distance formula between two points and the distance formula from a point to a straight line, we can calculate the minimum distance from the obstacle to $Line1$ and $Line2$ is $D$. If $D$ is greater than $R$, then the temporary point $P_{t}(x_{t}, y_{t})$ can satisfy the above constraint.
</p>

<p style='font-size: 16px; line-height:23px;  text-indent: 35px;'> 
Once a valid temporary point is determined, then randomly generate a certain number $N$ of adjustment points on Line1 and Line2. 

<p style='font-size: 16px; line-height:33px;  text-indent: 35px;'> 

<p style='font-size: 16px; line-height:23px;  text-indent: 35px;'> 
The reason why we randomly generate a certain number of adjustment points on $Line1$ and $Line2$ is to give individuals more ability to optimize path smoothness later on. A possible individual was shown in Figure 2. Each individual $m$ in a population of $u$ is composed of:
    <ul style="list-style-position:outside; font-size: 16px; line-height:20px;">
        <li style='height: 26px;'>A feasible walking path: $Individual_m = Path_m$</li>
        <li style='height: 26px;'>Distance fitness: $Fit_{dis}(m)$</li>
        <li style='height: 26px;'>Smoothness fitness: $Fit_{smooth}(m)$</li>
        <li style='height: 26px;'>A set of evaluable strategy parameters: $S_m$</li>
    </ul>
</p>

<img src='https://github.com/WeichunAuto/Path_Planning_EA/blob/main/images/individuals.png'/>
    <p align='center'>Figure 2: The examples of individuals.</p>


### Calculate The Fitness
<img src="https://github.com/WeichunAuto/Path_Planning_EA/blob/main/images/fitness%20func.png"/>

### Parent Selection
<p style='font-size: 16px; line-height:26px;  text-indent: 35px;'> 
    An asexual reproduction policy will be applied in this study, which means new offsprings have only a single parent who has the best fitness over generations. By applying a rank function $f_{rank}(Fit(m))$ and compare with the global fittest individual $Fit_{global}$, the fitter individual will be chosen as a single parent. In other words, the algorithm in this study will maintain a global fittest individual $Fit_{global}$ as a single parent for each generation, and the global fittest individual will be replaced if there is a new offspring who has a better fitness than the global individual in the subsequent generations. The selected parent can be expressed as:
    <img src="https://github.com/WeichunAuto/Path_Planning_EA/blob/main/images/parent%20selection.png"/>

### Mutation and Crossover
<p style='font-size: 16px; line-height:26px;  text-indent: 35px;'> 
The points, $P_k(x_k,y_k),(k = 1,2...,N)$, on a feasible walking path of the individual, each point has a possibility for mutation and crossover with the next adjacent point. the strategy parameters for mutation and crossover were defined as below:

<img src="https://github.com/WeichunAuto/Path_Planning_EA/blob/main/images/crossover.png"/>


<p style='font-size: 16px; line-height:26px;  text-indent: 35px;'> 
    <b>Note that</b>, due to the constraints in obstacles and the safe radius $R$ of the robot, the new point $P_k^,(x_k^,,y_k^,)$ which is mutated from point $P_k(x_k,y_k)$ should fulfill the below first constraint. Similarly, when point $P_k(x_k,y_k)$ crossovers with the next adjacent point $P_{k+1}(x_{k+1},y_{k+1})$, a new walking path should fulfill the below second constraints.
</p>
    <ul style="list-style-position:outside; list-style-type:decimal; font-size: 16px; line-height:20px;">
        <li style='height: 26px;'>The path consists of point $P_{k-1}(x_{k-1},y_{k-1})$, point $P_k^,(x_k^,,y_k^,)$, and point $P_{k+1}(x_{k+1},y_{k+1})$, which cannot collide with obstacles.</li>
        <li style='height: 26px;'>The path consists of point $P_{k-1}(x_{k-1},y_{k-1})$, point $P_{k+1}(x_{k+1},y_{k+1})$, point $P_k(x_k,y_k)$, and point $P_{k+2}(x_{k+2},y_{k+2})$, which cannot collide with obstacles.</li>
    </ul>




# References


<ol style="list-style-position:outside; list-style-type:decimal; font-size: 16px; line-height:20px;">
    <li style='height: 70px;'>Jean Berger et al. “A new mixed-integer linear programming model for rescue path planning in uncertain adversarial environment”. In: Computers Operations Research 39.12 (2012), pp. 3420–3430. issn: 0305-0548. doi: https://doi.org/10.1016/j.cor.2012.05.002. url: https://www.sciencedirect.com/science/ article/pii/S0305054812001049.</li>
    <li style='height: 70px;'>Jun Chen, Jing Liang, and Yan Tong. “Path Planning of Mobile Robot Based on Improved Differential Evolution Algorithm”. In: 2020 16th International Conference on Control, Automation, Robotics and Vision (ICARCV). 2020, pp. 811–816. doi: 10.1109/ICARCV50220.2020.9305415.</li>
    <li style='height: 70px;'>Mahmut Dirik, Oscar Castillo, and Fatih Kocamaz. Vision-Based Mobile Robot Control and Path Planning Algorithms in Obstacle Environments Using Type-2 Fuzzy Logic. Springer International Publishing, 2021. doi: 10.1007/978-3-030-69247-6. url: https://doi.org/10.1007%2F978-3-030-69247-6.</li>
    <li style='height: 50px;'>A.E. Eiben and J.E. Smith. Introduction to Evolutionary Computing. Springer Berlin Heidelberg, 2015. doi: 10.1007/978-3-662-44874-8. url: https://doi.org/10.1007%2F978-3-662-44874-8.</li>
    <li style='height: 50px;'>Ruilong Gao et al. “Apple-Picking Robot Picking Path Planning Algorithm Based on Improved PSO”. In: Electronics 12.8 (Apr. 2023), p. 1832. issn: 2079-9292. doi: 10.3390/electronics12081832. url: http: //dx.doi.org/10.3390/electronics12081832.</li>
    <li style='height: 50px;'>Na Geng, Dunwei Gong, and Yong Zhang. “Robot path planning in an environment with many terrains based on interval multi-objective PSO”. In: 2013 IEEE Congress on Evolutionary Computation. 2013, pp. 813–820. doi: 10.1109/CEC.2013.6557652.</li>
    <li style='height: 50px;'>Jian Guo et al. “Application of the Hybrid Algorithm in Path Planning of the Spherical Mobile Robot”. In: 2018 IEEE International Conference on Mechatronics and Automation (ICMA). 2018, pp. 323–328. doi: 10.1109/ICMA.2018.8484560.</li>
    <li style='height: 70px;'>Wei Hao and Shiyin Qin. “Multi-objective Path Planning for Space Exploration Robot Based on Chaos Immune Particle Swarm Optimization Algorithm”. In: Artificial Intelligence and Computational Intelligence. Ed. by Hepu Deng et al. Berlin, Heidelberg: Springer Berlin Heidelberg, 2011, pp. 42–52. isbn: 978-3-642- 23887-1.</li>
    <li style='height: 50px;'>Shuai Liu et al. “An Improved Differential Evolution Algorithm for Maritime Collision Avoidance Route Planning”. In: Abstract and Applied Analysis 2014 (2014), p. 614569. doi: 10.1155/2014/614569.</li>
    <li style='height: 50px;'>Nannan Lu, Yunlu Gong, and Jie Pan. “Path planning of mobile robot with path rule mining based on GA”. In: 2016 Chinese Control and Decision Conference (CCDC). 2016, pp. 1600–1604. doi: 10.1109/CCDC.2016. 7531239.</li>
    <li style='height: 50px;'>Xiaoning Shen. “A quantum evolutionary algorithm for robot path planning in dynamic environment”. In: Proceedings of the 32nd Chinese Control Conference. 2013, pp. 8061–8065.</li>
    <li style='height: 70px;'>Baoye Song, Zidong Wang, and Lei Zou. “An improved PSO algorithm for smooth path planning of mobile robots using continuous high-degree Bezier curve”. In: Applied Soft Computing 100 (2021), p. 106960. issn: 1568-4946. doi: https://doi.org/10.1016/j.asoc.2020.106960. url: https://www.sciencedirect. com/science/article/pii/S156849462030898X.</li>
    <li style='height: 50px;'>Rainer Storn and Kenneth Price. “Differential Evolution: A Simple and Efficient Adaptive Scheme for Global Optimization Over Continuous Spaces”. In: Journal of Global Optimization 23 (Jan. 1995).</li>
    <li style='height: 70px;'>Ismail AL-Taharwa, Alaa Sheta, and Mohammed Al-Weshah. “A Mobile Robot Path Planning Using Genetic Algorithm in Static Environment”. In: Journal of Computer Science 4.4 (Apr. 2008), pp. 341–344. doi: 10.3844/jcssp.2008.341.344. url: https://thescipub.com/abstract/jcssp.2008.341.344.</li>
    <li style='height: 50px;'>W. C. Wang and R. Chen. “Robot Path Planning with Low Learning Cost Using a Novel K-means-based Pointer Networks”. In: 2019 4th Asia-Pacific Conference on Intelligent Robot Systems (ACIRS). 2019, pp. 84– 88. doi: 10.1109/ACIRS.2019.8936004.</li>
    <li style='height: 70px;'>Chen Xiaoyou Wang Mengtian Zhang Wei. “A Optimal Obstacle Avoidance Path Model for Robots Based on Linear Programming”. In: (2022). doi: https://doi.org/10.12677/MOS.2022.114099. url: https: //www.hanspub.org/journal/PaperInformation.aspx?paperID=53733.</li>
    <li style='height: 50px;'>Liying Yang, Juntong Qi, and Jianda Han. “Path planning methods for mobile robots with linear pro- gramming”. In: 2012 Proceedings of International Conference on Modelling, Identification and Control. 2012, pp. 641–646.</li>
    <li style='height: 70px;'>Sami Zdiri, Jaouher Chrouta, and Abderrahmen Zaafouri. “Cooperative multi-swarm particle swarm opti- mization based on adaptive and time-varying inertia weights”. In: 2021 IEEE 2nd International Conference on Signal, Control and Communication (SCC). 2021, pp. 200–207. doi: 10.1109/SCC53769.2021.9768349.</li>
    <li style='height: 90px;'>Qiushi Zhang, Dandan Chen, and Ting Chen. “An Obstacle Avoidance Method of Soccer Robot Based on Evolutionary Artificial Potential Field”. In: Energy Procedia 16 (2012). 2012 International Conference on Future Energy, Environment, and Materials, pp. 1792–1798. issn: 1876-6102. doi: https://doi.org/ 10.1016/j.egypro.2012.01.276. url: https://www.sciencedirect.com/science/article/pii/ S187661021200286X.</li>
    <li style='height: 50px;'>Xueying Zhang, Xiaobing Yu, and Xuejing Wu. “Exponential Rank Differential Evolution Algorithm for Disaster Emergency Vehicle Path Planning”. In: IEEE Access 9 (2021), pp. 10880–10892. doi: 10.1109/ ACCESS.2021.3050764.</li>
    <li style='height: 70px;'>Zhaojun Zhang et al. “Robot path planning based on genetic algorithm with hybrid initialization method.” In: Journal of Intelligent Fuzzy Systems 42.3 (2022), pp. 2041–2056. issn: 10641246. url: https://search- ebscohost-com.ezproxy.aut.ac.nz/login.aspx?direct=true&db=bth&AN=156139263&site=ehost- live&scope=site.</li>
    <li style='height: 30px;'>Zhou Y, Li X, Gao L (2013) A differential evolution algorithm with intersect mutation operator. Appl Soft Comput 13:390–401</li>
    <li style='height: 50px;'>D. Zaharie, Critical values for the control parameters of differential evolution algorithms, in: Proceedings of 8th International Conference on Soft Computing, June, 2002, pp. 62–67.</li>
    <li style='height: 50px;'>
    J. Rönkkönen, S. Kukkonen, K.V. Price, Real-parameter optimization with dif- ferential evolution, in: Proceedings of Congress on Evolution Computation, vol. 1, IEEE Computer Press, 2005, pp. 567–574.</li>
    <li style='height: 70px;'>
    R. Gämperle, S.D. Müller, P. Koumoutsakos, A parameter study for dif- ferential evolution, in: WSEAS Int. Conf. on Advances in Intelligent Systems, Fuzzy Systems, Evolutionary Computation, Press, 2002, pp. 293–298.</li>
    
</ol>
