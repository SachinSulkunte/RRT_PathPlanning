import pygame
import math
import random

class RRTMap:
    def __init__(self, start, goal, map_dim, obs_dim, obs_num):
        self.start = start
        self.goal = goal
        self.map_dim = map_dim
        self.maph, self.mapw = self.map_dim

        # Window
        self.map_window_name = 'RRT Path Planning'
        pygame.display.set_caption(self.map_window_name)
        self.map = pygame.display.set_mode((self.mapw, self.maph))
        self.map.fill((255,255,255))
        self.node_radius = 2
        self.node_thickness = 0
        self.edge_thickness = 1

        self.obstacles = []
        self.obs_dim = obs_dim
        self.obs_num = obs_num

        # Colors
        self.black = (0, 0, 0)
        self.grey = (70, 70, 70)
        self.blue = (0, 0, 255)
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.white = (255, 255, 255)

    def draw_map(self, obstacles):
        pygame.draw.circle(self.map, self.green, self.start, self.node_radius+5, 0)
        pygame.draw.circle(self.map, self.green, self.goal, self.node_radius+20, 1)
        self.draw_obs(obstacles)
    
    def draw_path(self, path):
        for node in path:
            pygame.draw.circle(self.map, self.red, node, 3, 0)

    def draw_obs(self, obstacles):
        obstacle_list = obstacles.copy()
        while (len(obstacle_list) > 0):
            obstacle = obstacle_list.pop(0)
            pygame.draw.rect(self.map, self.grey, obstacle)

class RRTGraph:
    def __init__(self, start, goal, map_dim, obs_dim, obs_num):
        (x, y) = start
        self.start = start
        self.goal = goal
        self.goal_flag = False
        self.maph, self.mapw = map_dim

        self.x = [] # x-coords of tree nodes
        self.y = [] # y-coords of tree nodes
        self.parent = [] # parent of node
        # initialize tree
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0) # root is own parent

        # obstacles
        self.obstacles = []
        self.obs_dim = obs_dim
        self.obs_num = obs_num

        # path
        self.goal_state = None
        self.path = []

    # generate random coordinates corresponding to upper left corner of obstacle
    def makeRandomRect(self):
        upper_cornerx = int(random.uniform(0, self.mapw - self.obs_dim))
        upper_cornery = int(random.uniform(0, self.maph - self.obs_dim))

        return (upper_cornerx, upper_cornery)

    def makeobs(self):
        obs = []
        for i in range(0, self.obs_num):
            rect = None
            startgoal = True
            while startgoal: # check if start or goal is inside obstacle
                upper = self.makeRandomRect()
                rect = pygame.Rect(upper,(self.obs_dim, self.obs_dim))
                if rect.collidepoint(self.start) or rect.collidepoint(self.goal):
                    startgoal = True
                else:
                    startgoal = False
            obs.append(rect)
        self.obstacles = obs.copy()
        return obs

    def add_node(self, num, x, y):
        self.x.insert(num, x)
        self.y.append(y)

    def remove_node(self, num):
        self.x.pop(num)
        self.y.pop(num)

    def add_edge(self, parent, child):
        self.parent.insert(child, parent) # indexed by child

    def remove_edge(self, child_index):
        self.parent.pop(child_index)

    def number_nodes(self): # total num of nodes
        return len(self.x)

    def distance(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        px = (float(x1) - float(x2))**2
        py = (float(y1) - float(y2))**2
        return (px+py)**(0.5) # distance formula between two points

    def sample_env(self):
        x = int(random.uniform(0, self.mapw))
        y = int(random.uniform(0, self.maph))
        return x, y

    def nearest(self, n): # closest node
        dmin = self.distance(0, n)
        nnear = 0
        for i in range(0, n):
            if self.distance(i, n) < dmin:
                dmin = self.distance(i, n)
                nnear = i
        return nnear

    def isFree(self): # determines obstacle presence in free space
        n = self.number_nodes() - 1
        (x, y) = (self.x[n], self.y[n])
        obs = self.obstacles.copy()
        while len(obs) > 0:
            rect = obs.pop(0)
            if rect.collidepoint(x, y):
                self.remove_node(n)
                return False
        return True

    def cross_obs(self, x1, x2, y1, y2): # check if line to node would cross an obstacle
        obs = self.obstacles.copy()
        while (len(obs) > 0):
            rect = obs.pop(0)
            for i in range(0, 101): # interpolation between two points
                u = i/100
                x = x1*u + x2*(1 - u)
                y = y1*u + y2*(1 - u)
                if rect.collidepoint(x, y):
                    return True
        return False

    def connect(self, n1, n2): # connect a node via an edge if it does not cross an obstacle
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        if self.cross_obs(x1, x2, y1, y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1, n2)
            return True

    def step(self, nnear, nrand, dmax=35):
        d = self.distance(nnear, nrand)
        if d > dmax:
            u = dmax / d
            (xnear, ynear) = (self.x[nnear], self.y[nnear])
            (xrand, yrand) = (self.x[nrand], self.y[nrand])
            (px, py) = (xrand - xnear, yrand - ynear)
            theta = math.atan2(py, px)
            (x, y) = (int(xnear + dmax * math.cos(theta)),
                      int(ynear + dmax * math.sin(theta)))
            self.remove_node(nrand)
            if abs(x - self.goal[0]) <= dmax and abs(y - self.goal[1]) <= dmax:
                self.add_node(nrand, self.goal[0], self.goal[1])
                self.goal_state = nrand
                self.goal_flag = True
            else:
                self.add_node(nrand, x, y)

    def path_to_goal(self):
        if self.goal_flag:
            self.path = []
            self.path.append(self.goal_state)
            newpos = self.parent[self.goal_state]
            while (newpos != 0):
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
        return self.goal_flag

    def get_path_coords(self):
        pathCoords = []
        for node in self.path:
            x, y = (self.x[node], self.y[node])
            pathCoords.append((x, y))
        return pathCoords

    def bias(self, ngoal):
        n = self.number_nodes()
        self.add_node(n, ngoal[0], ngoal[1])
        nnear = self.nearest(n)
        self.step(nnear, n)
        self.connect(nnear, n)
        return self.x, self.y, self.parent

    def expand(self):
        n = self.number_nodes()
        x, y = self.sample_env()
        self.add_node(n, x, y)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest, n)
            self.connect(xnearest, n)
        return self.x, self.y, self.parent

    def cost(self):
        ninit = 0
        n = n
        parent = self.parent[n]
        c = 0
        while n is not ninit:
            c = c + self.distance(n, parent)
            n = parent
            if n is not ninit:
                parent = self.parent[n]
        return c

    def getTrueObs(self, obs):
        TOBS = []
        for ob in obs:
            TOBS.append(ob.inflate(-50, -50))
        return TOBS

    def waypoints2path(self):
        oldpath = self.get_path_coords()
        path = []
        for i in range(0, len(self.path) - 1):
            print(i)
            if i >= len(self.path):
                break
            x1, y1 = oldpath[i]
            x2, y2 = oldpath[i + 1]
            print('---------')
            print((x1, y1), (x2, y2))
            for i in range(0, 5):
                u = i / 5
                x = int(x2 * u + x1 * (1 - u))
                y = int(y2 * u + y1 * (1 - u))
                path.append((x, y))
                print((x, y))

        return path
