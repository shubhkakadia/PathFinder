from utils import PriorityQueue
import math
from Node import *
from events import *


class task1:

    def __init__(self, start, goal, terrain, threshold):

        # constructor variables
        self.pos_path = []
        self.path = []
        self.start = start
        self.goal = goal
        self.terrain = terrain
        self.threshold = threshold

        # other variables used in search algorithm
        self.path_cost = 0
        self.head = Node(self.start, self.terrain)
        self.explored = set()
        self.searchedNodes = set()
        self.vertical_length = len(self.terrain)
        self.horizontal_length = self.terrain.size / self.vertical_length

    # Function to calculate G value from start to the Node passed on the path.
    def get_g_value(self, n):
        path_cost = 0
        self.calculate_path(n)

        for item in self.path:
            if item == self.head:
                path_cost += item.cost
            elif item.position == self.goal:
                path_cost += item.cost
                return path_cost
            else:
                path_cost += item.cost * 2

        return path_cost

    # Function to calculate H value, finds the distance between 2 nodes.
    def get_h_value(self, n):
        estimate_distance = math.sqrt((n.position[0] - self.goal[0]) ** 2 + (n.position[1] - self.goal[1]) ** 2)
        return estimate_distance

    # Function to calculate F value, sums G value and H value
    def get_f_value(self, n):
        return float(self.get_g_value(n)) + float(self.get_h_value(n))

    # Function to iterate through all the Node's last Node until reached to start. Then store them to list to easily
    # access it later.
    def calculate_path(self, node):
        self.path = []
        while node is not None:
            self.path.append(node)
            node = node.last_node
        self.path.reverse()
        return self.path

    # Function to find the Object location of node objects. This is because if there is a node already
    # created and exists and want to access it. so that program dont create new reference and use the old one instead.
    def find_object(self, pos):
        for node in self.searchedNodes:
            if node.position == pos:
                return node, True
        return None, False

    # Function to use object memory address to access positions in the path.
    def convertObjectRefToNodePos(self, n):
        self.pos_path = []
        for node in self.calculate_path(n):
            self.pos_path.append(node.position)

    # A* algorithm function to find path.
    def astart_algorithm(self):
        frontier = PriorityQueue('min', lambda n: self.get_f_value(n))

        # Check if cost of start node is satisfied by threshold value.
        if self.head.cost < self.threshold:
            frontier.append(self.head)
            self.searchedNodes.add(self.head)

        # loop through frontier
        while frontier:

            node = frontier.pop()
            log_visit_state(node.position, int(self.get_f_value(node)))

            # check if goal reached, if yes return appropriate values.
            if node.position == self.goal:
                self.convertObjectRefToNodePos(node)
                return self.get_g_value(node), self.pos_path

            # add explored nodes to set
            self.explored.add(node.position)

            # search for Neighbouring nodes of current node.
            neighbors = self.neighbor_blocks(node)

            # loop through Neighbouring nodes
            for neibour in neighbors:
                # skip the neibour if already explored.
                if neibour.position in self.explored:
                    log_ignore_state(neibour.position, int(self.get_f_value(neibour)))
                    continue

                # check if cost of the node satisfies threshold value.
                elif neibour.cost > self.threshold:
                    log_ignore_state(neibour.position, int(self.get_f_value(neibour)))
                    continue

                else:
                    # check if node not in frontier, add if not.
                    if neibour not in frontier:
                        log_enqueue_state(neibour.position, int(self.get_f_value(neibour)))
                        frontier.append(neibour)

                    # if node already in frontier, find which is best path and remove and add node to frontier again.
                    else:
                        incumbent_cost = frontier[neibour]
                        if self.get_f_value(neibour) < incumbent_cost:
                            del frontier[neibour]
                            neibour.last_node = node
                            frontier.append(neibour)
        return None, None

    def neighbor_blocks(self, n):
        neibour = []
        # Top neighbor // Check if current position is in first row.
        if n.position[0] != 0:
            position = (n.position[0] - 1, n.position[1])
            if self.find_object(position)[1]:
                node = self.find_object(position)[0]
            else:
                node = Node(position, self.terrain)
                node.last_node = n
            neibour.append(node)

        # Bottom neighbor // Check if current position is in last row.
        if n.position[0] != self.vertical_length - 1:
            position = (n.position[0] + 1, n.position[1])
            if self.find_object(position)[1]:
                node = self.find_object(position)[0]
            else:
                node = Node(position, self.terrain)
                node.last_node = n
            neibour.append(node)

        # Left neighbor // Check if current position is in left first column.
        if n.position[1] != 0:
            position = (n.position[0], n.position[1] - 1)
            if self.find_object(position)[1]:
                node = self.find_object(position)[0]
            else:
                node = Node(position, self.terrain)
                node.last_node = n
            neibour.append(node)

        # Right neighbor // Check if current position is in right last column.
        if n.position[1] != self.horizontal_length - 1:
            position = (n.position[0], n.position[1] + 1)
            if self.find_object(position)[1]:
                node = self.find_object(position)[0]
            else:
                node = Node(position, self.terrain)
                node.last_node = n
            neibour.append(node)

        for i in neibour:
            self.searchedNodes.add(i)

        return neibour

