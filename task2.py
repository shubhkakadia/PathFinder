from utils import PriorityQueue
import math
from t2_node import *
from events import *


class task2:

    def __init__(self, start, goal, terrain, threshold, eneymy_terrain, probability_threshold):

        # Constructor variables
        self.start = start
        self.goal = goal
        self.terrain = terrain
        self.enemy_terrain = eneymy_terrain
        self.threshold = threshold
        self.probability_threshold = probability_threshold

        # other variables used for searching
        self.pos_path = []
        self.path = []
        self.path_cost = 0
        self.head = t2_node(self.start, self.terrain, self.enemy_terrain)
        self.explored = set()
        self.searchedNodes = set()
        self.vertical_length = len(self.terrain)
        self.horizontal_length = self.terrain.size / self.vertical_length

    # Function to calculate success probability on given perameter (path).
    def calculate_path_success_probability(self, n):
        probability = 1.0

        self.calculate_path(n)
        for item in self.path:
            probability *= (1 - (item.enemy_prob / 100))
        return probability

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
                path_cost += (item.cost * 2)

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
    def calculate_path(self, n):
        self.path = []
        while n is not None:
            self.path.append(n)
            n = n.last_node
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
        self.calculate_path(n)
        for node in self.path:
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
            log_visit_state(node.position, int(self.get_f_value(node)), self.calculate_path_success_probability(node))
            self.calculate_path(node)

            # check if goal reached, if yes return appropriate values.
            if node.position == self.goal:
                self.convertObjectRefToNodePos(node)
                return self.get_g_value(node), self.calculate_path_success_probability(node), self.pos_path

            # add explored nodes to set
            self.explored.add(node.position)

            # search for Neighbouring nodes of current node.
            neighbors = self.neighbor_blocks(node)
            counter = 0

            for neibour in neighbors:
                if neibour.position in self.explored:
                    counter += 1

            add_all = False

            # loop through Neighbouring nodes
            for neibour in neighbors:

                # check if success probability with that node in path satisfies success probability threshold.
                if self.calculate_path_success_probability(neibour) < self.probability_threshold:
                    log_ignore_state(neibour.position, int(self.get_f_value(neibour)),
                                     self.calculate_path_success_probability(neibour))
                    continue

                # if Neighbour already explored, explore again so that program can avoid enemy by visiting explored
                # node. Here this neighbour node is not in path so worth exploring again within the path. Add the
                # node to frontier again and change the last node so that new chain gets used. this could only be
                # added again if frontier has no node to execute and all the neighbouring nodes are explored already
                # (no fresh node available). nodes are explored but are not part of the current chain so needs to
                # explore again.
                if counter == len(neighbors):
                    # use boolean (add_all) because if above scenario is true we need to add all the possible neibours
                    # again and not just 1 neibour
                    if len(frontier) == 0:
                        add_all = True
                    if neibour != node.last_node and add_all:
                        print("here")
                        frontier.append(neibour)
                        neibour.last_node = node
                        log_enqueue_state(neibour.position, int(self.get_f_value(neibour)),
                                          self.calculate_path_success_probability(neibour))
                        continue

                    else:
                        log_ignore_state(neibour.position, int(self.get_f_value(neibour)),
                                         self.calculate_path_success_probability(neibour))
                        continue

                # skip the neibour if already explored.
                elif neibour.position in self.explored:
                    log_ignore_state(neibour.position, int(self.get_f_value(neibour)),
                                     self.calculate_path_success_probability(neibour))
                    continue

                # check if node not in frontier, add if not.
                if neibour not in frontier:
                    frontier.append(neibour)
                    log_enqueue_state(neibour.position, int(self.get_f_value(neibour)),
                                      self.calculate_path_success_probability(neibour))

                # if node already in frontier, find which is best path and remove and add node to frontier again.
                else:
                    incumbent_cost = frontier[neibour]
                    # assign neibour's last node to temporary variable. then change the neibour's last to current node.
                    # its done to compare both path are getting how much cost. if this new path is shorter last node
                    # has been changed anyway if it isnt shorter path, if its longer then change the neibours last to
                    # what it was before because we need to keep original path.
                    temp = neibour.last_node
                    neibour.last_node = node
                    if self.get_f_value(neibour) <= incumbent_cost:
                        del frontier[neibour]
                        frontier.append(neibour)
                        log_enqueue_state(neibour.position, int(self.get_f_value(neibour)),
                                          self.calculate_path_success_probability(neibour))
                    else:
                        log_ignore_state(neibour.position, int(self.get_f_value(neibour)),
                                         self.calculate_path_success_probability(neibour))
                        neibour.last_node = temp

        return None, None, None

    # Function to search for neighbouring nodes, append them to list and use that list in above loop.
    def neighbor_blocks(self, n):
        neibour = []
        # Top neighbor // Check if current position is in first row.
        if n.position[0] != 0:
            position = (n.position[0] - 1, n.position[1])

            # using this function check if the node was already created or not, if yes use that instead of creating new.
            if self.find_object(position)[1]:
                node = self.find_object(position)[0]
            else:
                node = t2_node(position, self.terrain, self.enemy_terrain)
                node.last_node = n

            # check if cost of the node satisfies threshold value.
            if node.cost < self.threshold:
                neibour.append(node)
            else:
                log_ignore_state(node.position, int(self.get_f_value(node)),
                                 self.calculate_path_success_probability(node))

        # Bottom neighbor // Check if current position is in last row.
        if n.position[0] != self.vertical_length - 1:
            position = (n.position[0] + 1, n.position[1])
            if self.find_object(position)[1]:
                node = self.find_object(position)[0]
            else:
                node = t2_node(position, self.terrain, self.enemy_terrain)
                node.last_node = n

            # check if cost of the node satisfies threshold value.
            if node.cost < self.threshold:
                neibour.append(node)
            else:
                log_ignore_state(node.position, int(self.get_f_value(node)),
                                 self.calculate_path_success_probability(node))

        # Left neighbor // Check if current position is in left first column.
        if n.position[1] != 0:
            position = (n.position[0], n.position[1] - 1)
            if self.find_object(position)[1]:
                node = self.find_object(position)[0]
            else:
                node = t2_node(position, self.terrain, self.enemy_terrain)
                node.last_node = n

            # check if cost of the node satisfies threshold value.
            if node.cost < self.threshold:
                neibour.append(node)
            else:
                log_ignore_state(node.position, int(self.get_f_value(node)),
                                 self.calculate_path_success_probability(node))

        # Right neighbor // Check if current position is in right last column.
        if n.position[1] != self.horizontal_length - 1:
            position = (n.position[0], n.position[1] + 1)
            if self.find_object(position)[1]:
                node = self.find_object(position)[0]
            else:
                node = t2_node(position, self.terrain, self.enemy_terrain)
                node.last_node = n

            # check if cost of the node satisfies threshold value.
            if node.cost < self.threshold:
                neibour.append(node)
            else:
                log_ignore_state(node.position, int(self.get_f_value(node)),
                                 self.calculate_path_success_probability(node))

        for i in neibour:
            self.searchedNodes.add(i)

        return neibour
