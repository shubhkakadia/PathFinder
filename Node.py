from maps import *


class Node:

    def __init__(self, position, terrain):
        self.position = position
        self.cost = terrain[self.position[0]][self.position[1]]
        self.last_node = None

    def get_position(self):
        return self.position

    def get_last_node(self):
        return self.last_node

    def set_last_node(self, node):
        self.last_node = node

    def __lt__(self, other):
        return other

    def __le__(self, other):
        return other
