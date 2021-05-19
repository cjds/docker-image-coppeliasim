import csv
import heapq

from dataclasses import dataclass, field
from typing import Dict, Optional, List

import random

@dataclass
class Node:
    x: float
    y: float
    cost_to_goal: float = 0
    edges:Dict[str, float]  = field(default_factory=dict) # index, weight
    parent:Optional[int] = None


@dataclass
class Obstacle:
    x: float
    y: float
    diameter: float

class Graph:

    def __init__(self):
        self.node_dict = {}

    def add_node(self, node_idx, x,y, cost_to_go):
        self.node_dict[node_idx] = Node(x, y, float(cost_to_go))

    def add_edge(self, node1, node2, weight):
        if(node1 not in self.node_dict or node2 not in self.node_dict):
            # these nodes don't exist
            raise RuntimeError(f"Edges were inserted into {node1}->{node2} but those nodes don't exist")
        self.node_dict[node1].edges[node2] = float(weight)
        # bidirectional
        self.node_dict[node2].edges[node1] = float(weight)

    def get_nodes(self):
        nodes = []
        for n in self.node_dict:
            node = self.node_dict[n]
            nodes.append((n + 1, round(node.x, 3), round(node.y, 3), round(node.cost_to_goal, 3)))
        return nodes

    def get_edges(self):
        edges = []
        for n in self.node_dict:
            node = self.node_dict[n]
            for e in node.edges:
                # avoid duplicating edges
                if int(e) < int(n):
                    edges.append((e + 1, n + 1, round(node.edges[e], 3)))
        return edges

class PRMGenerator:

    def __init__(self, no_of_nodes, robot_radius):
        self.static_obstacles = []
        self.no_of_nodes = no_of_nodes
        self.robot_radius = robot_radius

    def add_obstacle(self, x: str, y:str, diameter: str):
        self.static_obstacles.append(Obstacle(float(x), float(y), float(diameter)))

    def check_for_node_paths_that_collide(self, n1, n2):
        # all obstacles modeled as circles
        # TODO
        # https://mathworld.wolfram.com/Circle-LineIntersection.html
        dx = n2.x - n1.x
        dy = n2.y - n1.y
        dr__2 = dx**2 + dy **2 # kept squared
        D = n1.x * n2.y  - n2.x * n1.y
        for o in self.static_obstacles:
            o_r = (o.diameter/2.0) + self.robot_radius
            delta = ((o_r**2 * dr__2) - D**2)
            if(delta >= 0):
                return True
        return False

    def check_for_collision(self, x, y):
        # all obstacles modeled as circles
        for o in self.static_obstacles:
            if (x - o.x) ** 2 + (y - o.y )**2  < self.robot_radius + (o.diameter/2.0):
                return True
        return False

    def check_if_x_y_in_nodes(self, g, x, y):
        for id in g.node_dict:
            # if we already have a node less than a robot away skip this
            if abs(g.node_dict[id].x - x) < self.robot_radius and abs(g.node_dict[id].y - y) < self.robot_radius:
                return True
        return False

    def get_distance_between_points_sqaured(self, p1, p2):
        return (p1[0] - p2[0])**2 + (p1[1] - p2[1])**2

    def generate_nodes(self, graph, start_node_tuple, end_node_tuple):
        current_no_of_nodes: int = 0
        graph.add_node(current_no_of_nodes, start_node_tuple[0], start_node_tuple[1], 0)
        graph.add_node(current_no_of_nodes, end_node_tuple[0], end_node_tuple[1], self.get_distance_between_points_sqaured(start_node_tuple, end_node_tuple))
        while current_no_of_nodes < self.no_of_nodes:
            # choose random position
            x: float = random.random() - .5
            y: float = random.random() - .5
            if not self.check_if_x_y_in_nodes(graph, x, y) and not self.check_for_collision(x, y):
                graph.add_node(current_no_of_nodes, x, y, self.get_distance_between_points_sqaured((x,y), end_node_tuple))
                current_no_of_nodes += 1

    def generate_edges(self, graph, goal_node_x_y_tuple):
        for n in graph.node_dict:
            for n2 in graph.node_dict:
                if n != n2 and not self.check_for_node_paths_that_collide(graph.node_dict[n], graph.node_dict[n2]):
                    # use goal node to generate a heuristic to store with each node
                    x1 = graph.node_dict[n].x
                    y1 = graph.node_dict[n].y
                    x2 = graph.node_dict[n].x
                    y2 = graph.node_dict[n2].y
                    graph.add_edge(n, n2, self.get_distance_between_points_sqaured((x1, y1), (x2, y2)))

    def generate_graph(self, start_node, goal_node) -> Graph:
        g:Graph = Graph()
        self.generate_nodes(g, start_node, goal_node)
        self.generate_edges(g, goal_node)
        return g


def load_csv_file(file_name, func_to_run):
    with open(file_name) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            # skip comments
            if(row[0].startswith('#')):
                continue
            func_to_run(*row)


def output_csv_file(file_name, rows):
    with open(file_name, 'w') as csv_file:
        csv_writer = csv.writer(csv_file, delimiter=',')
        line_count = 0
        for row in rows:
            csv_writer.writerow(row)

def a_star(g: Graph, starting_node_index: str, ending_node_key: str) -> List[str]:
    visited = []
    to_visit = [(g.node_dict[starting_node_index].cost_to_goal, None, starting_node_index)]

    while(len(to_visit) > 0):
        f: float
        parent: Optional[Node]
        node_index: str
        f, parent, node_key = to_visit.pop(0)
        output = []
        if(node_key == ending_node_key):
            while(g.node_dict[node_key].parent != starting_node_index):
                output.append(node_key)
                node_key = g.node_dict[node_key].parent
            output.append(starting_node_index)
            output.reverse()
            return output
        node = g.node_dict[node_key]
        visited.append(node_key)
        for other_node_key in node.edges:
            if other_node_key not in visited:
                other_node = g.node_dict[other_node_key]
                heapq.heappush(to_visit, (f + node.edges[other_node_key] + g.node_dict[starting_node_index].cost_to_goal, node, other_node_key))
                other_node.parent = node_key


if __name__ == "__main__":
    p: PRMGenerator = PRMGenerator(4, 0)
    load_csv_file("obstacles.csv", p.add_obstacle)
    g = p.generate_graph((-0.5, -.5), (.5, .5))
    # print(a_star(g, '1', '12'))
    output_csv_file('nodes.csv', g.get_nodes())
    output_csv_file('edges.csv', g.get_edges())
    # input edges.csv
    # input nodes.csv
    # output path.csv
