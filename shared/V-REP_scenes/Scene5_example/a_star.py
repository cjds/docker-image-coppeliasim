import csv
import heapq

from dataclasses import dataclass, field
from typing import Dict, Optional, List


@dataclass
class Node:
    x: float
    y: float
    cost_to_goal: float
    edges:Dict[str, float]  = field(default_factory=dict) # index, weight
    parent:Optional[int] = None


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
    g: Graph = Graph()
    load_csv_file("nodes.csv", g.add_node)
    load_csv_file("edges.csv", g.add_edge)
    print(a_star(g, '1', '12'))
    output_csv_file('path2.csv', [a_star(g, '1', '12')])
    # input edges.csv
    # input nodes.csv
    # output path.csv
