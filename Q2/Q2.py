import timeit
import functools
import os
import sys


# https://www.geeksforgeeks.org/prims-minimum-spanning-tree-mst-greedy-algo-5/
# A Python program for Prim's Minimum Spanning Tree (MST) algorithm.
# The program is for adjacency matrix representation of the graph

class Graph_Prim():

    def __init__(self, vertices):
        self.V = vertices
        self.graph = [[0 for column in range(vertices)]
                      for row in range(vertices)]

        # A utility function to print the constructed MST stored in parent[]

    def printMST(self, parent):
        print("Edge \tWeight")
        for i in range(1, self.V):
            print(parent[i], "-", i, "\t", self.graph[i][parent[i]])

    # A utility function to find the vertex with
    # minimum distance value, from the set of vertices
    # not yet included in shortest path tree
    def minKey(self, key, mstSet):

        # Initilaize min value
        min_val = float("inf")
        min_index = 0

        for v in range(self.V):
            if key[v] < min_val and not mstSet[v]:
                min_val = key[v]
                min_index = v

        return min_index

        # Function to construct and print MST for a graph

    # represented using adjacency matrix representation
    def primMST(self):

        # Key values used to pick minimum weight edge in cut
        key = [float("inf")] * self.V
        parent = [None] * self.V  # Array to store constructed MST
        # Make key 0 so that this vertex is picked as first vertex
        key[0] = 0
        mstSet = [False] * self.V

        parent[0] = -1  # First node is always the root of

        for cout in range(self.V):

            # Pick the minimum distance vertex from
            # the set of vertices not yet processed.
            # u is always equal to src in first iteration
            u = self.minKey(key, mstSet)

            # Put the minimum distance vertex in
            # the shortest path tree
            mstSet[u] = True

            # Update dist value of the adjacent vertices
            # of the picked vertex only if the current
            # distance is greater than new distance and
            # the vertex in not in the shotest path tree
            for v in range(self.V):
                # graph[u][v] is non zero only for adjacent vertices of m
                # mstSet[v] is false for vertices not yet included in MST
                # Update the key only if graph[u][v] is smaller than key[v]
                if self.graph[u][v] > 0 and not mstSet[v] and key[v] > self.graph[u][v]:
                    key[v] = self.graph[u][v]
                    parent[v] = u

        self.printMST(parent)



# Python program for Kruskal's algorithm to find
# Minimum Spanning Tree of a given connected,
# undirected and weighted graph

from collections import defaultdict


# Class to represent a graph
class Graph_MST:

    def __init__(self, vertices):
        self.V = vertices  # No. of vertices
        self.graph = []  # default dictionary
        # to store graph

    # function to add an edge to graph
    def addEdge(self, u, v, w):
        self.graph.append([u, v, w])

        # A utility function to find set of an element i

    # (uses path compression technique)
    def find(self, parent, i):
        if parent[i] == i:
            return i
        return self.find(parent, parent[i])

        # A function that does union of two sets of x and y

    # (uses union by rank)
    def union(self, parent, rank, x, y):
        xroot = self.find(parent, x)
        yroot = self.find(parent, y)

        # Attach smaller rank tree under root of
        # high rank tree (Union by Rank)
        if rank[xroot] < rank[yroot]:
            parent[xroot] = yroot
        elif rank[xroot] > rank[yroot]:
            parent[yroot] = xroot

            # If ranks are same, then make one as root
        # and increment its rank by one
        else:
            parent[yroot] = xroot
            rank[xroot] += 1

    # The main function to construct MST using Kruskal's
    # algorithm
    def KruskalMST(self):

        result = []  # This will store the resultant MST

        i = 0  # An index variable, used for sorted edges
        e = 0  # An index variable, used for result[]

        # Step 1:  Sort all the edges in non-decreasing
        # order of their
        # weight.  If we are not allowed to change the
        # given graph, we can create a copy of graph
        self.graph = sorted(self.graph, key=lambda item: item[2])

        parent = [];
        rank = []

        # Create V subsets with single elements
        for node in range(self.V):
            parent.append(node)
            rank.append(0)

            # Number of edges to be taken is equal to V-1
        while e < self.V - 1:

            # Step 2: Pick the smallest edge and increment
            # the index for next iteration
            u, v, w = self.graph[i]
            i = i + 1
            x = self.find(parent, u)
            y = self.find(parent, v)

            # If including this edge does't cause cycle,
            # include it in result and increment the index
            # of result for next edge
            if x != y:
                e = e + 1
                result.append([u, v, w])
                self.union(parent, rank, x, y)
                # Else discard the edge

        # print the contents of result[] to display the built MST
        print("Edge\tWeight")
        for u, v, weight in result:
            print("{} - {} \t {}".format(u, v, weight))


def load_txt_as_graph_list(filename):
    file = open(filename, "r")
    vertex_count = int(file.readline())
    g = Graph_MST(vertex_count)
    edge_count = int(file.readline())
    print("{} vertices, {} edges".format(vertex_count, edge_count))
    for line in file.readlines():
        u = int(line.split(" ")[0])
        v = int(line.split(" ")[1])
        w = float(line.split(" ")[2])
        g.addEdge(u, v, w)
    file.close()
    return g


def load_txt_as_graph_matrix(filename):
    file = open(filename, "r")
    vertex_count = int(file.readline())
    g = Graph_Prim(vertex_count)
    edge_count = int(file.readline())
    print("{} vertices, {} edges".format(vertex_count, edge_count))
    for line in file.readlines():
        u = int(line.split(" ")[0])
        v = int(line.split(" ")[1])
        w = float(line.split(" ")[2])
        g.graph[u][v] = w
        g.graph[v][u] = w
    file.close()
    return g


def Q2_output():
    rel_path = "/data/mediumEWG.txt"
    cwd = os.getcwd()
    abs_file_path = cwd + rel_path
    g_kruskal = load_txt_as_graph_list(abs_file_path)
    print("Finidng MST using Kruskal")
    g_kruskal.KruskalMST()

    g_prim = load_txt_as_graph_matrix(abs_file_path)
    print("Finidng MST using Prims")
    g_prim.primMST()


def Q2_timing():
    rel_path = "/data/mediumEWG.txt"
    cwd = os.getcwd()
    abs_file_path = cwd + rel_path
    g_kruskal = load_txt_as_graph_list(abs_file_path)
    print("Timing MST using Kruskal")
    kruskal_timer = timeit.Timer(functools.partial(g_kruskal.KruskalMST))
    k = kruskal_timer.timeit(1)

    g_prim = load_txt_as_graph_matrix(abs_file_path)
    print("Timing MST using Prims")
    g_prim.primMST()
    prim_timer = timeit.Timer(functools.partial(g_prim.primMST))
    p = prim_timer.timeit(1)
    print("Kruskals completes in {} seconds".format(k))
    print("Prims completes in {} seconds".format(p))


def main():
    rel_path = sys.argv[1]  # "/data/mediumEWG.txt"
    cwd = os.getcwd()
    abs_file_path = cwd + rel_path
    g_kruskal = load_txt_as_graph_list(abs_file_path)
    print("Finidng MST using Kruskal")
    g_kruskal.KruskalMST()

    g_prim = load_txt_as_graph_matrix(abs_file_path)
    print("Finidng MST using Prims")
    g_prim.primMST()


if __name__ == "__main__":
    main()

