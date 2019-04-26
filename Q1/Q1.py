import os
import sys
# We can use a depth first traversal to detect cycles in a graph

# If while visiting adjacent vertices we end up visiting something that has already been visited
# Return False, graph is not acyclic
# Else
# Return True, graph is acyclic

# But not exactly like that because we need to traverse
# Also need to handle the cases of a disconnected graph

# Based on https://www.geeksforgeeks.org/depth-first-search-or-dfs-for-a-graph/
# modified for use

# This class represents a directed graph using adjacency
# list representation
class Graph:
    cyclic_bool = False

    def __init__(self, num_vertices):
        self.graph = [[] for _ in range(num_vertices)]

    # function to add an edge to graph
    def addEdge(self, u, v):
        self.graph[u].append(v)

        # A function used by DFS

    def DFSUtil(self, v, visited):

        # Mark the current node as visited and print it
        visited[v] = True

        # Recur for all the vertices adjacent to
        # this vertex
        for i in self.graph[v]:  # Uses adjacency list to store edges, visit all neighbors
            if not visited[i]:  # Only if it is unvisited
                self.DFSUtil(i, visited)
            else:  # If the DFS tries to visit a neighbor that has already visited, then there must be a cycle
                self.cyclic_bool = True

    # The function to do DFS traversal. It uses
    # recursive DFSUtil()
    def DFS(self):
        V = len(self.graph)  # total vertices

        # Mark all the vertices as not visited
        visited = [False] * (V)

        # Call the recursive helper function to print
        # DFS traversal starting from all vertices one
        # by one
        for i in range(V):  # Visits each node in the graph
            if not visited[i]:  # But only if it is unvisited
                self.DFSUtil(i, visited)


def load_txt_as_graph(filename):
    file = open(filename, "r")
    vertex_count = int(file.readline())
    g = Graph(vertex_count)
    edge_count = int(file.readline())
    print("{} vertices, {} edges".format(vertex_count, edge_count))
    for line in file.readlines():
        u = int(line.split(" ")[0])
        v = int(line.split(" ")[1])
        w = float(line.split(" ")[2])
        g.addEdge(u, v)
    file.close()
    return g


def Q1():
    rel_path = "/Q1/data/mediumEWG.txt"
    cwd = os.getcwd()
    abs_file_path = cwd + rel_path
    g = load_txt_as_graph(abs_file_path)
    print("Running modified DFS to look for cycles")
    g.DFS()
    print("Does graph contain cycles? {}".format(g.cyclic_bool))


def main():
    rel_path = sys.argv[1]  # "/data/mediumEWG.txt"
    cwd = os.getcwd()
    abs_file_path = cwd + rel_path
    g = load_txt_as_graph(abs_file_path)
    print("Running modified DFS to look for cycles")
    g.DFS()
    print("Does graph contain cycles? {}".format(g.cyclic_bool))


if __name__ == "__main__":
    main()
