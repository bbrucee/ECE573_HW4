import sys
import os
sys.setrecursionlimit(1000000)


# Based on https://www.geeksforgeeks.org/depth-first-search-or-dfs-for-a-graph/
# modified for use
# https://www.geeksforgeeks.org/breadth-first-search-or-bfs-for-a-graph/

# This class represents a directed graph using adjacency
# list representation
class Graph:

    def __init__(self, num_vertices):
        self.graph = [[] for _ in range(num_vertices)]

    # function to add an edge to graph
    def addEdge(self, u, v):
        self.graph[u].append(v)

        # A function used by DFS

    def DFSUtil(self, v, visited):

        # Mark the current node as visited and print it
        visited[v] = True
        print(v)

        # Recur for all the vertices adjacent to
        # this vertex
        for i in self.graph[v]:  # Uses adjacency list to store edges, visit all neighbors
            if not visited[i]:  # Only if it is unvisited
                self.DFSUtil(i, visited)

    # The function to do DFS traversal. It uses
    # recursive DFSUtil()
    def DFS_recursive(self, s):
        V = len(self.graph)  # total vertices

        # Mark all the vertices as not visited
        visited = [False] * V

        # Start at source
        self.DFSUtil(s, visited)

        # Also call on rest of nodes untouched by source's DFS
        for i in range(V):  # Visits each node in the graph
            if not visited[i]:  # But only if it is unvisited
                self.DFSUtil(i, visited)

    # iterative DFS
    def DFS(self, s):

        # Mark all the vertices as not visited
        visited = [False] * (len(self.graph))

        # Create a stack for DFS
        stack = []

        # Mark the source node as
        # visited and enqueue it
        stack.append(s)
        visited[s] = True

        while stack:

            # take a vertex from
            # stack and print it
            s = stack.pop()
            print(s)

            # Get all adjacent vertices of the
            # vertex s. If a adjacent
            # has not been visited, then mark it
            # visited and stack it
            for i in self.graph[s]:
                if not visited[i]:
                    stack.append(i)
                    visited[i] = True

    # Function to print a BFS of graph
    def BFS(self, s):

        # Mark all the vertices as not visited
        visited = [False] * (len(self.graph))

        # Create a queue for BFS
        queue = []

        # Mark the source node as
        # visited and enqueue it
        queue.append(s)
        visited[s] = True

        while queue:

            # Dequeue a vertex from
            # queue and print it
            s = queue.pop(0)
            print(s)

            # Get all adjacent vertices of the
            # dequeued vertex s. If a adjacent
            # has not been visited, then mark it
            # visited and enqueue it
            for i in self.graph[s]:
                if not visited[i]:
                    queue.append(i)
                    visited[i] = True


def load_txt_as_graph_list(filename):
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


def Q5_output():
    rel_path = "/data/NYC.txt"
    cwd = os.getcwd()
    abs_file_path = cwd + rel_path

    g = load_txt_as_graph_list(abs_file_path)
    print("Running BFS on NYC data, 0 as the source node")
    g.BFS(0)
    print("Running DFS on NYC data, 0 as the source node")
    g.DFS(0)


def main():
    rel_path = sys.argv[1]  # "/data/NYC.txt"
    cwd = os.getcwd()
    abs_file_path = cwd + rel_path

    g = load_txt_as_graph_list(abs_file_path)
    print("Running BFS on NYC data, 0 as the source node")
    g.BFS(0)
    print("Running DFS on NYC data, 0 as the source node")
    g.DFS(0)




if __name__ == "__main__":
    # Q5_output()
    main()