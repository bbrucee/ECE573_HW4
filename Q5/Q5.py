# Based on https://www.geeksforgeeks.org/depth-first-search-or-dfs-for-a-graph/
# modified for use
# https://www.geeksforgeeks.org/breadth-first-search-or-bfs-for-a-graph/

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
        print(v)

        # Recur for all the vertices adjacent to
        # this vertex
        for i in self.graph[v]:  # Uses adjacency list to store edges, visit all neighbors
            if not visited[i]:  # Only if it is unvisited
                self.DFSUtil(i, visited)
            else:  # If the DFS tries to visit a neighbor that has already visited, then there must be a cycle
                cyclic_bool = True

    # The function to do DFS traversal. It uses
    # recursive DFSUtil()
    def DFS(self, s):
        V = len(self.graph)  # total vertices

        # Mark all the vertices as not visited
        visited = [False] * (V)

        # Start at source
        self.DFSUtil(s, visited)

        # Also call on rest of nodes untouched by source's DFS
        for i in range(V):  # Visits each node in the graph
            if not visited[i]:  # But only if it is unvisited
                self.DFSUtil(i, visited)

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
                if visited[i] == False:
                    queue.append(i)
                    visited[i] = True


def timing():
    pass


if __name__ == "__main__":
    # Driver code
    # Create a graph given in the above diagram
    g = Graph(4)
    g.addEdge(0, 1)
    g.addEdge(0, 2)
    g.addEdge(1, 2)
    g.addEdge(2, 0)
    g.addEdge(2, 3)
    g.addEdge(3, 3)

    print("Following is Depth First Traversal")
    g.DFS(1)
    print("Following is Breadth First Traversal")
    g.BFS(1)