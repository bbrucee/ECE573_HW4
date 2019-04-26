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


if __name__ == "__main__":
    # Create a graph given in the above diagram
    g = Graph(4)
    g.addEdge(0, 1)
    g.addEdge(0, 2)
    g.addEdge(1, 2)
    g.addEdge(2, 0)
    g.addEdge(2, 3)
    g.addEdge(3, 3)

    print("Following is Depth First Traversal")
    g.DFS()
    print("Is acyclic? {}".format(g.cyclic_bool))