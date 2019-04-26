# https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-greedy-algo-7/

# Python program for Dijkstra's single
# source shortest path algorithm. The program is
# for adjacency matrix representation of the graph

class Graph():

    def __init__(self, vertices):
        self.V = vertices
        self.graph = [[0 for column in range(vertices)]
                      for row in range(vertices)]

    def printSolution(self, dist):
        print("Vertex   Distance from Source")
        for i in range(self.V):
            print("{} \t\t {:.2f}".format(i, dist[i]))

            # A utility function to find the vertex with

    # minimum distance value, from the set of vertices
    # not yet included in shortest path tree
    def minDistance(self, dist, sptSet):

        # Initilaize minimum distance for next node
        min_val = float("inf")

        # Search not nearest vertex not in the
        # shortest path tree
        for v in range(self.V):
            if dist[v] < min_val and not sptSet[v]:
                min_val = dist[v]
                min_index = v

        return min_index

        # Funtion that implements Dijkstra's single source

    # shortest path algorithm for a graph represented
    # using adjacency matrix representation
    def dijkstra(self, src):

        dist = [float("inf")] * self.V
        dist[src] = 0
        sptSet = [False] * self.V

        for cout in range(self.V):

            # Pick the minimum distance vertex from
            # the set of vertices not yet processed.
            # u is always equal to src in first iteration
            u = self.minDistance(dist, sptSet)

            # Put the minimum distance vertex in the
            # shotest path tree
            sptSet[u] = True

            # Update dist value of the adjacent vertices
            # of the picked vertex only if the current
            # distance is greater than new distance and
            # the vertex in not in the shotest path tree
            for v in range(self.V):
                if self.graph[u][v] != 0 and sptSet[v] == False and dist[v] > dist[u] + self.graph[u][v]:
                    dist[v] = dist[u] + self.graph[u][v]

        self.printSolution(dist)


def Q6():
    g = Graph(8)

    '''
    4 5  0.35
    5 4  0.35
    4 7  0.37
    5 7  0.28
    7 5  0.28
    5 1  0.32
    0 4  0.38
    0 2  0.26
    7 3  0.39
    1 3  0.29
    2 7  0.34
    6 2 -1.20
    3 6  0.52
    6 0 -1.40
    6 4 -1.25
    '''

    edge_list = [
        (4, 5, 0.35),
        (5, 4, 0.35),
        (4, 7, 0.37),
        (5, 7, 0.28),
        (7, 5, 0.28),
        (5, 1, 0.32),
        (0, 4, 0.38),
        (0, 2, 0.26),
        (7, 3, 0.39),
        (1, 3, 0.29),
        (2, 7, 0.34),
        (6, 2, -1.20),
        (3, 6, 0.52),
        (6, 0, -1.40),
        (6, 4, -1.25),
    ]

    for u, v, w in edge_list:
        g.graph[u][v] = w

    print("Running Dijkstra on graph from Q4a (negative weights)")
    g.dijkstra(0)

    g = Graph(8)

    '''
    4 5  0.35
    5 4 -0.66
    4 7  0.37
    5 7  0.28
    7 5  0.28
    5 1  0.32
    0 4  0.38
    0 2  0.26
    7 3  0.39
    1 3  0.29
    2 7  0.34
    6 2  0.40
    3 6  0.52
    6 0  0.58
    6 4  0.93
    '''

    edge_list = [
        (4, 5, 0.35),
        (5, 4, -0.66),
        (4, 7, 0.37),
        (5, 7, 0.28),
        (7, 5, 0.28),
        (5, 1, 0.32),
        (0, 4, 0.38),
        (0, 2, 0.26),
        (7, 3, 0.39),
        (1, 3, 0.29),
        (2, 7, 0.34),
        (6, 2, .4),
        (3, 6, 0.52),
        (6, 0, 0.58),
        (6, 4, .93),
    ]

    for u, v, w in edge_list:
        g.graph[u][v] = w

    print("Running Dijkstra on graph from Q4b (negative cycle)")
    g.dijkstra(0)


if __name__ == "__main__":
    Q6()
