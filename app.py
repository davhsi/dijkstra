import heapq
import matplotlib.pyplot as plt

class Graph:
    def __init__(self):
        self.nodes = set()
        self.edges = {}
    
    def add_node(self, value):
        self.nodes.add(value)
    
    def add_edge(self, from_node, to_node, weight):
        self.edges.setdefault(from_node, []).append((to_node, weight))
        self.edges.setdefault(to_node, []).append((from_node, weight))

def dijkstra(graph, start):
    shortest_distance = {node: float('inf') for node in graph.nodes}
    shortest_distance[start] = 0
    previous_node = {node: None for node in graph.nodes}
    visited = set()
    heap = [(0, start)]

    while heap:
        current_distance, current_node = heapq.heappop(heap)
        if current_node in visited:
            continue
        visited.add(current_node)

        for neighbor, weight in graph.edges.get(current_node, []):
            distance = current_distance + weight
            if distance < shortest_distance[neighbor]:
                shortest_distance[neighbor] = distance
                previous_node[neighbor] = current_node
                heapq.heappush(heap, (distance, neighbor))

    return shortest_distance, previous_node

def shortest_path(graph, start, end, previous_node):
    path = []
    current_node = end
    while current_node is not None:
        path.append(current_node)
        current_node = previous_node[current_node]
    path.reverse()
    return path

def visualize(graph, shortest_distances, shortest_path):
    for node in graph.nodes:
        plt.text(node[0], node[1], str(node), fontsize=12, ha='center', va='center')
        plt.scatter(node[0], node[1], color='blue', zorder=5)
    
    for node, edges in graph.edges.items():
        for edge in edges:
            plt.plot([node[0], edge[0][0]], [node[1], edge[0][1]], color='black')
            weight_x = (node[0] + edge[0][0]) / 2
            weight_y = (node[1] + edge[0][1]) / 2
            plt.text(weight_x, weight_y, str(edge[1]), fontsize=10, ha='center', va='center')

    for node, distance in shortest_distances.items():
        plt.text(node[0], node[1], str(distance), fontsize=10, ha='center', va='center', color='red')
    
    plt.title("Shortest Path Visualization using Dijkstra's Algorithm")
    plt.xlabel("X")
    plt.ylabel("Y")

    if shortest_path:
        path_x = [node[0] for node in shortest_path]
        path_y = [node[1] for node in shortest_path]
        plt.plot(path_x, path_y, color='green', linewidth=2)
        print("Shortest path:", shortest_path)

    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    graph = Graph()
    graph.add_node((0, 0))
    graph.add_node((1, 1))
    graph.add_node((2, 0))
    graph.add_node((1, 2))
    graph.add_node((4,3))
    graph.add_edge((0, 0), (1, 1), 1)
    graph.add_edge((1, 1), (2, 0), 2)
    graph.add_edge((1, 1), (1, 2), 3)
    graph.add_edge((0, 0), (1, 2), 4)
    graph.add_edge((2, 0), (4, 3), 5)

    start_node = (0, 0)
    end_node = (4, 3)
    shortest_distances, previous_node = dijkstra(graph, start_node)
    shortest_path_nodes = shortest_path(graph, start_node, end_node, previous_node)
    print("Shortest distance from", start_node, "to", end_node, ":", shortest_distances[end_node])

    visualize(graph, shortest_distances, shortest_path_nodes)