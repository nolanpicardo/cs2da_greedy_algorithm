import heapq  # Priority queue 
import math   # For infinity and NaN values

def dijkstra_shortest_path(graph, start):
    """
    Finds shortest paths from a start node using Dijkstra's algorithm.
    Args:
    graph: Adjacency matrix with weights or inf/NaN for no edge.
    start: Starting node index.
    Returns:
    distances: Shortest distances from the start node.
    prev: Previous nodes in shortest paths.
    """
    n = len(graph)
    distances = [math.inf] * n
    distances[start] = 0
    prev = [None] * n
    pq = [(0, start)]  # (distance, node)

    while pq:
        current_dist, current_node = heapq.heappop(pq)

        if current_dist > distances[current_node]:  # Skip outdated distances
            continue

        for next_node in range(n):
            if math.isnan(graph[current_node][next_node]) or graph[current_node][next_node] == math.inf:
                continue  # Skip invalid edges

            distance = current_dist + graph[current_node][next_node]

            if distance < distances[next_node]:  # Update shorter distance
                distances[next_node] = distance
                prev[next_node] = current_node
                heapq.heappush(pq, (distance, next_node))  # Add to queue

    return distances, prev

def reconstruct_path(prev, start, end):
    path = []
    current = end
    while current is not None:
        path.append(current)
        current = prev[current]
    return list(reversed(path))


# Example graph with NaN values
graph = [
    [0, math.nan, math.nan, 16, 14, 16],
    [22, 0, 19, math.nan, math.nan, math.nan],
    [math.nan, 19, 0, math.nan, 16, 34],
    [16, math.nan, math.nan, 0, math.nan, 24],
    [math.nan, 18, math.nan, math.nan, 0, 40],
    [16, math.nan, 34, math.nan, math.nan, 0]
]

# Find shortest paths from node A (index 0)
start_node = 0
distances, prev = dijkstra_shortest_path(graph, start_node)

# Target nodes
target_nodes = [1, 2, 3, 4, 5]

print("Shortest Distances and Paths from A:")
for node in target_nodes:
    # Reconstruct path
    path = reconstruct_path(prev, start_node, node)
    path_str = " -> ".join(chr(n + 65) for n in path)

    print(f"{chr(node + 65)}: Distance = {distances[node]} miles, Path = {path_str}")
