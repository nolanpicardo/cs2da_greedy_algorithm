import heapq
import math

def calculate_travel_time(distance):
    """
    Calculate travel time based on distance and speed rules:
    - Less than 5 miles: 5 miles per hour
    - Between 5 and 10 miles: 20 miles per hour
    - More than 10 miles: 30 miles per hour

    Returns time in hours.
    """
    if distance < 5:
        return distance / 5
    elif distance <= 10:  # Corrected to include 10 miles into 5-10.
        return distance / 20
    else:
        return distance / 30

def dijkstra_shortest_time(graph, start_node):
    n = len(graph)
    times = [math.inf] * n
    times[start_node] = 0
    prev = [None] * n
    pq = [(0, start_node)]  # Priority queue with (time, node)

    while pq:
        current_time, current_node = heapq.heappop(pq)

        if current_time > times[current_node]:
            continue

        for next_node in range(n):
            if math.isnan(graph[current_node][next_node]):  # No direct connection
                continue

            # Calculate time to travel to the next node
            travel_time = calculate_travel_time(graph[current_node][next_node])
            total_time = current_time + travel_time

            if total_time < times[next_node]:
                times[next_node] = total_time
                prev[next_node] = current_node
                heapq.heappush(pq, (total_time, next_node))

    return times, prev


def reconstruct_path(prev, start, end):
    """
    Reconstruct the path from start to end using the prev array.
    """
    path = []
    current = end
    while current is not None:
        path.append(current)
        current = prev[current]
    return list(reversed(path))

# Original adjacency matrix (distances in miles)
graph = [
    [0, math.nan, math.nan, 16, 14, 16],
    [22, 0, 19, math.nan, math.nan, math.nan],
    [math.nan, 19, 0, math.nan, 16, 34],
    [16, math.nan, math.nan, 0, math.nan, 24],
    [math.nan, 18, math.nan, math.nan, 0, 40],
    [16, math.nan, 34, math.nan, math.nan, 0]
]

# Distances from G to other nodes
distances_from_g = [12, 2, 53, 2, 10, 20]

# Append G to the graph
n = len(graph)
for row in graph:
    row.append(math.nan)  # Add column for G
graph.append([math.nan] * (n + 1))  # Add row for G

# Populate distances from G
for i, dist in enumerate(distances_from_g):
    graph[n][i] = dist  # G to other nodes
    graph[i][n] = math.nan  # No paths from other nodes to G

# Run Dijkstra's algorithm from G (node index 6)
start_node = n  # Index of G
times, prev = dijkstra_shortest_time(graph, start_node)

# Output shortest travel times and paths
node_labels = ['A', 'B', 'C', 'D', 'E', 'F', 'G']
print("Shortest travel times from G (Fastest possible Paths):")
for i in range(n):  # Exclude G itself
    path = reconstruct_path(prev, start_node, i)
    path_str = " -> ".join(node_labels[node] for node in path)
    print(f"{node_labels[i]}: Time = {times[i]:.2f} hours, Path = {path_str}")
