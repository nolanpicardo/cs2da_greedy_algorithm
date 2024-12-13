import math
import heapq

def prim_mst(graph):
    """
    Find MST using Prim's algorithm.
    Args:
        graph (list): Symmetric adjacency matrix
    Returns:
        tuple: (edges, total cost)
    """
    num_nodes = len(graph)
    visited = [False] * num_nodes
    min_heap = [(0, 0, -1)]  # (cost, node, parent)
    mst_edges = []
    total_cost = 0

    while min_heap:
        cost, current_node, parent_node = heapq.heappop(min_heap)
        if visited[current_node]:
            continue
        visited[current_node] = True
        if parent_node != -1:  # Skip root
            mst_edges.append((parent_node, current_node, cost))
            total_cost += cost

        for neighbour, weight in enumerate(graph[current_node]):
            if not visited[neighbour] and not math.isnan(weight):
                heapq.heappush(min_heap, (weight, neighbour, current_node))

    return sorted(mst_edges, key=lambda edge: edge[2]), total_cost

def symmetric_min_graph(graph):
    """
    Make graph symmetric with minimum weights.
    Args:
        graph (list): Adjacency matrix
    Returns:
        list: Symmetric matrix
    """
    num_nodes = len(graph)
    return [[
        min(
            graph[i][j] if not math.isnan(graph[i][j]) else math.inf,
            graph[j][i] if not math.isnan(graph[j][i]) else math.inf
        ) if not math.isnan(graph[i][j]) or not math.isnan(graph[j][i]) else math.nan
        for j in range(num_nodes)
    ] for i in range(num_nodes)]

# Input graph
graph = [
    [0, math.nan, math.nan, 16, 14, 16, 12],
    [22, 0, 19, math.nan, math.nan, math.nan, 2],
    [math.nan, 19, 0, math.nan, 16, 34, 53],
    [16, math.nan, math.nan, 0, math.nan, 24, 2],
    [math.nan, 18, math.nan, math.nan, 0, 40, 10],
    [16, math.nan, 34, math.nan, math.nan, 0, 20],
    [12, 2, 53, 2, 10, 20, 0]
]

# Make symmetric graph
symmetric_graph = symmetric_min_graph(graph)

# Compute MST
mst_edges, total_cost = prim_mst(symmetric_graph)

# Print results
print("Prim's Algorithm: Minimum Spanning Tree Edges (sorted by weight):")
for from_node, to_node, cost in mst_edges:
    print(f"{chr(65 + from_node)} - {chr(65 + to_node)} : {cost}")
print(f"\nTotal Cost: {total_cost}")
