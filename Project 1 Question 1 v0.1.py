import sys
import heapq

def find(parent, u):
    if parent[u] != u:
        parent[u] = find(parent, parent[u])
    return parent[u]

def union(parent, rank, u, v):
    root_u = find(parent, u)
    root_v = find(parent, v)
    
    if root_u == root_v:
        return False  
    
    if rank[root_u] > rank[root_v]:
        parent[root_v] = root_u
    elif rank[root_u] < rank[root_v]:
        parent[root_u] = root_v
    else:
        parent[root_v] = root_u
        rank[root_u] += 1  
    
    return True

def dijkstra(n, edges, root):
    INF = float('inf')
    adj = [[] for _ in range(n)]
    for u, v, w in edges:
        adj[u].append((v, w))
        adj[v].append((u, w))  
    
    dist = [INF] * n
    dist[root] = 0  
    pq = [(0, root)]  
    
    while pq:
        d, u = heapq.heappop(pq)  
        
        if d > dist[u]:  
            continue
        
        for v, w in adj[u]:  
            if dist[u] + w < dist[v]:  
                dist[v] = dist[u] + w
                heapq.heappush(pq, (dist[v], v))  
    
    return dist  

def kruskal(n, edges, increasing=True):
    parent = list(range(n))  
    rank = [1] * n  
    total_closeness = 0  
    edge_count = 0  
    
    # 🔥 Debug: Check sorting order
    print("\nEdges before sorting (for {} Closeness Spanning Tree):".format("Minimum" if increasing else "Maximum"))
    for e in edges:
        print("d_e: {}, Edge: ({}, {}) Weight: {}".format(e[0], e[1], e[2], e[3]))

    # Fix: Sort edges by (d_e DESC, w DESC) for FCST
    edges = sorted(edges, key=lambda x: (x[0], x[3]), reverse=not increasing)

    # 🔥 Debug: Check sorted order
    print("\nEdges after sorting:")
    for e in edges:
        print("d_e: {}, Edge: ({}, {}) Weight: {}".format(e[0], e[1], e[2], e[3]))
    
    for d_e, u, v, w in edges:
        if union(parent, rank, u, v):  
            total_closeness += d_e  
            edge_count += 1
            if edge_count == n - 1:  
                break
    
    # 🔥 Debug: Check selected edges and final closeness sum
    print("\nTotal closeness for {} Closeness Spanning Tree: {}".format("Minimum" if increasing else "Maximum", total_closeness))
    
    return total_closeness

# Read input data
data = sys.stdin.read().split()
n, m, r = map(int, data[:3])  

edges = []
index = 3
for _ in range(m):
    u, v, w = map(int, data[index:index+3])
    edges.append((u, v, w))
    index += 3

# Compute shortest path distances from the root node
shortest_distances = dijkstra(n, edges, r)

# 🔥 Debug: Check computed shortest distances
print("\nShortest distances from root {}:".format(r))
for i in range(n):
    print("Node {}: Distance {}".format(i, shortest_distances[i]))

# Compute d_e values for each edge
edge_closeness = [(min(shortest_distances[u], shortest_distances[v]), u, v, w) for u, v, w in edges]

# Compute the Minimum and Maximum Closeness Spanning Trees
min_closeness = kruskal(n, edge_closeness, increasing=True)
max_closeness = kruskal(n, edge_closeness, increasing=False)

# Output the results
print("\nFinal Output:")
print(min_closeness, max_closeness)
