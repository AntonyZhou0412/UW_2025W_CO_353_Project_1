import sys
import heapq

# 读取输入
data = sys.stdin.read().split()

# 第一行输入
n, m, r = map(int, data[:3])

# 读取边的信息
edges = []
index = 3
for _ in range(m):
    u, v, w = map(int, data[index:index+3])
    edges.append((u,v,w))
    index += 3

def dijkstra(n, edges, root):
    INF = float('inf')
    adj = [[] for _ in range(n)]
    for u, v, w in edges:
        adj[u].append((v,w))
        adj[v].append((u,w))

    dist = [INF] * n
    dist[root] = 0
    pq = [(0,root)]

    while pq:
        d, u = heapq.heappop(pq)
        if d > dist[u]:
            continue

        for v,w in adj[u]:
            if dist[u] + w < dist[v]:
                dist[v] = dist[u] + w
                heapq.heappush(pq,(dist[v],v))
    
    return dist

# 计算最短距离
distances = dijkstra(n, edges, r)

# 计算边的closeness
edge_closeness = []
for u, v, w in edges:
    d_e = min(distances[u], distances[v])
    edge_closeness.append((d_e,u,v,w))

# 输出结果
print("Distances from root:", distances)
print("Edge closeness:", edge_closeness)