import sys
import heapq

def main():
    data = sys.stdin.read().split()
    ptr = 0
    n = int(data[ptr]); ptr += 1
    m = int(data[ptr]); ptr += 1
    r = int(data[ptr]); ptr += 1

    edges = []
    for _ in range(m):
        u = int(data[ptr]); ptr += 1
        v = int(data[ptr]); ptr += 1
        w = int(data[ptr]); ptr += 1
        edges.append((u, v, w))

    # Dijkstra's algorithm to compute shortest paths from root r
    def dijkstra(n, edges, root):
        INF = float('inf')
        adj = [[] for _ in range(n)]
        for u, v, w in edges:
            adj[u].append((v, w))
            adj[v].append((u, w))

        dist = [INF] * n
        dist[root] = 0
        heap = [(0, root)]

        while heap:
            d, u = heapq.heappop(heap)
            if d > dist[u]:
                continue
            for v, w in adj[u]:
                if dist[v] > d + w:
                    dist[v] = d + w
                    heapq.heappush(heap, (dist[v], v))
        return dist

    distances = dijkstra(n, edges, r)

    # Calculate edge closeness (d_e, u, v)
    edge_closeness = []
    for u, v, w in edges:
        de = min(distances[u], distances[v])
        edge_closeness.append((de, u, v))

    # Kruskal's algorithm for minimum and maximum spanning tree
    def kruskal(n, edges, max_tree):
        # Sort edges based on max_tree flag
        sorted_edges = sorted(edges, key=lambda x: x[0], reverse=max_tree)
        parent = list(range(n))
        rank = [1] * n

        def find(u):
            if parent[u] != u:
                parent[u] = find(parent[u])
            return parent[u]

        def union(u, v):
            u_root = find(u)
            v_root = find(v)
            if u_root == v_root:
                return False
            if rank[u_root] < rank[v_root]:
                parent[u_root] = v_root
            else:
                parent[v_root] = u_root
                if rank[u_root] == rank[v_root]:
                    rank[u_root] += 1
            return True

        total = 0
        count = 0
        for de, u, v in sorted_edges:
            if union(u, v):
                total += de
                count += 1
                if count == n - 1:
                    break
        return total

    min_closeness = kruskal(n, edge_closeness, False)
    max_closeness = kruskal(n, edge_closeness, True)

    print(min_closeness, max_closeness)

if __name__ == "__main__":
    main()
