# CO 353 Project 1.1
# Python version: 3.13.1

import sys
import heapq

def main():
    # Read all input data at once
    data = sys.stdin.read().split()
    ptr = 0

    # Parse graph parameters: n (vertices), m (edges) ,r (roots)
    n = int(data[ptr]); ptr += 1
    m = int(data[ptr]); ptr += 1
    r = int(data[ptr]); ptr += 1

    # Read edges: u, v, weight
    edges = []
    for _ in range(m):
        u = int(data[ptr]); ptr += 1
        v = int(data[ptr]); ptr += 1
        w = int(data[ptr]); ptr += 1
        edges.append((u, v, w))

    # Dijkstra's algorithm to compute shortest paths from root 'r'
    def dijkstra(n, edges, root):
        INF = float('inf')
        adj = [[] for _ in range(n)] #Adjacency list

        # Build adjacency list for the graph
        for u, v, w in edges:
            adj[u].append((v, w))
            adj[v].append((u, w))

        # Initialize distances: dist[i] = shortest dist form root to i
        dist = [INF] * n
        dist[root] = 0

        # Priority queue: (distance, vertex)
        heap = [(0, root)]

        while heap:
            d, u = heapq.heappop(heap)
            if d > dist[u]:
                continue

            # Update distance for neighbors of 'u'
            for v, w in adj[u]:
                if dist[v] > d + w:
                    dist[v] = d + w
                    heapq.heappush(heap, (dist[v], v))
        return dist # Shortest distances from root to all vertices
    
    # Compute shortest distances from root 'r'
    distances = dijkstra(n, edges, r)

    # Calculate edge closeness (d_e, u, v)
    edge_closeness = []
    for u, v, w in edges:
        de = min(distances[u], distances[v]) # Closeness definition
        edge_closeness.append((de, u, v)) # Store as (d_e, u, v)

    # Kruskal's algorithm for minimum and maximum spanning tree
    def kruskal(n, edges, max_tree):
        """
        - n: Number of vertices
        - edges: List of edges in format (d_e, u, v)
        - max_tree: True for Maximum Closeness Spanning Tree; False for Minimum Closeness Spanning Tree
        Returns:
        - Total closeness of the spanning trees
        """
        # Sort edges based on max_tree flag
        sorted_edges = sorted(edges, key=lambda x: x[0], reverse=max_tree)
        
        # Union-Find data structure to track connected components
        parent = list(range(n)) #parent[i] = parent of vertex i
        rank = [1] * n # Union by rank to balance tree height

        # Find root of u with path compression
        def find(u):
            if parent[u] != u:
                parent[u] = find(parent[u])
            return parent[u]

        # Merge sets containing u and v. 
        # Return True if merged
        def union(u, v):
            u_root = find(u)
            v_root = find(v)
            if u_root == v_root:
                return False # Already in the same set

            # Union by rank to balance tree
            if rank[u_root] < rank[v_root]:
                parent[u_root] = v_root
            else:
                parent[v_root] = u_root
                if rank[u_root] == rank[v_root]:
                    rank[u_root] += 1 # Increase rank if trees are equal height
            return True

        total = 0 # Sum of d_e for selected edges
        count = 0 # Count edges added to the spanning tree
        for de, u, v in sorted_edges:
            if union(u, v):
                total += de
                count += 1
                # Early exit if spanning tree has n-1 edges
                if count == n - 1:
                    break
        return total

    # Compute closeness
    min_closeness = kruskal(n, edge_closeness, False)
    max_closeness = kruskal(n, edge_closeness, True)

    # Output results
    print(min_closeness, max_closeness)

if __name__ == "__main__":
    main()