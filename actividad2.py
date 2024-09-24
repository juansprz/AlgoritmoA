from queue import PriorityQueue

# Definir el grafo del sistema de transporte masivo
graph = {
    'A': {'B': 1, 'C': 3},
    'B': {'A': 1, 'C': 1, 'D': 5},
    'C': {'A': 3, 'B': 1, 'D': 2, 'E': 4},
    'D': {'B': 5, 'C': 2, 'E': 1},
    'E': {'C': 4, 'D': 1}
}

# Definir una función heurística (por ejemplo, la distancia)
heuristic = {
    'A': 5,
    'B': 4,
    'C': 2,
    'D': 6,
    'E': 0
}

def a_star_search(graph, start, goal):
    # Cola de prioridad para los nodos a explorar
    pq = PriorityQueue()
    pq.put((0, start))
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not pq.empty():
        current_cost, current_node = pq.get()
        
        # Si alcanzamos el destino, terminamos
        if current_node == goal:
            break
        
        # Explorar vecinos
        for neighbor, cost in graph[current_node].items():
            new_cost = cost_so_far[current_node] + cost
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic[neighbor]
                pq.put((priority, neighbor))
                came_from[neighbor] = current_node
    
    return reconstruct_path(came_from, start, goal)

def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path

# Ejecutar la búsqueda desde A hasta E
start = 'D'
goal = 'A'
path = a_star_search(graph, start, goal)
print("La mejor ruta de", start, "a", goal, "es:", path)