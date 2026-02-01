from my_graph import *
from timeit import default_timer as timer
import psutil
import os

START_NODE = "Telavi"
END_NODE = "Sokhumi"
process = psutil.Process(os.getpid())

def depth_limited_dfs(graph, node, depth_limit, visited, path):
    if depth_limit < 0:
        return []

    visited.add(node)
    path.append(node)

    paths = [path]

    for neighboor in graph[node]:
        if neighboor not in visited:
            sub_paths = depth_limited_dfs(graph, neighboor, depth_limit - 1, visited.copy(), path.copy())
            if sub_paths:
                paths.extend(sub_paths)

    return paths

def bidirectional_iterative_deepening_dfs(graph, start, goal):
    depth = 0
    while True:
        visited_from_start = {}
        visited_from_goal = {}

        start_paths = depth_limited_dfs(graph, start, depth, set(), [])
        for path in start_paths:
            if path:
                visited_from_start[path[-1]] = path

        goal_paths = depth_limited_dfs(graph, goal, depth, set(), [])
        for path in goal_paths:
            if path:
                visited_from_goal[path[-1]] = path

        for meeting_point in visited_from_start:
            if meeting_point in visited_from_goal:
                path_from_start = visited_from_start[meeting_point]
                path_from_goal = visited_from_goal[meeting_point][::-1]
                return path_from_start + path_from_goal[1:]

        depth += 1

def a_star_search(graph, start, goal):
    open_set = [(start, 0, 0, [start])]
    visited = set()

    while open_set:
        open_set.sort(key=lambda x: x[1] + x[2])
        node, g, f, path = open_set.pop(0)

        if node == goal:
            return path

        if node in visited:
            continue
        visited.add(node)

        for neighbor in graph.get(node, []):
            if neighbor not in visited:
                new_g = g + 1
                new_f = new_g + heuristic.get(neighbor)
                open_set.append((neighbor, new_g, new_f, path + [neighbor]))

    return None


# Measure Bidirectional IDDFS
print("Bidirectional IDDFS")
initial_memory = process.memory_info().rss / (1024 * 1024)  # Memory in MB
start = timer()
path1 = bidirectional_iterative_deepening_dfs(graph, START_NODE, END_NODE)
end = timer()
final_memory = process.memory_info().rss / (1024 * 1024)  # Memory in MB

print(" -> ".join(path1))
print(f"Time taken: {end - start:.6f} seconds")
print(f"Memory used: {final_memory - initial_memory:.2f} MB")

print('-' * 100)

# Measure A* Search
print("A* Search")
initial_memory = process.memory_info().rss / (1024 * 1024)  # Memory in MB
start = timer()
path2 = a_star_search(graph, START_NODE, END_NODE)
end = timer()
final_memory = process.memory_info().rss / (1024 * 1024)  # Memory in MB

print(" -> ".join(path2))
print(f"Time taken: {end - start:.6f} seconds")
print(f"Memory used: {final_memory - initial_memory:.2f} MB")
