from utils import *
from collections import deque
from queue import PriorityQueue
from grid import Grid
from spot import Spot

def bfs(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    if start == None or end == None:
        return False
    
    from collections import deque

    queue = deque()
    queue.append(start)
    visited = {start}
    camefrom = {}

    while len(queue):
        # if quit:
        #     pygame.quit()
        current = queue.popleft()
        if current == end:
            while current in camefrom:
                current = camefrom[current]
                current.make_path()
                draw()
            end.make_end()
            start.make_start()
            return True
        for neighbor in current.neighbors:
            if neighbor not in visited and not neighbor.is_barrier():
                visited.add(neighbor)
                camefrom[neighbor] = current
                queue.append(neighbor)
                neighbor.make_open()
        draw()

        if current != start:
            current.make_closed()
    

def dfs(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    if start == None or end == None:
        return False
    
    stack = [start]
    visited = {start}
    camefrom = {}

    while len(stack):
        current = stack.pop()
        if current == end:
            while current in camefrom:
                current = camefrom[current]
                current.make_path()
                draw()
            end.make_end()
            start.make_start()
            return True
        for neighbor in current.neighbors:
            if neighbor not in visited and not neighbor.is_barrier():
                visited.add(neighbor)
                camefrom[neighbor] = current
                stack.append(neighbor)
                neighbor.make_open()
        draw()
        if current != start:
            current.make_closed()
    return False
    

def h_manhattan_distance(p1: tuple[int, int], p2: tuple[int, int]) -> float:
    """
    Heuristic function for A* algorithm: uses the Manhattan distance between two points.
    Args:
        p1 (tuple[int, int]): The first point (x1, y1).
        p2 (tuple[int, int]): The second point (x2, y2).
    Returns:
        float: The Manhattan distance between p1 and p2.
        
    """
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])
    

def h_euclidian_distance(p1: tuple[int, int], p2: tuple[int, int]) -> float:
    """
    Heuristic function for A* algorithm: uses the Euclidian distance between two points.
    Args:
        p1 (tuple[int, int]): The first point (x1, y1).
        p2 (tuple[int, int]): The second point (x2, y2).
    Returns:
        float: The Manhattan distance between p1 and p2.
    """
    import math
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def astar(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    count = 0
    open_heap = PriorityQueue()

    open_heap.put((0, count, start))
    camefrom = {}
    g_score = {}
    f_score = {}
    for row in grid.grid:
        for col in row:
            g_score[col] = 999999
            f_score[col] = 999999
    g_score[start] = 0
    f_score[start] = h_manhattan_distance(start.get_position(), end.get_position())
    lookup_set = {start}

    while not open_heap.empty():
        current = open_heap.get()[2]
        lookup_set.remove(current)

        if current == end:
            while current in camefrom:
                current = camefrom[current]
                current.make_path()
                draw()
            end.make_end()
            start.make_start()
            return True
        for neighbor in current.neighbors:
            tentative_g = g_score[current] + 1
            if tentative_g < g_score[neighbor]:
                camefrom[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + h_manhattan_distance(neighbor.get_position(), end.get_position())
                if neighbor not in lookup_set:
                    count += 1
                    open_heap.put((f_score[neighbor], count, neighbor))
                    lookup_set.add(neighbor)
                    neighbor.make_open()
        draw()
        if current != start:
            current.make_closed()
    return False
    pass

# and the others algorithms...
# ▢ Depth-Limited Search (DLS)

def dls(draw: callable, grid: Grid, start: Spot, end: Spot, limit: int) -> bool:
    if start == None or end == None:
        return False
    
    stack = [(start, 0)]
    visited = {start}
    camefrom = {}

    while len(stack):
        current, depth = stack.pop()
        if current == end:
            while current in camefrom:
                current = camefrom[current]
                current.make_path()
                draw()
            end.make_end()
            start.make_start()
            return True
        if depth < limit:
            for neighbor in current.neighbors:
                if neighbor not in visited and not neighbor.is_barrier():
                    visited.add(neighbor)
                    camefrom[neighbor] = current
                    stack.append((neighbor, depth + 1))
                    neighbor.make_open()
        draw()
        if current != start:
            current.make_closed()
    return False


# ▢ Uninformed Cost Search (UCS)
from queue import PriorityQueue

def ucs(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    if start is None or end is None:
        return False

    priority_queue = PriorityQueue()
    priority_queue.put((0, start))
    camefrom = {}
    cost_so_far = {start: 0}
    visited = set()

    while not priority_queue.empty():
        current_cost, current_node = priority_queue.get()

        if current_node in visited:
            continue
        visited.add(current_node)

        if current_node == end:
            while current_node in camefrom:
                current_node = camefrom[current_node]
                current_node.make_path()
                draw()
            end.make_end()
            start.make_start()
            return True
        
        for neighbor in current_node.neighbors:
            if neighbor.is_barrier():
                continue

            new_cost = cost_so_far[current_node] + 1

            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                camefrom[neighbor] = current_node
                priority_queue.put((new_cost, neighbor))
                neighbor.make_open()

        draw()

        if current_node != start:
            current_node.make_closed()

    return False


# ▢ Greedy Search

from queue import PriorityQueue

def greedy(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    if start is None or end is None:
        return False

    priority_queue = PriorityQueue()
    camefrom = {}
    visited = set()

    start_h = h_manhattan_distance(start.get_position(), end.get_position())
    priority_queue.put((start_h, start))

    while not priority_queue.empty():
        _, current = priority_queue.get()

        if current in visited:
            continue
        visited.add(current)
        if current == end:
            while current in camefrom:
                current = camefrom[current]
                current.make_path()
                draw()
            end.make_end()
            start.make_start()
            return True

        for neighbor in current.neighbors:
            if neighbor.is_barrier():
                continue
            if neighbor not in visited:
                camefrom[neighbor] = current
                h_value = h_manhattan_distance(neighbor.get_position(), end.get_position())
                priority_queue.put((h_value, neighbor))
                neighbor.make_open()

        draw()
        if current != start:
            current.make_closed()

    return False

# ▢ Iterative Deepening Search/Iterative Deepening Depth-First Search (IDS/IDDFS)

def ids(draw: callable, grid: Grid, start: Spot, end: Spot) -> bool:
    if start is None or end is None:
        return False

    limit = 0
    max_depth = 100

    while limit <= max_depth:

        for row in grid.grid:
            for spot in row:
                if not spot.is_barrier() and spot != start and spot != end:
                    spot.reset()

        found = dls(draw, grid, start, end, limit)
        if found:
            return True

        limit += 1

    return False
# ▢ Iterative Deepening A* (IDA)
# Assume that each edge (graph weight) equalss