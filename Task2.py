import numpy as np
import matplotlib.pyplot as plt
from collections import deque
import heapq
import math
import time

# Визначення світу (матриця)
world = [
    [0, 0, 0, 0, float('inf'), 0, 0, 0, 0, 0],
    [0, 0, 0, 0, float('inf'), 0, 0, 0, 0, 0],
    [0, 0, 0, 0, float('inf'), 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, float('inf'), float('inf'), float('inf'), float('inf'), float('inf'), 0, 0],
    [0, 0, 0, float('inf'), 0, 0, 0, 0, 0, 0],
    [0, 0, 0, float('inf'), 0, 0, 0, float('inf'), 0, 0]
]

start = (0, 0)  # Початкова позиція (П)
finish = (7, 5)  # Фінальна позиція (Ф)


# Всенапрямлений пошук
def breadth_first_search(world, start, finish):
    m, n = len(world), len(world[0])
    A = np.full((m, n), float('inf'))  # Матриця для відстаней
    A[start] = 0

    queue = deque([start])
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Вверх, вниз, вліво, вправо

    while queue:
        x, y = queue.popleft()

        if (x, y) == finish:
            break

        for dx, dy in directions:
            nx, ny = x + dx, y + dy

            if 0 <= nx < m and 0 <= ny < n and world[nx][ny] == 0 and A[nx][ny] == float('inf'):
                A[nx][ny] = A[x][y] + 1
                queue.append((nx, ny))

    # Відновлення шляху
    path = []
    x, y = finish
    while (x, y) != start:
        path.append((x, y))
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < m and 0 <= ny < n and A[nx][ny] == A[x][y] - 1:
                x, y = nx, ny
                break
    path.append(start)
    path.reverse()

    return A, path


# Візуалізація шляху
def visualize_path(world, path, title=''):
    plt.figure(figsize=(10, 8))
    plt.imshow(np.array(world) == float('inf'), cmap='gray', alpha=0.5)  # Перешкоди
    plt.xticks(range(len(world[0])))
    plt.yticks(range(len(world)))

    # Позначення старту та фінішу
    plt.text(start[1], start[0], 'P', ha='center', va='center', color='blue', fontsize=12, fontweight='bold')
    plt.text(finish[1], finish[0], 'F', ha='center', va='center', color='green', fontsize=12, fontweight='bold')

    # Позначення шляху
    for (x, y) in path:
        plt.text(y, x, '☻', ha='center', va='center', color='orange', fontsize=10)


    plt.grid(True)
    plt.show()


# Алгоритм A*
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def a_star_search(world, start, finish):
    m, n = len(world), len(world[0])
    G = np.full((m, n), float('inf'))  # Матриця вартості шляху
    F = np.full((m, n), float('inf'))  # Матриця евристичної функції
    G[start] = 0
    F[start] = heuristic(start, finish)

    open_set = [(F[start], start)]
    came_from = {}
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == finish:
            break

        for dx, dy in directions:
            nx, ny = current[0] + dx, current[1] + dy

            if 0 <= nx < m and 0 <= ny < n and world[nx][ny] == 0:
                tentative_g_score = G[current] + 1

                if tentative_g_score < G[nx][ny]:
                    came_from[(nx, ny)] = current
                    G[nx][ny] = tentative_g_score
                    F[nx][ny] = tentative_g_score + heuristic((nx, ny), finish)
                    heapq.heappush(open_set, (F[nx][ny], (nx, ny)))

    # Відновлення шляху
    path = []
    x, y = finish
    while (x, y) != start:
        path.append((x, y))
        x, y = came_from[(x, y)]
    path.append(start)
    path.reverse()

    return G, path


# Порівняння ефективності алгоритмів
start_time = time.time()
bfs_path = breadth_first_search(world, start, finish)[1]
bfs_time = time.time() - start_time
print("Час виконання всенапрямленого пошуку:", bfs_time)

start_time = time.time()
a_star_path = a_star_search(world, start, finish)[1]
a_star_time = time.time() - start_time
print("Час виконання алгоритму A*:", a_star_time)

# Візуалізація результатів
visualize_path(world, bfs_path, 'Всенапрямлений пошук')
visualize_path(world, a_star_path, 'Алгоритм A*')
