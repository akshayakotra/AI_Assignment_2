import math
import copy

class State:
    def __init__(self, grid, pos, goal):
        self.grid = grid
        self.pos = pos
        self.goal = goal

    def goalTest(self):
        return self.pos == self.goal
    
    def moveGen(self):
        moves = [(-1,0), (1,0), (0,-1), (0,1),
                 (-1,-1), (-1,1), (1,-1), (1,1)]
        children = []
        n = len(self.grid)
        i, j = self.pos
        for di, dj in moves:
            ni, nj = i + di, j + dj
            if 0 <= ni < n and 0 <= nj < n and self.grid[ni][nj] == 0:
                children.append(State(self.grid, (ni,nj), self.goal))
        return children

    def h(self):
        # Manhattan distance heuristic
        return abs(self.pos[0] - self.goal[0]) + abs(self.pos[1] - self.goal[1])

    def k_step_cost(self, other):
        return 1
    
    def __eq__(self, other):
        return self.pos == other.pos

    def __hash__(self):
        return hash(self.pos)

    def __str__(self):
        return str(self.pos)

# Best First Search 
def best_first_search(start):
    OPEN = [start]
    CLOSED = set()
    parent = {start: None}

    while OPEN:
        N = min(OPEN, key=lambda x: x.h())
        OPEN.remove(N)

        if N.goalTest():
            return reconstruct_path(parent, N)

        CLOSED.add(N)
        for M in N.moveGen():
            if M not in OPEN and M not in CLOSED:
                parent[M] = N
                OPEN.append(M)
    return None

# A* Search
def a_star(start):
    OPEN = [start]
    CLOSED = set()
    parent = {start: None}
    g = {start: 0}
    f = {start: g[start] + start.h()}

    while OPEN:
        N = min(OPEN, key=lambda x: f[x])
        OPEN.remove(N)

        if N.goalTest():
            return reconstruct_path(parent, N)

        CLOSED.add(N)
        for M in N.moveGen():
            tentative_g = g[N] + N.k_step_cost(M)
            if tentative_g < g.get(M, float("inf")):
                parent[M] = N
                g[M] = tentative_g
                f[M] = g[M] + M.h()
                if M not in OPEN and M not in CLOSED:
                    OPEN.append(M)
    return None

# Path Reconstruction 
def reconstruct_path(parent, goal):
    path = []
    while goal is not None:
        path.append(goal.pos)
        goal = parent[goal]
    return path[::-1]

if __name__ == "__main__":
    n = int(input("Enter grid size n: "))
    grid = []
    print("Enter grid rows (space-separated, 0 for free, 1 for blocked):")
    for i in range(n):
        row = list(map(int, input().split()))
        grid.append(row)

    start = State(grid, (0,0), (n-1,n-1))

    if grid[0][0] == 1 or grid[n-1][n-1] == 1:
        print("Best First Search → Path length: -1")
        print("A* Search → Path length: -1")
    else:
        bfs_path = best_first_search(start)
        if bfs_path:
            print("Best First Search → Path length:", len(bfs_path), ", Path:", bfs_path)
        else:
            print("Best First Search → Path length: -1")

        a_star_path = a_star(start)
        if a_star_path:
            print("A* Search → Path length:", len(a_star_path), ", Path:", a_star_path)
        else:
            print("A* Search → Path length: -1")
