import heapq                    # Priority queue
import itertools                # For breaking ties in priority queue
import math                     # For square root
import copy
import numpy as np

FREE = 0  # 0 = free, anything else = blocked

class Node:
    # This class holds a "state" of the problem

    def __init__(self, position, parent=None, action=()):
        self.position = position
        self.parent = parent
        self.action = position

    def setParent(self, parent):
        self.parent = parent

    def getParent(self):
        return self.parent

    def getPos(self):
        return self.position

    # For printing the object out (state)
    def __repr__(self):
        return str(self.position)
    
    # For using as a dictionary key
    def __hash__(self):
        # Convert position to a tuple of tuples for hashing
        return hash(self.position)
    
    # For comparison against other Node objects
    def __eq__(self, other):
        if isinstance(other, Node):
            return self.position == other.position
        return False

    # Moving the robot around
    def moveCurrSpot(self, x, y, grid):
        cx, cy = self.position
        nx, ny = cx + x, cy + y
        h, w = grid.shape
        # bounds check
        if 0 <= nx < w and 0 <= ny < h and grid[ny, nx] == FREE:
            return Node((nx, ny), self, (nx, ny))
        return None
    
    # Reconstruct a path from current node to root
    def solutionPath(self):
        node, path = self, []
        while node:
            path.append(node.position)   # store (x, y) coordinate
            node = node.parent
        return path[::-1]  # reverse to get start→goal order

# A* Search with Manhattan Distance Heuristic
def manhattanDistHeuristic(currPos, goalPos):
    return abs(currPos[0] - goalPos[0]) + abs(currPos[1] - goalPos[1])

# Search problem for a solution
def best_first(grid, start, goal):
    nodeNum, maxQueue, goalDepth = 0, 0, 0
    counter = itertools.count()

    currPos = Node(start)

    # Inside the tuple, we store f_n, counter, state, h_n
    # Initilized with initial state of problem
    priority_queue = [(0, 0, next(counter), currPos)]
    visited = {}                        # Visited states; starts empty
    operators = [(1, 0), (-1, 0), (0, 1), (0, -1)]  # List of operations for movement

    while priority_queue:
        h_n = 0     # Current heuristic
        f_n = 0     # Total cost

        # Note max size of priority queue
        if maxQueue < len(priority_queue):
            maxQueue = len(priority_queue)

        # Pop cheapest state from queue, ignore other values
        _, h_n, _, currPos = heapq.heappop(priority_queue)
        print(f"The best state to expand with h(n) = {h_n} is... {currPos}\n")

        # If state matches goal state, end
        if currPos.getPos() == goal:
            print("Goal!!!")
            print(f"\nTo solve this problem the search algorithm expanded a total of {nodeNum} nodes")
            print(f"The maximum number of nodes in the queue at any one time: {maxQueue}")

            return currPos.solutionPath()
        
        # If already visited cheaper node before
        if currPos in visited:
            continue
        
        # Add node to the list of visited
        # Use dictionary to store key - position; value - position
        visited[currPos] = currPos

        # Increment nodes expanded
        nodeNum = nodeNum + 1

        # Iterate through all possible operators (directions)
        for x, y in operators:
            # Break up tuple into x & y components
            newPos = currPos.moveCurrSpot(x, y, grid)

            # If we can move in that direction, then compute heuristic
            if newPos != None and (newPos not in visited):
                h_n = manhattanDistHeuristic(newPos.getPos(), goal)

                f_n = h_n # Calculate total cost (just heuristic for greedy)
                    
                # Add state to priority queue
                heapq.heappush(priority_queue, (f_n, h_n, next(counter), newPos))
            else:
                continue

    print("Failed. No solution found.")
    return 0