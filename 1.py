def dfs(g,s,v=None):
    if v is None:
        v=set()
    v.add(s)
    print(s,end=" ")
    
    for neighbor in g[s]:   
        if neighbor not in v:
           dfs(g,neighbor,v)



g={
    'a':['b','c'],
    'b':['d','e'],
    'c':['f'],
    'd':[],
    'e':['f'],
    'f':[]
}
print("dfs")
dfs(g,'a')
------------------------------------------------------

def bfs(g,s):
    v=set()
    q=[s]
    while q:
        n=q.pop(0)
        if n not in v:
        
          print(n,end=" ")
          v.add(n)
          q.extend(g[n])
g={
    'a':['b','c'],
    'b':['d','e'],
    'c':['f'],
    'd':[],
    'e':['f'],
    'f':[]
}
print("bfs")
bfs(g,'a')
---------------------
a*
import heapq

def manhattan_distance(start, goal):
    """Calculate the Manhattan distance between two points."""
    return abs(start[0] - goal[0]) + abs(start[1] - goal[1])

def astar(graph, start, goal):
    """Find the shortest path from start to goal using A* algorithm."""
    frontier = [(0, start)]  # Priority queue to store nodes to be explored
    came_from = {}  # Store the parent of each node
    cost_so_far = {start: 0}  # Cost of reaching each node from start

    while frontier:
        _, current = heapq.heappop(frontier)  # Get the node with the lowest cost

        if current == goal:
            break  # Reached the goal

        for next_node in graph[current]:  # Explore neighbors of the current node
            new_cost = cost_so_far[current] + graph[current][next_node]
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                # Update cost and priority if a shorter path is found
                cost_so_far[next_node] = new_cost
                priority = new_cost + manhattan_distance(goal, next_node)
                heapq.heappush(frontier, (priority, next_node))
                came_from[next_node] = current

    # Reconstruct the path from goal to start
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()

    return path

# Example graph
graph = {
    (0, 0): {(0, 1): 1, (1, 0): 1},
    (0, 1): {(0, 0): 1, (1, 1): 1},
    (1, 0): {(0, 0): 1, (1, 1): 1},
    (1, 1): {(0, 1): 1, (1, 0): 1}
}

start = (0, 0)
goal = (1, 1)
path = astar(graph, start, goal)
print("Path from", start, "to", goal, ":", path)
--------------------------
prims:-

INF = 9999999
# number of vertices in graph
N = 5
#creating graph by adjacency matrix method
G = [[0, 19, 5, 0, 0],
     [19, 0, 5, 9, 2],
     [5, 5, 0, 1, 6],
     [0, 9, 1, 0, 1],
     [0, 2, 6, 1, 0]]

selected_node = [0, 0, 0, 0, 0]

no_edge = 0

selected_node[0] = True

# printing for edge and weight
print("Edge : Weight\n")
while (no_edge < N - 1):
    
    minimum = INF
    a = 0
    b = 0
    for m in range(N):
        if selected_node[m]:
            for n in range(N):
                if ((not selected_node[n]) and G[m][n]):  
                    # not in selected and there is an edge
                    if minimum > G[m][n]:
                        minimum = G[m][n]
                        a = m
                        b = n
    print(str(a) + "-" + str(b) + ":" + str(G[a][b]))
    selected_node[b] = True
    no_edge += 1
---------------------------------------------
g cvoloring:-

def color_graph(graph):
    colors = {}
    color_palette = ["Red", "Green", "Blue", "Yellow", "Orange", "Purple", "Cyan", "Magenta"]
    for node in graph:
        neighbor_colors = {colors[neighbor] for neighbor in graph[node] if neighbor in colors}
        for color in color_palette:
            if color not in neighbor_colors:
                colors[node] = color
                break
    return colors

# Example graph
graph = {
    'A': ['B', 'C'],
    'B': ['A', 'C', 'D'],
    'C': ['A', 'B', 'D'],
    'D': ['B', 'C']
}

colors = color_graph(graph)
for node, color in colors.items():
    print(f"Node {node} = Color {color}")
----------------------------------------------
n queen:-


global N
N = 4
def printSolution(board):
	for i in range(N):
		for j in range(N):
			print (board[i][j],end=' ')
		print()
# A utility function to check if a queen can
# be placed on board[row][col]. Note that this
# function is called when "col" queens are
# already placed in columns from 0 to col -1.
# So we need to check only left side for
# attacking queens
def isSafe(board, row, col):
	# Check this row on left side
	for i in range(col):
		if board[row][i] == 1:
			return False
	# Check upper diagonal on left side
	for i, j in zip(range(row, -1, -1), range(col, -1, -1)):
		if board[i][j] == 1:
			return False
	# Check lower diagonal on left side
	for i, j in zip(range(row, N, 1), range(col, -1, -1)):
		if board[i][j] == 1:
			return False
	return True

def solveNQUtil(board, col):
	# base case: If all queens are placed
	# then return true
	if col >= N:
		return True
	# Consider this column and try placing
	# this queen in all rows one by one
	for i in range(N):
		if isSafe(board, i, col):
			# Place this queen in board[i][col]
			board[i][col] = 1
			# recur to place rest of the queens
			if solveNQUtil(board, col + 1) == True:
				return True
			# If placing queen in board[i][col
			# doesn't lead to a solution, then
			# queen from board[i][col]
			board[i][col] = 0
	# if the queen can not be placed in any row in
	# this column col then return false
	return False
# This function solves the N Queen problem using
# Backtracking. It mainly uses solveNQUtil() to
# solve the problem. It returns false if queens
# cannot be placed, otherwise return true and
# placement of queens in the form of 1s.
# note that there may be more than one
# solutions, this function prints one of the
# feasible solutions.
def solveNQ():
	board = [ [0, 0, 0, 0],
			[0, 0, 0, 0],
			[0, 0, 0, 0],
			[0, 0, 0, 0]
			]

	if solveNQUtil(board, 0) == False:
		print ("Solution does not exist")
		return False
	printSolution(board)
	return True
# driver program to test above function
solveNQ()
