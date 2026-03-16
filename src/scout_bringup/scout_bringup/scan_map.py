from collections import deque

# row: Represents row index of a node
# col: Represent column index of a node
# row_max: Index of last row
# col_max: Index of last column
def get_available_neighbors(graph, row, col, row_max, col_max):
    neighbor_list = list()

    # top left
    if (row-1 != -1) and (col-1 != -1):
        neighbor_row = row - 1
        neighbor_col = col - 1
        if (graph[neighbor_row][neighbor_col] != "visited") and (graph[neighbor_row ][neighbor_col] != -1) and (graph[neighbor_row][neighbor_col] != 100):
            neighbor_list.append([neighbor_row, neighbor_col])

	# left
    if (col-1 != -1):
        neighbor_row = row
        neighbor_col = col - 1
        if (graph[neighbor_row][neighbor_col] != "visited") and (graph[neighbor_row ][neighbor_col] != -1) and (graph[neighbor_row][neighbor_col] != 100):
            neighbor_list.append([neighbor_row, neighbor_col])

	# bottom left
    if (row+1 <= row_max) and (col-1 != -1):
        neighbor_row = row + 1
        neighbor_col = col - 1
        if (graph[neighbor_row][neighbor_col] != "visited") and (graph[neighbor_row ][neighbor_col] != -1) and (graph[neighbor_row][neighbor_col] != 100):
            neighbor_list.append([neighbor_row, neighbor_col])

	# bottom
    if (row+1 <= row_max):
        neighbor_row = row + 1
        neighbor_col = col
        if (graph[neighbor_row][neighbor_col] != "visited") and (graph[neighbor_row ][neighbor_col] != -1) and (graph[neighbor_row][neighbor_col] != 100):
            neighbor_list.append([neighbor_row, neighbor_col])

	# bottom right
    if (row+1 <= row_max) and (col+1 <= col_max):
        neighbor_row = row + 1
        neighbor_col = col + 1
        if (graph[neighbor_row][neighbor_col] != "visited") and (graph[neighbor_row ][neighbor_col] != -1) and (graph[neighbor_row][neighbor_col] != 100):
            neighbor_list.append([neighbor_row, neighbor_col])

	# right
    if (col+1 <= col_max):
        neighbor_row = row
        neighbor_col = col + 1
        if (graph[neighbor_row][neighbor_col] != "visited") and (graph[neighbor_row ][neighbor_col] != -1) and (graph[neighbor_row][neighbor_col] != 100):
            neighbor_list.append([neighbor_row, neighbor_col])

	# top right
    if (row-1 != -1) and (col+1 <= col_max):
        neighbor_row = row - 1
        neighbor_col = col + 1
        if (graph[neighbor_row][neighbor_col] != "visited") and (graph[neighbor_row ][neighbor_col] != -1) and (graph[neighbor_row][neighbor_col] != 100):
            neighbor_list.append([neighbor_row, neighbor_col])

	# top
    if (row-1 != -1):
        neighbor_row = row - 1
        neighbor_col = col
        if (graph[neighbor_row][neighbor_col] != "visited") and (graph[neighbor_row ][neighbor_col] != -1) and (graph[neighbor_row][neighbor_col] != 100):
            neighbor_list.append([neighbor_row, neighbor_col])

    return neighbor_list


# graph: 2D array representing the grid of nodes
# starting_row: Represents the row the starting node is
# ending_row: Represents the column the starting column is
def bfs(graph, starting_row, starting_col):
    queue = deque([[starting_row, starting_col]])
    while len(queue) > 0:
        node = queue.popleft()
        row = node[0]
        col = node[1]
        if (graph[row][col] != "visited"):
            print(f"row: {row} | col: {col}")
            graph[row][col] = "visited"
            for neighbor in get_available_neighbors(graph, row, col, len(graph[row]) - 1, len(graph[col]) - 1):
                queue.append(neighbor)


# Sample input
graph = [
    [0, -1, -1],
    [0, 0, -1],
    [0, 0, -1]
]

bfs(graph, 0, 0)
# should print out: 
# - row: 0 | col: 0
# - row: 1 | col: 0
# - row: 1 | col: 1
# - row: 2 | col: 0
# - row: 2 | col: 1