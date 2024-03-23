from mazelib import Maze
from mazelib.generate.Prims import Prims
import numpy as np
from mazelib.solve.BacktrackingSolver import BacktrackingSolver

def generate_maze(start_pos=(0,0), goal_pos=(29,29), grid_size_x=30, grid_size_y=30):
    # Ensure start and goal positions are within the grid boundaries
    start_pos = (min(max(start_pos[0], 0), grid_size_x - 1), min(max(start_pos[1], 0), grid_size_y - 1))
    goal_pos = (min(max(goal_pos[0], 0), grid_size_x - 1), min(max(goal_pos[1], 0), grid_size_y - 1))

    # Set the size of the maze
    height, width = grid_size_x, grid_size_y

    # Initialize Maze generator
    m = Maze()
    m.generator = Prims(height, width)
    m.solver = BacktrackingSolver()

    # Generate the maze
    m.generate()

    # Set start and goal positions for the maze
    m.start = (start_pos[1], start_pos[0])  # y, x
    m.end = (goal_pos[1], goal_pos[0])      # y, x
    #
    # # Solve the maze to ensure there's a path from start to goal
    # m.solve()
    #
    # # If the maze is not solvable, regenerate the maze
    # while not m.solved:
    #     m.generate()
    #     m.solve()

    # Convert the maze to a grid format
    maze_grid = m.grid

    # Print the maze for visualization
    for row in maze_grid:
        print(''.join(['#' if cell else ' ' for cell in row]))

    return np.array(maze_grid)