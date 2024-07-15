import numpy as np
import random

def generate_connected_zero_mass(rows, cols, zero_fraction):
    # Start with an array filled with 1's
    array = np.ones((rows, cols), dtype=int)

    # Determine the number of 0's to place
    total_cells = rows * cols
    num_zeros = int(total_cells * zero_fraction)

    # Initialize the starting point for the flood fill
    start_x = np.random.randint(0, rows)
    start_y = np.random.randint(0, cols)
    array[start_x, start_y] = 0
    num_zeros -= 1

    # Use a stack for flood fill
    stack = [(start_x, start_y)]

    while num_zeros > 0 and stack:
        x, y = stack.pop()

        # Get neighbors (4-connectivity)
        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
        random.shuffle(neighbors)

        for nx, ny in neighbors:
            if 0 <= nx < rows and 0 <= ny < cols and array[nx, ny] == 1:
                array[nx, ny] = 0
                stack.append((nx, ny))
                num_zeros -= 1
                if num_zeros <= 0:
                    break

    # Ensure the bottom 20% of the map is clear
    array[-1, :] = 0
    array[-2, :] = 0

    return array

def place_enemies_in_map(original_array):
    # Copy the original array to a new array
    new_array = np.zeros_like(original_array)
    
    # Get the positions of all 0's in the original array
    zero_positions = np.argwhere(original_array == 0)
    one_positions = np.argwhere(original_array == 1)
    
    # Randomly decide which 0's to replace with 1's
    for pos in zero_positions:
        if np.random.rand() < 0.01:  # 50% chance to replace a 0 with a 1
            new_array[tuple(pos)] = 1

    for pos in one_positions:
        new_array[tuple(pos)] = 1
    return new_array
