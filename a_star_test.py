from graph_gen import generate_connected_zero_mass
from art_gallery import place_agents
from a_star_new import astar
from darp import DARP
import matplotlib.pyplot as plt
from planner import MultiRobotPathPlanner
def print_maze(maze, path):
    maze_path = [['_' if cell == 0 else '#' for cell in row] for row in maze]
    for position in path:
        maze_path[position[0]][position[1]] = '*'
    for row in maze_path:
        print(''.join(row))
        
rows, cols = 50, 50
zero_fraction = 0.9  # Size of the zero block
NUM_AGENTS = 4
size = 1 / NUM_AGENTS
portions = [size] * NUM_AGENTS
# Generate the array
occupancy_map = generate_connected_zero_mass(rows, cols, zero_fraction)
init_pos = place_agents(occupancy_map, num_agents=4, fov_radius=8)

astar_initial_pos = occupancy_map.copy();

# Find the path
for i, pos in enumerate(init_pos):
    path = astar(astar_initial_pos, (rows - 2 ,i*2), pos)
    for pos in path:
        astar_initial_pos[pos[0],pos[1]] = i * 15
        
# plt.imshow(astar_initial_pos)
# plt.colorbar()
# plt.show()
        
darp_instance = MultiRobotPathPlanner(nx=rows, ny=cols, notEqualPortions=False, given_initial_positions=init_pos, given_portions=portions, 
                        obstacles_positions=occupancy_map, visualization=False,
                        MaxIter=100, CCvariation=0.01,
                        randomLevel=0.0001, dcells=2, importance=False)
