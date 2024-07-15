from graph_gen import generate_connected_zero_mass
from art_gallery import place_agents
from a_star_new import astar
from darp import DARP
import matplotlib.pyplot as plt
from planner import MultiRobotPathPlanner
import numpy as np

def to_1d_index(row, column, num_columns):
    return row * num_columns + column

rows, cols = 50, 50
zero_fraction = 0.9  # Size of the zero block
NUM_AGENTS = 4
size = 1 / NUM_AGENTS
portions = [size] * NUM_AGENTS
# Generate the array
occupancy_map = generate_connected_zero_mass(rows, cols, zero_fraction)
plt.imshow(occupancy_map);
plt.show();
obstacles = np.argwhere(occupancy_map == 1)

init_pos = place_agents(occupancy_map, num_agents=4, fov_radius=8)
obs_pos = [to_1d_index(pos[0], pos[1], cols) for pos in obstacles]
start_pos = [to_1d_index(pos[0], pos[1], cols) for pos in init_pos]

planner = MultiRobotPathPlanner(nx=rows, ny=cols, notEqualPortions=False, initial_positions=start_pos, portions=portions, 
                        obs_pos=obs_pos, visualization=True,
                        MaxIter=80000, CCvariation=0.01,
                        randomLevel=0.0001, dcells=2, importance=False)
 