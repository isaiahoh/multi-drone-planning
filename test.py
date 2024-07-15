from graph_gen import generate_connected_zero_mass
from art_gallery import place_agents
from a_star_new import astar
from darp import DARP
import matplotlib.pyplot as plt
from planner import MultiRobotPathPlanner
import numpy as np

rows, cols = 50, 50
zero_fraction = 0.9  # Size of the zero block
NUM_AGENTS = 4
size = 1 / NUM_AGENTS
portions = [size] * NUM_AGENTS
# Generate the array
occupancy_map = generate_connected_zero_mass(rows, cols, zero_fraction)
obstacles = np.argwhere(occupancy_map == 1)
init_pos = place_agents(occupancy_map, num_agents=4, fov_radius=8)

planner = MultiRobotPathPlanner(nx=rows, ny=cols, notEqualPortions=False, initial_positions=init_pos, portions=portions, 
                        obs_pos=obstacles, visualization=True,
                        MaxIter=80000, CCvariation=0.01,
                        randomLevel=0.0001, dcells=2, importance=False)
 