"""
    Workflow: 
        - Get map from simulation 
        - Use art gallery to get optimal starting positions
        - Use two-pass over map
        - Use DARP to partition map
        - Use Kruskal to generate global plans
"""
from darp import DARP
# For now, generate random graph
from graph_gen import generate_connected_zero_mass, place_enemies_in_map
from art_gallery import place_agents
from two_pass import analyze_connected_regions
import numpy as np
import matplotlib.pyplot as plt



