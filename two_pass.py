import numpy as np
from collections import defaultdict

def find_root(label, parent):
    while parent[label] != label:
        label = parent[label]
    return label

def union(label1, label2, parent):
    root1 = find_root(label1, parent)
    root2 = find_root(label2, parent)
    if root1 != root2:
        parent[root2] = root1

def two_pass_labeling(occupancy_map):
    rows, cols = occupancy_map.shape
    labels = np.zeros_like(occupancy_map, dtype=int)
    next_label = 1
    parent = [0]

    # First pass
    for r in range(rows):
        for c in range(cols):
            if occupancy_map[r, c] == 1:
                neighbors = []
                if r > 0 and occupancy_map[r - 1, c] == 1:
                    neighbors.append(labels[r - 1, c])
                if c > 0 and occupancy_map[r, c - 1] == 1:
                    neighbors.append(labels[r, c - 1])

                if not neighbors:
                    labels[r, c] = next_label
                    parent.append(next_label)
                    next_label += 1
                else:
                    min_label = min(neighbors)
                    labels[r, c] = min_label
                    for neighbor in neighbors:
                        union(min_label, neighbor, parent)

    # Second pass
    for r in range(rows):
        for c in range(cols):
            if labels[r, c] > 0:
                labels[r, c] = find_root(labels[r, c], parent)

    return labels

def analyze_connected_regions(occupancy_map, agent_position):
    labeled_map = two_pass_labeling(occupancy_map)
    regions = defaultdict(int)
    
    # Count the size of each connected region
    for r in range(labeled_map.shape[0]):
        for c in range(labeled_map.shape[1]):
            if labeled_map[r, c] > 0:
                regions[labeled_map[r, c]] += 1

    # Determine the number of connected regions
    num_regions = len(regions)

    if num_regions == 1:
        return 0

    # Calculate the global average path length
    total_length = sum(regions.values())
    average_length = total_length / num_regions

    # Find the region of the agent's initial position
    agent_region = labeled_map[agent_position[0], agent_position[1]]
    agent_region_length = regions[agent_region]

    if agent_region_length >= average_length:
        return 0
    else:
        return 1
