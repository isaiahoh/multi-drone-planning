import pygame
import random
import numpy as np
import matplotlib.pyplot as plt
from two_pass import analyze_connected_regions
from a_star import find_path  # Import the A* function from our new module
from art_gallery import place_agents
from graph_gen import generate_connected_zero_mass

# Initialize Pygame
pygame.init()

# Constants
WIDTH, HEIGHT = 800, 800
DRONE_COUNT = 8
DRONE_RADIUS = 3
DRONE_SPEED = 2
FOV_RADIUS = 4
GRID_SIZE = 20

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

# Function to generate dark colors
def generate_dark_colors(n):
    colors = []
    for _ in range(n):
        r = random.randint(0, 128)
        g = random.randint(0, 128)
        b = random.randint(0, 128)
        
        if max(r, g, b) < 64:
            brightest = random.choice(['r', 'g', 'b'])
            if brightest == 'r':
                r = random.randint(64, 128)
            elif brightest == 'g':
                g = random.randint(64, 128)
            else:
                b = random.randint(64, 128)
        
        colors.append((r, g, b))
    
    return colors

# Generate colors for drones
DRONE_COLORS = generate_dark_colors(DRONE_COUNT)

# Set up the display
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Drone Swarm Art Gallery Simulation with A* Pathfinding")

class Drone:
    def __init__(self, x, y, color):
        self.x = x
        self.y = y
        self.color = color
        self.target_x = x
        self.target_y = y
        self.path = []
        self.current_path_index = 0
    
    def move_along_path(self):
        if self.current_path_index < len(self.path):
            target = self.path[self.current_path_index]
            dx = target[0] - self.x
            dy = target[1] - self.y
            distance = (dx**2 + dy**2)**0.5
            
            if distance > DRONE_SPEED:
                self.x += (dx / distance) * DRONE_SPEED
                self.y += (dy / distance) * DRONE_SPEED
            else:
                self.x, self.y = target
                self.current_path_index += 1
    
    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), DRONE_RADIUS)
        if len(self.path) > 1:
            pygame.draw.lines(screen, self.color, False, self.path, 1)

# occupancy_map = np.zeros((GRID_SIZE, GRID_SIZE), dtype=int)
occupancy_map = generate_connected_zero_mass(GRID_SIZE, GRID_SIZE, 0.8)


# def place_structured_obstacles(occupancy_map):
#     # Place vertical lines
#     for x in [5, 15]:
#         for y in range(GRID_SIZE):
#             if y % 3 != 0:  # Leave gaps every 3 cells
#                 occupancy_map[x, y] = 1
    
#     # Place horizontal lines
#     for y in [5, 15]:
#         for x in range(GRID_SIZE):
#             if x % 3 != 0:  # Leave gaps every 3 cells
#                 occupancy_map[x, y] = 1
    
#     # Place some random obstacles in the corners
#     corners = [(0, 0), (0, GRID_SIZE-1), (GRID_SIZE-1, 0), (GRID_SIZE-1, GRID_SIZE-1)]
#     for corner in corners:
#         for _ in range(3):
#             x = random.randint(corner[0], corner[0] + 3)
#             y = random.randint(corner[1], corner[1] + 3)
#             if 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE:
#                 occupancy_map[x, y] = 1

# place_structured_obstacles(occupancy_map)

# Store the original occupancy map for obstacle drawing
original_occupancy_map = occupancy_map.copy()

# Calculate initial positions using the art gallery algorithm
initial_positions = place_agents(occupancy_map, DRONE_COUNT, FOV_RADIUS)

print("Agent positions:", initial_positions)
for agent_pos in initial_positions:
    print(analyze_connected_regions(occupancy_map, agent_pos))

# Mark agent positions on the occupancy map (for visualization purposes only)
for count, pos in enumerate(initial_positions):
    x, y = pos
    occupancy_map[x, y] = 2 + count  # Use values 2 and above for agents

# Create drones in a line at the bottom of the window
drones = []
spacing = WIDTH // (DRONE_COUNT + 1)
for i in range(DRONE_COUNT):
    x = spacing * (i + 1)
    y = HEIGHT - DRONE_RADIUS * 2
    drone = Drone(x, y, DRONE_COLORS[i])
    target_x, target_y = initial_positions[i]
    drone.target_x = (target_x / GRID_SIZE) * WIDTH
    drone.target_y = (target_y / GRID_SIZE) * HEIGHT
    
    # Calculate path using A*
    start = (int(drone.y / (HEIGHT / GRID_SIZE)), int(drone.x / (WIDTH / GRID_SIZE)))
    goal = (target_y, target_x)
    path = find_path(start, goal, original_occupancy_map)
    
    if path is None:
        print(f"Warning: No path found for drone {i}. Using direct path.")
        path = [start, goal]
    
    # Convert path to screen coordinates
    drone.path = [(p[1] * (WIDTH / GRID_SIZE), p[0] * (HEIGHT / GRID_SIZE)) for p in path]
    
    drones.append(drone)

# Main game loop
running = True
clock = pygame.time.Clock()

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    # Move drones along their paths
    for drone in drones:
        drone.move_along_path()
    
    # Clear the screen
    screen.fill(WHITE)
    
    # Draw obstacles
    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            if original_occupancy_map[i, j] == 1:  # Use the original occupancy map for drawing obstacles
                rect = pygame.Rect(
                    i * (WIDTH // GRID_SIZE),
                    j * (HEIGHT // GRID_SIZE),
                    WIDTH // GRID_SIZE,
                    HEIGHT // GRID_SIZE
                )
                pygame.draw.rect(screen, BLACK, rect)
    
    # Draw drones
    for drone in drones:
        drone.draw(screen)
    
    # Update the display
    pygame.display.flip()
    
    # Cap the frame rate
    clock.tick(60)

# Quit Pygame
pygame.quit()

# Display the final positions using matplotlib
plt.figure(figsize=(8, 8))
plt.imshow(occupancy_map, cmap='viridis')  # Use a colormap that distinguishes obstacles and agents
for drone in drones:
    plt.plot(drone.x * GRID_SIZE / WIDTH, drone.y * GRID_SIZE / HEIGHT, 'ro')
    path = np.array(drone.path) * GRID_SIZE / WIDTH
    plt.plot(path[:, 0], path[:, 1], '-', color=drone.color)
plt.title("Final Drone Positions and Paths")
plt.colorbar(label='Cell Type (0: Free, 1: Obstacle, 2+: Agents)')
plt.show()
