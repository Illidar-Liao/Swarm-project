import pygame
import sys
import random
import math

# --- CONFIGURATION ---
# Screen & Grid Settings
GRID_W = 50
GRID_H = 50
CELL_SIZE = 15  # Pixels per cell
SCREEN_WIDTH = GRID_W * CELL_SIZE
SCREEN_HEIGHT = GRID_H * CELL_SIZE
FPS = 30

# Colors (R, G, B)
WHITE = (255, 255, 255)  # EMPTY
BLACK = (0, 0, 0)        # Lines
GREEN = (0, 200, 0)      # START
BLUE  = (0, 0, 255)      # GOAL
RED   = (255, 50, 50)    # HAZARD (The Arc)
ORANGE = (255, 165, 0)   # DEPOT
YELLOW = (255, 255, 0)   # SAFE (Bridge)
GREY = (200, 200, 200)   # Grid Lines
ROBOT_COLOR = (50, 50, 50) # Dark Grey for robots
BROWN = (139, 69, 19)    # Builder Color

# Cell Types
EMPTY = 0
START = 1
GOAL = 2
HAZARD = 3
DEPOT = 4
SAFE = 5

class Builder:
    def __init__(self, x, y, id):
        self.id = id
        self.x = x
        self.y = y
        self.has_brick = False
        self.state = "WANDER" 

    def update(self, env, occupied_positions):
        """
        PURE SWARM LOGIC:
        No Compass. No Global Knowledge. 
        Just random movement + Local Stigmergy + Nucleation.
        """
        
        # 1. SCAN SURROUNDINGS (Local Sensing)
        neighbors = [
            (self.x, self.y - 1), (self.x, self.y + 1),
            (self.x - 1, self.y), (self.x + 1, self.y),
            (self.x - 1, self.y - 1), (self.x + 1, self.y - 1),
            (self.x - 1, self.y + 1), (self.x + 1, self.y + 1)
        ]

        valid_moves = []
        
        for nx, ny in neighbors:
            # A. Boundary Check
            if 0 <= nx < GRID_W and 0 <= ny < GRID_H:
                cell_type = env.grid[nx][ny]
                
                # B. Obstacle Check
                if cell_type != HAZARD:
                    # C. Agent Collision Check (Optimized O(1) Lookup)
                    if (nx, ny) not in occupied_positions:
                        valid_moves.append((nx, ny))

                # D. INTERACTION: BUILD LOGIC (Nucleation & Stigmergy)
                if self.has_brick and cell_type == HAZARD:
                    # 1. Check for Support (Stigmergy)
                    has_support = False
                    
                    # Look at neighbors of the TARGET HAZARD tile (nx, ny)
                    brick_neighbors = [
                        (nx, ny - 1), (nx, ny + 1),
                        (nx - 1, ny), (nx + 1, ny),
                        (nx - 1, ny - 1), (nx + 1, ny - 1),
                        (nx - 1, ny + 1), (nx + 1, ny + 1)
                    ]
                    
                    for bx, by in brick_neighbors:
                        if 0 <= bx < GRID_W and 0 <= by < GRID_H:
                            # Support comes from START or existing Bridge (SAFE)
                            if env.grid[bx][by] in [START, SAFE]: 
                                has_support = True
                                break
                    
                    # 2. Decision Logic
                    should_build = False
                    
                    if has_support:
                        # CASE A: STIGMERGY (High Priority)
                        should_build = True
                    else:
                        # CASE B: NUCLEATION (Low Priority)
                        # 1% chance per frame to start a new bridge section
                        if random.random() < 0.05: 
                            should_build = True
                            print(f"Robot {self.id}: Placed a Lucky Brick (Seed)!")

                    # Execute
                    if should_build:
                        env.grid[nx][ny] = SAFE 
                        self.has_brick = False
                        return 

        # 2. MOVEMENT LOGIC (Pure Random Walk + Local Resource Attraction)
        if valid_moves:
            best_move = None
            
            # PRIORITY: If I see a DEPOT and don't have a brick, go there!
            if not self.has_brick:
                for mx, my in valid_moves:
                    if env.grid[mx][my] == DEPOT:
                        best_move = (mx, my)
                        break
            
            # If no priority move, pick random (Brownian Motion)
            if best_move is None:
                best_move = random.choice(valid_moves)
            
            # Execute Move: Update Position and Occupancy Set
            occupied_positions.remove((self.x, self.y))
            self.x, self.y = best_move
            occupied_positions.add((self.x, self.y))

            # E. INTERACTION: PICKUP LOGIC
            if env.grid[self.x][self.y] == DEPOT and not self.has_brick:
                self.has_brick = True

    def draw(self, surface):
        x_pos = self.x * CELL_SIZE
        y_pos = self.y * CELL_SIZE
        padding = 3
        
        # Draw robot body as a Brown Cross (X)
        pygame.draw.line(surface, BROWN, (x_pos + padding, y_pos + padding), 
                         (x_pos + CELL_SIZE - padding, y_pos + CELL_SIZE - padding), 3)
        pygame.draw.line(surface, BROWN, (x_pos + padding, y_pos + CELL_SIZE - padding), 
                         (x_pos + CELL_SIZE - padding, y_pos + padding), 3)
        
        # Draw "Brick" indicator
        if self.has_brick:
            cx = x_pos + CELL_SIZE // 2
            cy = y_pos + CELL_SIZE // 2
            radius = CELL_SIZE // 4
            pygame.draw.circle(surface, ORANGE, (cx, cy), radius)

class Environment:
    def __init__(self):
        self.width = GRID_W
        self.height = GRID_H
        self.grid = [[EMPTY for _ in range(self.height)] for _ in range(self.width)]
        self.setup_landscape()
        
        # Spawn 10 Robots near Start (Scattered)
        self.agents = []
        for i in range(10):
            spawn_x = 2 + (i % 3)
            spawn_y = 45 + (i // 3)
            self.agents.append(Builder(spawn_x, spawn_y, i))

    def setup_landscape(self):
        # 1. START
        self.grid[0][self.width-1] = START
        # 2. GOAL
        self.grid[self.height-1][0] = GOAL

        # 3. DEPOT (Two 5x5 piles)
        depot_start_x = 9; depot_start_y = 42
        for x in range(depot_start_x, depot_start_x + 5):
            for y in range(depot_start_y, depot_start_y + 5):
                if 0 <= x < self.width and 0 <= y < self.height: self.grid[x][y] = DEPOT

        depot_start_x = 3; depot_start_y = 36 
        for x in range(depot_start_x, depot_start_x + 5):
            for y in range(depot_start_y, depot_start_y + 5):
                if 0 <= x < self.width and 0 <= y < self.height: self.grid[x][y] = DEPOT

        # 4. HAZARD (Diagonal River)
        line_start_x, line_start_y = self.width // 3, self.height // 10
        line_end_x, line_end_y = self.width - (self.width // 10), self.height - (self.height // 3)
        river_thickness = 12

        for x in range(self.width):
            for y in range(self.height):
                dist = self._distance_point_to_segment(x, y, line_start_x, line_start_y, line_end_x, line_end_y)
                if dist < river_thickness / 2:
                    if self.grid[x][y] not in [START, GOAL, DEPOT]:
                        self.grid[x][y] = HAZARD

    def _distance_point_to_segment(self, px, py, x1, y1, x2, y2):
        line_len_sq = (x2 - x1)**2 + (y2 - y1)**2
        if line_len_sq == 0: return math.sqrt((px - x1)**2 + (py - y1)**2)
        t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / line_len_sq
        t = max(0, min(1, t))
        closest_x = x1 + t * (x2 - x1)
        closest_y = y1 + t * (y2 - y1)
        return math.sqrt((px - closest_x)**2 + (py - closest_y)**2)

    def update(self):
        occupied_positions = set()
        for agent in self.agents:
            occupied_positions.add((agent.x, agent.y))

        for agent in self.agents:
            agent.update(self, occupied_positions)

    def draw(self, surface):
        for x in range(self.width):
            for y in range(self.height):
                rect = (x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
                cell_type = self.grid[x][y]
                color = WHITE
                if cell_type == START: color = GREEN
                elif cell_type == GOAL: color = BLUE
                elif cell_type == HAZARD: color = RED
                elif cell_type == DEPOT: color = ORANGE
                elif cell_type == SAFE: color = YELLOW
                
                pygame.draw.rect(surface, color, rect)
                pygame.draw.rect(surface, GREY, rect, 1)
        
        for agent in self.agents:
            agent.draw(surface)

def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Swarm Sim: Zero-Knowledge Builders")
    clock = pygame.time.Clock()

    env = Environment()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            if event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                gx, gy = mx // CELL_SIZE, my // CELL_SIZE
                if 0 <= gx < GRID_W and 0 <= gy < GRID_H:
                     if env.grid[gx][gy] == HAZARD:
                        env.grid[gx][gy] = SAFE

        env.update()
        
        screen.fill(BLACK)
        env.draw(screen)
        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()