import pygame
import sys
import random
import math
import heapq

# --- CONFIGURATION ---
GRID_W = 50
GRID_H = 50
CELL_SIZE = 15
SCREEN_WIDTH = GRID_W * CELL_SIZE
SCREEN_HEIGHT = GRID_H * CELL_SIZE
FPS = 5

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 200, 0)
BLUE  = (0, 0, 255)
RED   = (255, 50, 50)
ORANGE = (255, 165, 0)
YELLOW = (255, 255, 0)
GREY = (200, 200, 200)
BROWN = (139, 69, 19)    # Normal Builder
CYAN = (0, 255, 255)     # Messenger
DARK_GREY = (100, 100, 100) # Retired
DARK_GREEN = (0, 100, 0)    # Mission Robot
LIGHT_BLUE = (173, 216, 230) # Planned Path

# Cell Types
EMPTY = 0
START = 1
GOAL = 2
HAZARD = 3
DEPOT = 4
SAFE = 5

# --- A* PATHFINDING ---
def find_path_astar(start, goal, env_grid, retired_set):
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        current = heapq.heappop(open_set)[1]

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        x, y = current
        neighbors = [
            (x, y-1), (x, y+1), (x-1, y), (x+1, y),
            (x-1, y-1), (x+1, y-1), (x-1, y+1), (x+1, y+1)
        ]

        for nx, ny in neighbors:
            if 0 <= nx < GRID_W and 0 <= ny < GRID_H:
                # OBSTACLE CHECK:
                if env_grid[nx][ny] == HAZARD: continue
                if (nx, ny) in retired_set: continue

                tentative_g_score = g_score[current] + 1
                if tentative_g_score < g_score.get((nx, ny), float('inf')):
                    came_from[(nx, ny)] = current
                    g_score[(nx, ny)] = tentative_g_score
                    f = tentative_g_score + heuristic((nx, ny), goal)
                    f_score[(nx, ny)] = f
                    heapq.heappush(open_set, (f, (nx, ny)))
    return None

class MissionRobot:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.path = []
        self.move_timer = 0 
        self.state = "WAITING" 

    def update(self, env, retired_positions, active_positions):
        if self.state == "DONE": return

        self.move_timer += 1
        if self.move_timer < 2:
            return
        self.move_timer = 0

        goal_pos = (49, 0)
        
        # 1. PLAN PATH
        self.path = find_path_astar((self.x, self.y), goal_pos, env.grid, retired_positions)

        if self.path:
            self.state = "MOVING"
            next_step = self.path[0] 
            
            # 2. TRAFFIC CHECK
            # Wait if ANY active robot is in the way
            if next_step in active_positions:
                pass 
            else:
                self.x, self.y = next_step
                if env.grid[self.x][self.y] == GOAL:
                    self.state = "DONE"
                    print("MISSION SUCCESS!")
        else:
            self.state = "WAITING"
            self.path = [] # Clear path if none found

    def draw(self, surface):
        cx = self.x * CELL_SIZE + CELL_SIZE // 2
        cy = self.y * CELL_SIZE + CELL_SIZE // 2
        
        # Draw Hollow Dark Green Diamond
        half_size = CELL_SIZE // 2 - 2
        points = [
            (cx, cy - half_size),           # Top
            (cx + half_size, cy),           # Right
            (cx, cy + half_size),           # Bottom
            (cx - half_size, cy)            # Left
        ]
        pygame.draw.lines(surface, DARK_GREEN, True, points, 2)

class Builder:
    def __init__(self, x, y, id):
        self.id = id
        self.x = x
        self.y = y
        self.has_brick = False
        self.state = "WANDER"
        
        angle = random.uniform(0, 2 * math.pi)
        self.heading_x = math.cos(angle)
        self.heading_y = math.sin(angle)
        self.bias_strength = 0.0 
        self.robots_informed = 0

    def update(self, env, occupied_positions, all_agents):
        if self.state == "RETIRED": return

        if self.state == "MESSENGER":
            if self.robots_informed >= 5:
                if env.grid[self.x][self.y] in [EMPTY, DEPOT, START]:
                    self.state = "RETIRED"
                    return
            
            neighbors = self.get_neighbors()
            for nx, ny in neighbors:
                for other in all_agents:
                    if other.id != self.id and other.x == nx and other.y == ny:
                        self.robots_informed += 1
                        if other.state == "WANDER": other.state = "MESSENGER"

        if self.state == "WANDER":
            neighbors = self.get_neighbors()
            for nx, ny in neighbors:
                if 0 <= nx < GRID_W and 0 <= ny < GRID_H:
                    if env.grid[nx][ny] == GOAL:
                        self.state = "MESSENGER"
                        print(f"Robot {self.id} found GOAL! Epidemic Started.")
                        return
                
                for other in all_agents:
                    if other.id != self.id and other.x == nx and other.y == ny:
                        if other.state == "RETIRED":
                            self.state = "MESSENGER"
                            return

        self.update_heading(env)
        self.move(env, occupied_positions)

    def update_heading(self, env):
        repulsion_x, repulsion_y = 0, 0
        collision = False
        neighbors = self.get_neighbors()
        
        for nx, ny in neighbors:
            if not (0 <= nx < GRID_W and 0 <= ny < GRID_H):
                repulsion_x += (self.x - nx); repulsion_y += (self.y - ny)
                collision = True; continue
            
            cell = env.grid[nx][ny]
            should_repulse = False
            
            if self.state == "MESSENGER":
                if cell == HAZARD: should_repulse = True
            elif self.state == "WANDER":
                if self.has_brick:
                    if cell == HAZARD:
                        self.try_build(env, nx, ny)
                        should_repulse = True 
                else:
                    if cell in [HAZARD, SAFE]: should_repulse = True
                    if cell == DEPOT: self.has_brick = True

            if should_repulse:
                repulsion_x += (self.x - nx); repulsion_y += (self.y - ny)
                collision = True

        if collision:
            self.bias_strength = 1.0
            self.heading_x += repulsion_x; self.heading_y += repulsion_y
            l = math.sqrt(self.heading_x**2 + self.heading_y**2)
            if l!=0: self.heading_x /= l; self.heading_y /= l
        else:
            self.bias_strength = max(0.0, self.bias_strength - 0.05)

    def move(self, env, occupied_positions):
        neighbors = self.get_neighbors()
        valid_moves = []
        for nx, ny in neighbors:
            if 0 <= nx < GRID_W and 0 <= ny < GRID_H:
                if env.grid[nx][ny] != HAZARD:
                    if (nx, ny) not in occupied_positions:
                        valid_moves.append((nx, ny))
        
        if valid_moves:
            weights = []
            for mx, my in valid_moves:
                w = 1.0
                if self.bias_strength > 0:
                    dx = mx - self.x; dy = my - self.y
                    align = dx*self.heading_x + dy*self.heading_y
                    if align > 0:
                        bonus = 20.0 * self.bias_strength
                        if align > 0.5: w += bonus
                        else: w += (bonus/4)
                weights.append(w)
            
            best = random.choices(valid_moves, weights=weights, k=1)[0]
            occupied_positions.discard((self.x, self.y))
            self.x, self.y = best
            occupied_positions.add((self.x, self.y))
            
            if self.state == "WANDER" and env.grid[self.x][self.y] == DEPOT and not self.has_brick:
                self.has_brick = True

    def try_build(self, env, nx, ny):
        if self.state != "WANDER": return
        n = []
        for dx in [-1,0,1]:
            for dy in [-1,0,1]:
                if dx==0 and dy==0: continue
                n.append((nx+dx, ny+dy))
        
        supported = False
        for bx, by in n:
            if 0 <= bx < GRID_W and 0 <= by < GRID_H:
                if env.grid[bx][by] in [START, SAFE]: supported = True; break
        
        if not supported and random.random() < 0.01: supported = True 
        
        if supported:
            env.grid[nx][ny] = SAFE
            self.has_brick = False

    def get_neighbors(self):
        n = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0: continue
                n.append((self.x + dx, self.y + dy))
        return n

    def draw(self, surface):
        x_pos = self.x * CELL_SIZE
        y_pos = self.y * CELL_SIZE
        padding = 3
        col = BROWN
        if self.state == "MESSENGER": col = CYAN
        elif self.state == "RETIRED": col = DARK_GREY
        
        pygame.draw.line(surface, col, (x_pos + padding, y_pos + padding), 
                         (x_pos + CELL_SIZE - padding, y_pos + CELL_SIZE - padding), 3)
        pygame.draw.line(surface, col, (x_pos + padding, y_pos + CELL_SIZE - padding), 
                         (x_pos + CELL_SIZE - padding, y_pos + padding), 3)
        
        if self.has_brick and self.state == "WANDER":
            cx = x_pos + CELL_SIZE // 2; cy = y_pos + CELL_SIZE // 2; r = CELL_SIZE // 4
            pygame.draw.circle(surface, ORANGE, (cx, cy), r)

class Environment:
    def __init__(self):
        self.width = GRID_W; self.height = GRID_H
        self.grid = [[EMPTY for _ in range(self.height)] for _ in range(self.width)]
        self.setup_landscape()
        self.agents = []
        poss = []
        for x in range(1, 15):
            for y in range(40, 49): poss.append((x, y))
        chosen = random.sample(poss, 30)
        for i in range(30):
            self.agents.append(Builder(chosen[i][0], chosen[i][1], i))
        
        self.mission_robot = MissionRobot(0, 49)

    def setup_landscape(self):
        for x in range(self.width):
            for y in range(self.height): self.grid[x][y] = EMPTY
        
        self.grid[0][self.width-1] = START
        self.grid[self.height-1][0] = GOAL

        line_start_x, line_start_y = self.width // 3, self.height // 10
        line_end_x, line_end_y = self.width - (self.width // 10), self.height - (self.height // 3)
        river_thickness = 12
        for x in range(self.width):
            for y in range(self.height):
                dist = self._distance_point_to_segment(x, y, line_start_x, line_start_y, line_end_x, line_end_y)
                if dist < river_thickness / 2:
                    if self.grid[x][y] not in [START, GOAL]:
                        self.grid[x][y] = HAZARD

        placed = 0; attempts = 0
        while placed < 15 and attempts < 2000:
            attempts += 1
            dx = random.randint(2, 40); dy = random.randint(10, 48)
            can_place = True
            for i in range(2):
                for j in range(2):
                    if not (0<=dx+i<GRID_W and 0<=dy+j<GRID_H and self.grid[dx+i][dy+j]==EMPTY):
                        can_place = False; break
                if not can_place: break
            if can_place:
                for i in range(2): 
                    for j in range(2): self.grid[dx+i][dy+j] = DEPOT
                placed += 1

    def _distance_point_to_segment(self, px, py, x1, y1, x2, y2):
        line_len_sq = (x2 - x1)**2 + (y2 - y1)**2
        if line_len_sq == 0: return math.sqrt((px - x1)**2 + (py - y1)**2)
        t = max(0, min(1, ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / line_len_sq))
        return math.sqrt((px - (x1 + t * (x2 - x1)))**2 + (py - (y1 + t * (y2 - y1)))**2)

    def update(self):
        occupied = set()
        for a in self.agents:
            occupied.add((a.x, a.y))
        
        # Mission bot is occupied too
        occupied.add((self.mission_robot.x, self.mission_robot.y))

        for a in self.agents: 
            a.update(self, occupied, self.agents)
        
        # NEW: Re-calculate positions for Mission Robot so it sees current state
        active_pos = set()
        retired_pos = set()
        for a in self.agents:
            if a.state == "RETIRED":
                retired_pos.add((a.x, a.y))
            else:
                active_pos.add((a.x, a.y))
        
        self.mission_robot.update(self, retired_pos, active_pos)

    def draw(self, surface):
        for x in range(self.width):
            for y in range(self.height):
                r = (x*CELL_SIZE, y*CELL_SIZE, CELL_SIZE, CELL_SIZE)
                t = self.grid[x][y]
                c = WHITE
                if t==START: c=GREEN
                elif t==GOAL: c=BLUE
                elif t==HAZARD: c=RED
                elif t==DEPOT: c=ORANGE
                elif t==SAFE: c=YELLOW
                pygame.draw.rect(surface, c, r); pygame.draw.rect(surface, GREY, r, 1)
        
        # Draw Planned Path
        if self.mission_robot.path:
            for (px, py) in self.mission_robot.path:
                pygame.draw.rect(surface, LIGHT_BLUE, (px * CELL_SIZE + 2, py * CELL_SIZE + 2, CELL_SIZE - 4, CELL_SIZE - 4))

        for a in self.agents: a.draw(surface)
        self.mission_robot.draw(surface)

def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    clock = pygame.time.Clock()
    
    font = pygame.font.Font(None, 36)

    env = Environment()
    run = True
    paused = True # Start Paused

    while run:
        for e in pygame.event.get():
            if e.type == pygame.QUIT: run = False
            
            if e.type == pygame.MOUSEBUTTONDOWN: 
                mx, my = pygame.mouse.get_pos()
                gx, gy = mx//CELL_SIZE, my//CELL_SIZE
                if env.grid[gx][gy] == HAZARD: env.grid[gx][gy] = SAFE
            
            if e.type == pygame.KEYDOWN:
                if e.key == pygame.K_SPACE:
                    paused = not paused
        
        if not paused:
            env.update()
        
        screen.fill(BLACK)
        env.draw(screen)

        if paused:
            text_surf = font.render("PAUSED (Press Space)", True, WHITE)
            text_rect = text_surf.get_rect(center=(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2))
            box_rect = text_rect.inflate(20, 20)
            pygame.draw.rect(screen, BLACK, box_rect)
            pygame.draw.rect(screen, WHITE, box_rect, 2)
            screen.blit(text_surf, text_rect)

        pygame.display.set_caption(f"Swarm Sim | {'PAUSED' if paused else 'RUNNING'}")
        pygame.display.flip()
        clock.tick(FPS)
    pygame.quit(); sys.exit()

if __name__ == "__main__": main()