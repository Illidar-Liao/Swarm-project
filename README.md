Markdown
# SwarmWorld: Persistent vs. Pure Random Walk Simulation

**Author:** Sheng-Hong Liao (Illdar)
**Institution:** University of Minnesota, M.S. Robotics
**Advisor:** Prof. Maria Gini

## 📌 Overview
This repository contains a 2D multi-agent simulation built with Pygame for my Master's Capstone Project. The simulation models a swarm of "Builder" robots tasked with picking up resources (bricks) and building a safe path (bridge) across a hazard zone (river) so that a "Mission Robot" can successfully navigate to its goal using A* pathfinding. 

The project evaluates swarm efficiency, task allocation (searching vs. transporting), and trajectory algorithms by comparing a **Pure Random Walk** against a **Persistent Random Walk** (inertia-based movement). It also features a Human-Robot Interaction (HRI) mode.

## ⚙️ Environment Setup & Prerequisites

This simulation is written in Python and uses the `pygame` library for visualization.

### Installation
1. Ensure you have Python 3.x installed.
2. Install the required dependency:
   ```bash
   pip install pygame
🚀 How to Run the Simulation
Run the main Python script from your terminal:

Bash
python SwarmWorld.py
(Note: The simulation starts in a PAUSED state. Press SPACE to begin.)

🎮 Interactive Controls
You can interact with the simulation in real-time using the following controls:

Keyboard Controls:
SPACE: Play / Pause simulation.

R: Full Reset (rebuilds landscape and resets agents).

H: Hide/Show the swarm (useful for observing just the Mission Robot).

S: Save the current map layout to custom_map.csv.

L: Load a map layout (defaults to wide_river.csv).

T: Toggle the UI overlay.

A: Toggle HRI (Human-Robot Interaction) Anchor Mode.

Arrow Keys: Move the HRI Anchor (only works when HRI is active).

Mouse Controls (Map Editing):
You can dynamically alter the environment by clicking on grid cells:

Left Click: Cycles cell type -> Empty > Hazard > Safe > Depot

Right Click: Cycles cell type in reverse -> Depot > Safe > Hazard > Empty

🎛️ Adjusting Simulation Parameters
To test different algorithmic behaviors, you can adjust the following parameters directly inside SwarmWorld.py:

Pure vs. Persistent Random Walk (Inertia):
By default, the simulation runs a Pure Random Walk baseline. To enable the Persistent Random Walk, navigate to the Builder.move() method (around line 185) and uncomment the INERTIA section:

Python
# --- INERTIA ---
if self.bias_strength > 0:
    dx = mx - self.x; dy = my - self.y
    align = dx*self.heading_x + dy*self.heading_y
    if align > 0:
        bonus = 20.0 * self.bias_strength
        if align > 0.5: w += bonus
        else: w += (bonus/4)
Nucleation Rate (Bridge Building):
This controls the probability that a robot will drop a brick in the hazard zone without being supported by an existing safe tile.
Navigate to the Builder.try_build() method (around line 207). Change the < 0.1 value to adjust the 10% nucleation chance:

Python
if self.can_nucleate and not supported and random.random() < 0.1: 
HRI Attraction Strength:
When HRI is active (A key), robots holding bricks are drawn to the anchor. You can adjust the gravitational pull of the anchor in the Builder.move() method (around line 178) by changing w += 20.0.

📊 Data Output & Logging
Upon the Mission Robot successfully reaching the goal, the simulation will automatically pause and generate the following data logs in the root directory:

simulation_log.csv: Time series data of steps, built tiles, and hazards.

agent_durations.csv: A log of how many steps agents spent searching vs. transporting.

simulation_summary.txt: An overview of total time, averages, and active agents.

/trajectories/: A folder containing images of the specific paths taken by every individual Builder robot.