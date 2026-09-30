# KPP PELATIHAN ROBOTIK ITS - Capture the Flag: Robotic Pathfinding Project

## Overview
This project features an autonomous robotic programming and pathfinding system designed to play a "Capture the Flag" style game. The robot navigates through different grid-based maps to locate and capture the flag while dynamically avoiding obstacles and restricted zones 

## Map & Grid System
The environment is divided into a grid system. The robot is programmed to navigate three distinct maps, each with varying layouts and difficulties. 

The grid consists of three types of squares:
*   🟩 **Stable Square:** Safe, open terrain. The robot can move freely through these spaces.
*   🧱 **Wall Square:** A physical obstacle. The robot cannot pass through or see through these squares.
*   ❌ **X Square (Restricted Zone):** A hazard, trap, or restricted area. The robot is strictly programmed to avoid these squares and cannot pass through them.

## Features
*   **3 Unique Maps:** Three different environmental layouts to test the robot's pathfinding capabilities.
*   **Autonomous Pathfinding:** Implements pathfinding algorithms (e.g., A*, Dijkstra's, or BFS) to calculate the most efficient route to the flag.
*   **Obstacle Avoidance:** Logic to detect and route around Walls and X Squares.
*   **Capture Logic:** Recognizes the target (flag) location and successfully navigates to it to trigger a win state.

## Getting Started

### Prerequisites
*(Note: Update this section based on your specific tech stack)*
*   Python 3.x (or your primary programming language)
*   Robotics simulator (e.g., ROS, Webots, CoppeliaSim) OR physical robot hardware specifics.
*   Required libraries (e.g., `numpy`, `matplotlib` for map visualization)

### Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/ctf-pathfinding-robot.git
   ```
2. Navigate to the project directory:
   ```bash
   cd ctf-pathfinding-robot
   ```
3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Usage
To run the simulation/robot on a specific map, execute the main script and pass the map number as an argument:

```bash
# Run Map 1
python main.py --map 1

# Run Map 2
python main.py --map 2

# Run Map 3
python main.py --map 3
```

## How It Works
1. **Environment Parsing:** The program reads the selected map data, translating the visual map into a 2D array or graph consisting of Stable, Wall, and X squares.
2. **Path Calculation:** The pathfinding algorithm evaluates the grid, assigning heavy or infinite movement penalties to Walls and X Squares to ensure the robot routes around them.
3. **Execution:** The robot follows the generated waypoints step-by-step from its starting coordinate to the flag's coordinate. 

## Future Improvements
*   Implement dynamic moving obstacles.
*   Add a multi-agent system (multiple robots competing for the flag).
*   Integrate real-time sensor data mapping for unknown environments.

## License
Distributed under the MIT License. See `LICENSE` for more information.
