---

# Motion Planning for arm Manipulator

This repository implements a motion planning pipeline for a mobile manipulator navigating a dynamic factory floor. The project focuses on decoupled motion planning for the robot's base and arm, employing RRT-based algorithms for efficient pathfinding in complex environments.

---

## Project Overview

Effective motion planning is vital with the rise of autonomous robots in industrial settings. This project presents a solution for a mobile manipulator tasked with navigating a simulated factory floor and performing precise manipulations at its destination.

---

### Features

- **Arm (3D Workspace)**: Implements RRT, RRT*, and Wrapped-RRT* for efficient pathfinding and optimization.  
- **Dynamic Collision Avoidance**: Detects obstacles and applies path smoothing for refined navigation.  
- **Algorithm Performance**: Evaluates RRT, RRT*, and Wrapped-RRT* based on success rate, path length, and computational efficiency.

---

## Robot Specifications

- **Manipulator**: 6-DOF KUKA iiwa robotic arm, operating in a 3D workspace (R⁶).

---

## Algorithms

- **RRT**: Baseline algorithm focusing on feasibility and exploration.  
- **RRT***: Asymptotically optimal algorithm with path rewiring for improved solutions.  
- **Wrapped-RRT***: A computationally efficient variant of RRT* that optimizes the final path.

---

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/your-username/motion-planning-manipulator.git
   cd motion-planning-manipulator
   ```
2. Install required dependencies:
   ```bash
   pip install -r requirements.txt
   ```

---

## Usage

1. Run the simulation:
   ```bash
   python simulation.py
   ```
2. Adjust configurations (e.g., obstacle layout, robot parameters) in the `config/` directory.

---

## Results

### Pathfinding Visualizations
- **RRT**:  
  ![RRT](https://github.com/junglator/iiwa_motion_planning/assets/46628685/a13e75ab-e825-433e-a222-1f17d59b4c42)  

- **RRT***:  
  ![RRTStar](https://github.com/junglator/iiwa_motion_planning/assets/46628685/926e05b2-b0b1-4f88-8832-0d388bf886c9)  

- **Wrapped-RRT***:  
  ![RRTWrapped](https://github.com/junglator/iiwa_motion_planning/assets/46628685/2b2af872-382c-4350-b2d2-d3bd46a010b9)  

---

## References

1. T. Sandakalum and M. H. Ang, “Motion Planning for Mobile Manipulators—A Systematic Review,” *Machines*, vol. 10, no. 2, p. 97, 2022.  
2. N. Castaman et al., “Receding horizon task and motion planning in changing environments,” *Robotics and Autonomous Systems*, vol. 145, p. 103863, 2021.  
