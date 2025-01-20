# Heuristic Search Assignment

This project implements solutions that explores heuristic search algorithms for pathfinding. The project consists of two tasks:  
1. Implementing the A* algorithm to find the least-cost path considering terrain difficulty.  
2. Enhancing the A* algorithm to consider both terrain difficulty and success probability in an enemy-controlled territory.

---

## Features
- **Task 1**: Calculates the least-cost path from a start location to a headquarters (HQ) on a grid map while accounting for terrain difficulty and a difficulty threshold.
- **Task 2**: Adds considerations for enemy presence, calculating paths that meet both a cost threshold and a success probability threshold.

The program adheres to strict requirements for input formats and Python version compatibility. Both tasks utilize the A* search algorithm with user-defined heuristic functions.

---

## Prerequisites
- **Python version**: 3.10 or higher  
- **Dependencies**: Install from `requirements.txt`  
  ```bash
  pip install -r requirements.txt

## How to Run
- **Task 1**:
  - python pathfinding_task1.py <start_row>,<start_col> <hq_row>,<hq_col> <terrain_file> <difficulty_threshold>
  - python pathfinding_task1.py 3,2 0,3 resources/terrain01.txt 50

- **Task 2**:
  - python safe_pathfinding_task2.py <start_row>,<start_col> <hq_row>,<hq_col> <terrain_file> <difficulty_threshold> <enemy_file> <success_threshold>
  - python safe_pathfinding_task2.py 3,2 0,3 resources/terrain01.txt 50 resources/enemy01.txt 1.0


