# RobTask
# Project Purpose

This project aims to address specific challenges in programming robotic systems,
focusing on simulation and algorithmic problem-solving for three key tasks:


Task 1: Localization in a 2D Torus World
A simulated robot navigates a 2D closed-world grid with two colors (R, G), following the toroidal topology. 
The robot must estimate its location based on probabilistic sensing and movements. Key objectives include:  
- Implementing a class `Robot` for scanning and moving.
- Computing the robot's probabilistic position distribution after following a pre-defined path and performing 
scans at each step.  
The solution effectively models real-world uncertainties in sensing and movement, with robust normalization 
to maintain probability consistency.



Task 2: Pathfinding in a Grid World 
Given a grid with obstacles, the task is to compute efficient paths between a start (`P`) and finish (`F`) point using:  
1. **Breadth-First Search (BFS)**: Exhaustively searches all paths and builds a cost matrix.  
2. **A\***: Implements heuristic-based optimization for finding the shortest path with reduced computation time.

The project evaluates the performance of these algorithms, providing visualization of paths and cost matrices.


Task 3: PID Controller Simulation  
Simulates a simplified robotic model with a 1D deviation, using a PID control algorithm:  
- Tests the controller under varying proportional (Kp), integral (Ki), and derivative (Kd) coefficients.  
- Models and visualizes system responses over time to analyze the impact of different parameters on error minimization.  

 Evaluation of Problem-Solving Efficiency

1. Task 1:  
   The implementation effectively simulates a probabilistic robot localization model. 
The solution adheres to real-world principles and ensures a systematic update of probabilities. 
Its performance scales well for the given constraints.  
   *Strengths*: Accurate representation of toroidal movement and probabilistic updates.  
   *Limitations*: Could benefit from scalability optimizations for larger worlds.

2. Task 2:  
   BFS and A\* both solve the pathfinding problem effectively. BFS is exhaustive but computationally expensive, 
   while A\* is more efficient due to its heuristic optimization. 
The visualized paths and cost matrices provide clear insights into the algorithms' performance.  
   *Strengths*: Comprehensive comparison of BFS and A\*.  
   *Limitations*: Scalability to larger or more complex grids may require additional optimizations.

3. Task 3:  
   The PID simulation demonstrates the controller's adaptability to parameter variations. 
   Visualization of results helps understand the impact of individual coefficients on system stability and error correction.  
   *Strengths*: Clear modeling and analysis of system responses.  
   *Limitations*: Assumes simplified linear behavior, which may not fully reflect nonlinear systems.



This project demonstrates a comprehensive approach to solving robotics programming challenges,
integrating algorithmic precision with practical simulations and visualizations.