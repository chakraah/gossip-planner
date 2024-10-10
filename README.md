# Multi-Robot Gossip Task Planner for Inspection and Monitoring

----------------------------------------------------------------------------------

This Python project simulates a multi-robot mission for inspection and monitoring in an industrial environment. In this mission, multiple mobile robots, each equipped with specific sensors, are tasked with performing measurements that are spatially distributed across various locations. This README provides instructions on how to use the project and run the simulations.

## Description

**Here is a brief description of the provided functions:**

* main.py: Main script that simulates a random mission and visualizes the cost evolution over time.
* scripts/: Contains different experiment files for specific simulations:
  * simulation: Runs simulations with scenarios that range from 5 to 20 robots and tasks from 10 to 40.
  * random_scenario_generator.py: Generates a random scenario.
  * scripts/src/: Contains the core of the algorithm:
    * TaskPlanner: Provides functions necessary for task planning, such as generating random solutions, calculating costs, 2-opt local search optimization, and task exchange mechanism between robots.
    * gossip_algorithm: Implements the Gossip Algorithm.
    * MILP_solver: Compute the lower bound of the optimal solution based on the linear relaxation of the MILP solver


## Reference

