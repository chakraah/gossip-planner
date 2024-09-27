# Multi-Robot Gossip Task Planner for Inspection and Monitoring

----------------------------------------------------------------------------------

This Python project simulates a multi-robot mission for inspection and monitoring in an industrial environment. In this mission, multiple mobile robots, each equipped with specific sensors, are tasked with performing measurements that are spatially distributed across various locations. This README provides instructions on how to use the project and run the simulations.

# Functions Description

**Here is a brief description of the provided functions:**

* main.py: Main script that simulates a random mission and visualizes the cost evolution over time.
* random_scenario_generator.py: Generates a random scenario.
* scripts/: Contains different experiment files for specific simulations:
  * experiment_6_robots: Runs simulations with 9 scenarios where the number of tasks varies but the number of robots is fixed at 6.
  * experiment_20_tasks: Runs simulations with 9 scenarios where the number of robots varies and the number of tasks is fixed at 20.
  * experiment_scaling: Runs simulations with 9 scenarios where both the number of robots and tasks increase.
  * scripts/src/: Contains the core of the algorithm:
    * TaskPlanner: rovides functions necessary for task planning, such as generating random solutions, calculating costs, 2-opt local search optimization, and task exchange mechanism between robots.
    * gossip_algorithm: Implements the Gossip Algorithm.


# Data Files

**The project includes three sets of pre-generated scenario data files, located in scripts/scenarios/. Each file contains 9 scenarios designed to study the effects of varying the number of tasks, robots, or both:**

* Data_6_robots: Scenarios with varying number of tasks and 6 robots.
* Data_20_robots: Scenarios with varying number of robots and 20 tasks.
* Data_scaling: Scenarios with both the number of robots and tasks increasing.


## Reference

