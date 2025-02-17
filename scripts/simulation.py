# -*- coding: utf-8 -*-
"""
Created on Fri Oct  4 15:34:37 2024

@author: Chakraa
"""

import time
import numpy as np

from random_scenario_generator import random_scenario_generator
from src.gossip_algorithm import gossip_algorithm
from src.MILP_solver import MILP_solver
from src.task_planner import TaskPlanner

def monte_carlo_simulation(scenario, num_simulations):

    # Upper bound initialisation
    upper_bound = 0

    # Run Monte Carlo simulations
    for _ in range(num_simulations):
        # Random task-to-robot assignment
        current_solution = scenario.generate_random_solution()
        # Capture the last cost from the costs over iterations
        upper_bound_new = scenario.compute_solution_cost(current_solution)

        # Update the upper bound
        upper_bound = upper_bound + upper_bound_new

    upper_bound = upper_bound / num_simulations

    return upper_bound

def execute_scenario(planner):
    """Dynamically run the scenario module and return the average costs and execution time."""
    # Dynamically import the specified scenario module
    
    costs = []  # To store the cost for each iteration
    iterations = []
    # Execute the scenario multiple times for average cost calculation
    for _ in range(10):
        start_time = time.time()
        improved_solution, costs_over_iterations, _, iteration = gossip_algorithm(planner, 50)
        # Capture the last cost from the costs over iterations
        final_cost = costs_over_iterations[-1]
        costs.append(final_cost)
        iterations.append(iteration)
        end_time = time.time()
    
    # Calculate the mean cost
    average_cost = np.mean(costs)
    average_iterations = np.mean(iterations)

    # Calculate total elapsed time for the last run
    total_elapsed_time = end_time - start_time

    return {
        'Average Cost': average_cost,
        'Elapsed Time': total_elapsed_time,
        'Average iterations': average_iterations,
    }

def simulation():
    
    task_requirements, robot_capabilities, cost_matrix = random_scenario_generator(1, 1, 30, 5)

    robots = 20
    tasks = 40
    # Lists to hold values for the 3D plot
    robot_counts = []
    task_counts = []
    results1 = []  # To store results of each scenario
    milp_costs = []
    upper_bound = []
    for i in range(4, robots):
        for j in range (9, tasks):
            print(f"Processing for {i+1} robots and {j+1} tasks...")
                    
            task_requirements, robot_capabilities, x = random_scenario_generator(i+1, j+1, 30, 5)
            planner = TaskPlanner(task_requirements, robot_capabilities, cost_matrix)
            result1 = execute_scenario(planner)
            results1.append(result1)
            milp_costs.append(MILP_solver(planner))
            up = monte_carlo_simulation(planner, 50)
            #up = upper_bound_calculation(i+1, 5, 30, robot_capabilities, cost_matrix)
            upper_bound.append(up)

            # Store robot and task numbers
            robot_counts.append(i+1)
            task_counts.append(j+1)
    
    # Data for plotting the results
    gossip_costs1 = [result1['Average Cost'] for result1 in results1]
    gossip_times1 = [result1['Elapsed Time'] for result1 in results1]
    gossip_iterations1 = [result1['Average iterations'] for result1 in results1]

    with open('output.txt', 'w') as file:
        file.write("x= " + str(robot_counts) + '\n')
        file.write("y= " + str(task_counts) + '\n')
        file.write("z1= " + str(gossip_costs1) + '\n')
        file.write("z2= " + str(milp_costs) + '\n')
        file.write("z3= " + str(upper_bound) + '\n')
        file.write("z5= " + str(gossip_times1) + '\n')
        file.write("z6= " + str(gossip_iterations1) + '\n')
    
    return robot_counts, task_counts