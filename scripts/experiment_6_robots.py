"""
This program simulates multiple scenarios with six robots and varying numbers of tasks.
Author: H. Chakraa
"""

import importlib
import numpy as np
import time
import pandas as pd
import matplotlib.pyplot as plt
from src.gossip_algorithm import gossip_algorithm
from src.task_planner import TaskPlanner

def execute_scenario(module_name):
    """Dynamically run the scenario module and return the average costs and execution time."""
    # Dynamically import the specified scenario module
    scenario_module = importlib.import_module(module_name)
    
    # Initialize task and robot lists, and cost matrix from the module
    task_list = scenario_module.task_list
    robot_list = scenario_module.robot_list
    cost_matrix = scenario_module.cost_matrix
    
    costs = []  # To store the cost for each iteration
    # Execute the scenario multiple times for average cost calculation
    for _ in range(10):
        start_time = time.time()
        planner = TaskPlanner(task_list, robot_list, cost_matrix)
        improved_solution, costs_over_iterations, _ = gossip_algorithm(planner, 50)
        # Capture the last cost from the costs over iterations
        final_cost = costs_over_iterations[-1]
        costs.append(final_cost)
        end_time = time.time()
    
    # Calculate the mean cost
    average_cost = np.mean(costs)

    # Calculate total elapsed time for the last run
    total_elapsed_time = end_time - start_time

    return {
        'Scenario': module_name,
        'Average Cost': average_cost,
        'Elapsed Time': total_elapsed_time
    }

def main():
    # List of scenario modules to be processed
    scenario_modules = [
        'scenarios.data_6_robots.scenario1',
        'scenarios.data_6_robots.scenario2',
        'scenarios.data_6_robots.scenario3',
        'scenarios.data_6_robots.scenario4',
        'scenarios.data_6_robots.scenario5',
        'scenarios.data_6_robots.scenario6',
        'scenarios.data_6_robots.scenario7',
        'scenarios.data_6_robots.scenario8',
        'scenarios.data_6_robots.scenario9',
    ]

    results = []  # To store results of each scenario
    # Process each scenario module
    for module_name in scenario_modules:
        print(f"Processing {module_name}...")
        result = execute_scenario(module_name)
        results.append(result)

    # Create a DataFrame for the results
    results_df = pd.DataFrame(results)
    print("\nResults Table:")
    print(results_df)
    
    # Data for plotting the results
    task_counts = [10, 15, 20, 25, 30, 35, 40, 45, 50]
    gossip_costs = [result['Average Cost'] for result in results]
    milp_costs = [15.4, 18.73, 21.73, 26.067, 26.667, 28, 28, 28, 27]
    
    # Plot the results
    plt.figure(figsize=(10, 6))
    plt.plot(task_counts, gossip_costs, marker='o', color='b', label='Gossip Algorithm Cost')
    plt.plot(task_counts, milp_costs, marker='s', color='r', label='Linear relaxation Cost')

    # Set plot labels and title
    plt.xlabel('Number of Tasks')
    plt.ylabel('Cost')

    # Add legend and grid
    plt.legend()
    plt.grid(True)

    # Display the plot
    plt.show()

if __name__ == "__main__":
    main()
