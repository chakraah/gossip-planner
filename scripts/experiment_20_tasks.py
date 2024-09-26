"""
This program simulates multiple scenarios with 20 tasks and varying numbers of robots.
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
    """Dynamically run the specified scenario module and return average costs and execution time."""
    # Import the scenario module dynamically
    scenario_module = importlib.import_module(module_name)

    # Retrieve task list, robot list, and cost matrix from the module
    task_list = scenario_module.task_list
    robot_list = scenario_module.robot_list
    cost_matrix = scenario_module.cost_matrix
    
    costs = []  # To store costs for multiple runs
    # Execute the scenario multiple times for averaging cost
    for _ in range(10):
        start_time = time.time()  # Start time for execution
        task_planner = TaskPlanner(task_list, robot_list, cost_matrix)
        improved_solution, costs_over_iterations, _ = gossip_algorithm(task_planner, 50)
        
        # Capture the final cost from the costs over iterations
        final_cost = costs_over_iterations[-1]
        costs.append(final_cost)
        
        end_time = time.time()  # End time for execution
    
    # Calculate the average cost from all runs
    average_cost = np.mean(costs)

    # Calculate the total elapsed time for the last run
    total_elapsed_time = end_time - start_time

    return {
        'Scenario': module_name,
        'Average Cost': average_cost,
        'Elapsed Time': total_elapsed_time
    }

def main():
    # List of scenario modules to process
    scenario_modules = [
        'scenarios.data_20_tasks.scenario1',
        'scenarios.data_20_tasks.scenario2',
        'scenarios.data_20_tasks.scenario3',
        'scenarios.data_20_tasks.scenario4',
        'scenarios.data_20_tasks.scenario5',
        'scenarios.data_20_tasks.scenario6',
        'scenarios.data_20_tasks.scenario7',
        'scenarios.data_20_tasks.scenario8',
        'scenarios.data_20_tasks.scenario9',
    ]

    results = []  # To store results from each scenario
    # Process each scenario module
    for module_name in scenario_modules:
        print(f"Processing {module_name}...")
        result = execute_scenario(module_name)
        results.append(result)

    # Create a DataFrame to display the results
    results_df = pd.DataFrame(results)
    print("\nResults Table:")
    print(results_df)
    
    # Data for plotting costs against the number of robots
    num_robots = [4, 6, 8, 10, 12, 14, 16, 18, 20]
    gossip_costs = [result['Average Cost'] for result in results]
    milp_costs = [27.567, 21.733, 18.817, 17.067, 15.09, 15.067, 14.442, 13.956, 13.567]
    
    # Create a plot to compare costs
    plt.figure(figsize=(10, 6))
    plt.plot(num_robots, gossip_costs, marker='o', color='b', label='Gossip Algorithm Cost')
    plt.plot(num_robots, milp_costs, marker='s', color='r', label='Linear relaxation Cost')

    # Set plot labels and title
    plt.xlabel('Number of Robots')
    plt.ylabel('Cost')

    # Add legend and grid for better readability
    plt.legend()
    plt.grid(True)

    # Display the plot
    plt.show()

if __name__ == "__main__":
    main()