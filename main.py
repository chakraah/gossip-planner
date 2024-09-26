"""
main function
@H. Chakraa
"""

import time
import matplotlib.pyplot as plt
from random_scenario_generator import random_scenario_generator
from scripts.src.task_planner import TaskPlanner
from scripts.src.gossip_algorithm import gossip_algorithm

def main():
    
    # Generating an example with 4 robots, and 40 tasks
    task_requirements, robot_capabilities, cost_matrix = random_scenario_generator(num_robots=4, num_tasks=30)
    
    # Create a Scenario instance
    scenario = TaskPlanner(task_requirements, robot_capabilities, cost_matrix)
    
    # Print number of robots, sites, and tasks
    print(f"Number of sites: {scenario.num_sites}")
    print(f"Number of measurments: {scenario.num_measurements}")
    print(f"Number of robots: {scenario.num_robots}")
    print(f"Number of tasks: {scenario.num_tasks}\n")
    
    # Define a limit for iterations
    max_iterations = 10
    
    # Run the gossip algorithm
    start_time = time.time()
    task_distribution, total_costs, stopping_criteria = gossip_algorithm(scenario, max_iterations)
    end_time = time.time()

    # Plot the cost over iterations
    plt.figure(figsize=(12, 8))
    
    # Plot cost after improvement
    plt.plot(total_costs, marker='o', linestyle='-', color='b', 
         label='Cost after improvement', linewidth=2.5, markersize=8)

    # Plot cost after tasks exchange
    plt.plot(stopping_criteria, marker='s', linestyle='--', color='r', 
         label='Cost after tasks exchange', linewidth=2.5, markersize=8)

    # Labels and title
    plt.xlabel('Iterations', fontsize=16)
    plt.ylabel('Cost', fontsize=16)

    # Grid styling
    plt.grid(True)

    # Legend
    plt.legend(fontsize=14)

    # Show plot
    plt.show()
    
    # Output the results
    print(f"Cost: {total_costs[-1]}")
    print(f"Execution Time: {end_time - start_time:.4f} seconds")
    
if __name__ == "__main__":
    main()