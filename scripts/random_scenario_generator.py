"""
generating a random scenario
@H. Chakraa
"""

import numpy as np

def random_scenario_generator(num_robots, num_tasks, num_sites, num_measurments):
    
    # Initialise task_list with zeros
    task_list = np.zeros((num_measurments, num_sites), dtype=int)
    
    # Randomly place the required number of tasks (1s) in the task_list
    while np.sum(task_list) < num_tasks:
        # Choose a random location to place a task
        row = np.random.randint(num_measurments)
        col = np.random.randint(1, num_sites)  # Ensure it does not place in the first column
        
        # Place a task if the spot is empty
        if task_list[row, col] == 0:
            task_list[row, col] = 1
    
    # Generate robot list ensuring each sensor have at least one robot equipped with it
    robot_list = np.random.randint(low=0, high=2, size=(num_measurments, num_robots), dtype=int)
    
    while 0 in robot_list.sum(axis=1):
        robot_list = np.random.randint(low=0, high=2, size=(num_measurments, num_robots), dtype=int)
    
        
    # Generate symmetric cost matrix
    upper_triangular = np.random.randint(1, 100, size=(num_sites, num_sites))
    upper_triangular = np.triu(upper_triangular)
    cost_matrix = upper_triangular + upper_triangular.T
    np.fill_diagonal(cost_matrix, 0)  # Diagonal elements should be zero
    
    # Floyd-Warshall Algorihm
    for k in range(num_sites):
        for i in range(num_sites):
            for j in range(num_sites):
                if cost_matrix[i][j] > cost_matrix[i][k] + cost_matrix[k][j]:
                    # Update the matrix to satisfy the triangular inequality
                    cost_matrix[i][j] = cost_matrix[i][k] + cost_matrix[k][j]

    return task_list, robot_list, cost_matrix
