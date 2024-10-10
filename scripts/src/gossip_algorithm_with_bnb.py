"""
Gossip Algorithm
@H. Chakraa
"""
import numpy as np

def gossip_algorithm_with_bnb(scenario, max_iterations):

    # Get a random solution
    current_solution = scenario.generate_random_solution()
    
    # Compute the current cost of the solution
    current_cost = scenario.compute_solution_cost(current_solution)
    
    # Lists to track costs over time
    improved_solution_costs = [current_cost]    # Costs when a better solution is found
    total_iteration_costs = [current_cost]     # Costs after each task exchange
    
    # Get the possible robot pairs that can exchange tasks
    robot_pairs = scenario.select_compatible_robots()
    
    iteration_count  = 1  # Track iterations for stopping criteria
    total_iterations = 1  # Track the total number of iterations
    convergence_iteration = 0  # Iteration at which equilibrium is reached
        
    while iteration_count < max_iterations:
        # Shuffle the robot pairs randomly
        np.random.shuffle(robot_pairs)
        
        for pair in robot_pairs: 
                
            # Apply the gossip mechanism between the selected pair of robots
            new_solution = scenario.task_exchange_mechanism(current_solution, pair)
                
            # Compute the cost for the newly obtained solution
            new_cost = scenario.compute_solution_cost(new_solution)
            total_iteration_costs.append(new_cost)
                
            if new_cost < current_cost:
                
                # If a better solution is found, reset the iteration count
                iteration_count = 1
                convergence_iteration = total_iterations
                
                # Update to the better solution
                current_solution = new_solution
                
                # Apply TSP algorithm for the pair
                current_solution[pair[0]-1] = scenario.branch_and_bound_tsp(current_solution[pair[0]-1])
                current_solution[pair[1]-1] = scenario.branch_and_bound_tsp(current_solution[pair[1]-1])
                
                current_cost = scenario.compute_solution_cost(current_solution)
                
                # Track the improved solution's cost
                improved_solution_costs.append(current_cost)
                
            else:
                
                iteration_count += 1
                
            total_iterations += 1
    
    return current_solution, improved_solution_costs, total_iteration_costs, convergence_iteration
