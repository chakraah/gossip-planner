"""
Task planning optimisation
@H. Chakraa
"""

import numpy as np
import random
import itertools

class TaskPlanner:
    
    def __init__(self, task_list, robot_list, cost_matrix):
        self.task_list = np.array(task_list)
        self.robot_list = np.array(robot_list)
        self.cost_matrix = np.array(cost_matrix)

        # Calculate the number of sites, measurements, robots, and tasks
        self.num_sites = self.task_list.shape[1]
        self.num_measurements = self.robot_list.shape[0]
        self.num_robots = self.robot_list.shape[1]
        self.num_tasks = np.sum(self.task_list)
        
    ## | ---------------- Function: generating a random solution --------------- |

    def generate_random_solution(self):
        
        # Initialize the random sequence for each robot
        random_sequence = [[] for _ in range(self.num_robots)]
        
        # Assign tasks to robots for each site (excluding the depot, which is site 1)
        for site_idx in range(1, self.num_sites):
            # Get the actual site
            site_data = self.task_list[:, site_idx]
            
            # Find the required measurments in this site
            measurement_numbers = np.where(site_data == 1)[0]
            
            # Iterate over each measurement
            for measurement in measurement_numbers:
                # Get the row corresponding to the current measurement from robot_list 
                robots_measurement = self.robot_list[measurement, :]
                
                # Find robots that can perform the measurement
                suitable_robots = np.where(robots_measurement == 1)[0]
                
                # Randomly assign one of the suitable robots
                if suitable_robots.size > 0:
                    allocated_robot = random.choice(suitable_robots)
                    task = {'site': site_idx+1, 'measure': measurement+1}
                    
                    # Add the task to the selected robot's task sequence
                    random_sequence[allocated_robot].append(task)
        
        # Generate a random order for the tasks assigned to each robot
        for robot_idx in range(self.num_robots):
            random.shuffle(random_sequence[robot_idx])
        
        depot = {'site': 1}
        
        # Ensure each robot's task sequence starts and ends at the depot
        for robot_idx in range(self.num_robots):
            random_sequence[robot_idx] = [depot] + random_sequence[robot_idx] + [depot]
        return random_sequence
    
    ## | ---------------- Function: selection of two robots --------------- |
    
    def select_compatible_robots(self):
        compatible_pairs = []  # Initialize the list to store selected robot pairs
        for i in range(self.num_robots-1):
            for j in range(i + 1, self.num_robots):
                compatibility = (self.robot_list[:,i] == 1) & (self.robot_list[:,j] == 1)
                
                if np.any(compatibility):
                    compatible_pairs.append([i+1, j+1])

        return np.array(compatible_pairs)
    
    ## | ---------------- Function: cost calculation of a solution --------------- |
    
    def compute_solution_cost(self, sequences):
        robot_costs = np.zeros(self.num_robots)
        
        for robot_idx in range(self.num_robots):
            
            if sequences[robot_idx]: # Check if the robot has any tasks assigned
                robot_sequence = np.array([entry['site'] for entry in sequences[robot_idx]])
                from_sites = robot_sequence[:-1]-1
                to_sites = robot_sequence[1:]-1
                
                travel_costs = self.cost_matrix[from_sites, to_sites]
                
                robot_costs[robot_idx] = np.sum(travel_costs)
                
        # The total cost is the maximum cost incurred by any robot (makespan)
        return np.max(robot_costs)
    
    def compute_robot_cost(self, sequence):
                    
        if sequence: # Check if the robot has any tasks assigned
            robot_sequence = np.array([entry['site'] for entry in sequence])
            from_sites = robot_sequence[:-1]-1
            to_sites = robot_sequence[1:]-1
                
            travel_costs = self.cost_matrix[from_sites, to_sites]
                                            
        return np.sum(travel_costs)
    
    ## | ---------------- Function: 2-Opt local search --------------- |
    
    def two_opt(self, sequence):
     best = sequence
     improved = True
     while improved:
          improved = False
          for i in range(1, len(sequence)-2):
               for j in range(i+1, len(sequence)):
                    if j-i == 1: continue # changes nothing, skip then
                    new_route = sequence[:]
                    new_route[i:j] = sequence[j-1:i-1:-1] # this is the 2woptSwap
                    if self.compute_robot_cost(new_route) < self.compute_robot_cost(best):
                         best = new_route
                         improved = True
          sequence = best
     return best
 
    ## | ---------------- Function: Brute force algorithm --------------- |
    
    def brute_force(self, sequence):
        
        # Extract only the entries that have both 'site' and 'measure'
        elements = [entry for entry in sequence[1:] if 'measure' in entry]

        # Generate permutations of the list of dictionaries
        permutations = list(itertools.permutations(elements))

        # Wrap the beginning and end with {'site': 1}
        site_1 = {'site': 1}

        # Create new permutations with 'site': 1 at the beginning and end
        final_permutations = [[site_1] + list(perm) + [site_1] for perm in permutations]
        
        min_cost = float('inf')
        
        for perm in final_permutations:
            cost = self.cost_computation_one_robot(perm)
            if cost < min_cost:
                min_cost = cost
                sequence = perm
                
        return sequence
        
     
    ## | ---------------- Function: Task exchange mechanism --------------- |
    
    def task_exchange_mechanism(self, sequence, selected_robots):
        
        # If no robots were selected, return the original task sequences
        if selected_robots.size == 0:
            return sequence
        
        def exchange_tasks_between_robots(source_robot_idx, target_robot_idx):
            
            # Find tasks that both robots can perform
            compatibility  = (self.robot_list[:, selected_robots[source_robot_idx] - 1] == 1) & (self.robot_list[:, selected_robots[target_robot_idx] - 1] == 1)
            common_capabilities = np.where(compatibility)
            common_capabilities = tuple(arr + 1 for arr in common_capabilities)
            common_capabilities_set = set(common_capabilities [0])
            
            # Identify common tasks in source_robot sequence
            exchangeable_tasks = [item for item in sequence[selected_robots[source_robot_idx] - 1] if item.get('measure') in common_capabilities_set]

            while exchangeable_tasks:
                # Select a random task from exchangeable_tasks
                random_task = random.choice(exchangeable_tasks)
            
                # Remove the selected task from exchangeable_tasks
                exchangeable_tasks.remove(random_task)
            
                # Insert the task into the target robot's task sequence
                target_robot_sequence = sequence[selected_robots[target_robot_idx] - 1].copy()
                target_robot_sequence.insert(-1, random_task)
            
                # Optimise the target robot's new sequence using the 2-Opt algorithm
                target_robot_sequence = self.two_opt(target_robot_sequence)
                
                # Compare the cost between the new sequences
                if self.compute_robot_cost(target_robot_sequence) < self.compute_robot_cost(sequence[selected_robots[source_robot_idx] - 1]):
                    sequence[selected_robots[target_robot_idx] - 1] = target_robot_sequence 
                    sequence[selected_robots[source_robot_idx] - 1].remove(random_task)
                        
        # Compare the costs for the two selected robots
        cost_robot_1 = self.compute_robot_cost(sequence[selected_robots[0] - 1])
        cost_robot_2 = self.compute_robot_cost(sequence[selected_robots[1] - 1])
    
        if cost_robot_1 > cost_robot_2:
            exchange_tasks_between_robots(0, 1)  # Higher cost: Robot 1, Lower cost: Robot 2
        else:
            exchange_tasks_between_robots(1, 0)  # Higher cost: Robot 2, Lower cost: Robot 1
    
        return sequence