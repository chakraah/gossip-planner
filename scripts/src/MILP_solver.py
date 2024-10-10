"""
Linear relaxation lower bound with pyomo
@H. Chakraa
"""

import pyomo.environ as pyEnv

def MILP_solver(scenario):
    
    #Model
    model = pyEnv.ConcreteModel()

    #Indexes for the cities
    model.M = pyEnv.RangeSet(scenario.num_measurements)                
    model.S = pyEnv.RangeSet(scenario.num_sites)
    model.A = pyEnv.RangeSet(scenario.num_sites)
    model.R = pyEnv.RangeSet(scenario.num_robots)
    

    #Decision variable xijk
    model.x=pyEnv.Var(model.R,model.S,model.S, within=pyEnv.Reals, bounds=(0, 1))

    #Decision variable ui
    model.u=pyEnv.Var(model.R,model.S, within=pyEnv.NonNegativeReals)

    #Cost Matrix cij
    model.c = pyEnv.Param(model.S, model.S,initialize=lambda model, i, j: scenario.cost_matrix[i-1][j-1])

    #robot_list
    model.robot_list = pyEnv.Param(model.M, model.R,initialize=lambda model, i, j: scenario.robot_list[i-1][j-1])

    #task_list
    model.task_list = pyEnv.Param(model.M, model.S,initialize=lambda model, i, j: scenario.task_list[i-1][j-1])
    
    # Define auxiliary variable 'z' (representing the maximum)
    model.z = pyEnv.Var(domain=pyEnv.NonNegativeReals)

    # Define the new objective: minimize 'z'
    def obj_func_minmax(model):
        return model.z

    model.objective = pyEnv.Objective(rule=obj_func_minmax, sense=pyEnv.minimize)

    # Add constraints to ensure 'z' is at least as large as every summation term
    def max_constraint_rule(model, k):
        return model.z >= sum(model.x[k,i,j] * model.c[i,j] for i in model.S for j in model.S)

    model.max_constraint = pyEnv.Constraint(model.R, rule=max_constraint_rule)

    def add_const(model, S, R):
        return model.x[R,S,S] == 0

    model.add_const = pyEnv.Constraint(model.S, model.R, rule=add_const)

    def rule_const1(model, R):
        return sum(model.x[R,1,i] for i in model.S if i!=1 ) == 1

    model.const1 = pyEnv.Constraint(model.R,rule=rule_const1)

    def rule_const2(model,S ,R):
        return sum(model.x[R, S, j] for j in model.S if j != S) == sum(model.x[R, j, S] for j in model.S if j != S)

    model.rest2 = pyEnv.Constraint(model.S,model.R,rule=rule_const2)

    def rule_const3(model,R,S,A):
        if S!=A: 
            return model.u[R,S] + model.x[R,S,A] <= model.u[R,A] + (scenario.num_sites - 1) * (1 - model.x[R,S,A])
        else:
           #Yeah, this else doesn't say anything
           return model.u[R,S] - model.u[R,S] == 0 
        
    model.rest3 = pyEnv.Constraint(model.R,model.S,model.A,rule=rule_const3)

    def rule_const4(model,S,M):
        return sum(model.robot_list[M,k] * model.x[k,i,S] for i in model.S for k in model.R) >= model.task_list[M,S]

    model.rest4 = pyEnv.Constraint(model.S,model.M,rule=rule_const4)

    #Solves
    solver = pyEnv.SolverFactory('cplex')
    result = solver.solve(model, tee=False)
    
    if result.solver.termination_condition == pyEnv.TerminationCondition.optimal:
        return pyEnv.value(model.objective)
    else:
        print("No optimal solution found.")
