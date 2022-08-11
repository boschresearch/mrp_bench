# Copyright (c) 2022 - for information on the respective copyright owner
# see the NOTICE file or the repository https://github.com/boschresearch/mrp-bench.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import ortools
from ortools.linear_solver import pywraplp
from ortools.sat.python import cp_model
import numpy as np
from ament_index_python.packages import get_package_share_directory
import yaml
from dotmap import DotMap
import os
import timeit

# load configuration
print(os.getcwd() )
with open(os.path.join(get_package_share_directory('bench_pkg'), 'param/config.yaml'), 'r') as stream:
    try:
        config = DotMap(yaml.safe_load(stream), _dynamic=False)
    except yaml.YAMLError as exc:
        print(exc)


class TaskAllocation:
    def __init__(self, benchManager) -> None:
        self.bm = benchManager

    # starts = [(5, 10), (25,54), (25, 19), (19, 59)]
    # goals = [(1, 69), (13,11), (25, 54), (45, 11)]

    def calculate_costs_euclidean(self, starts, goals):
        return [[np.linalg.norm(np.array(start)-np.array(goal)) for goal in goals] for start in starts]

    def calculate_costs_manhattan(self, starts, goals):
        return [[abs(start[0]-goal[0]) + abs(start[1]-goal[1]) for goal in goals] for start in starts]

    def performAllocation(self, starts, goals):
        cost_func = getattr(self, 'calculate_costs_' + config.taskAllocation.costFunction)
        
        # in the case of manhattan or euclidean, this is trivial and fast.
        # However, A* may be used later and then this becomes a relevant time factor.
        def performCall(starts, goals):
            global res
            res = cost_func(starts, goals)

        iterations, totalTime = timeit.Timer(lambda: performCall(starts, goals)).autorange() # autorange to get valid results for small calc time
        costs = globals()['res']

        timeToCalculateDistanceMatrix = totalTime/iterations
        self.bm.metrics.updateMetric('timeToCalculateDistanceMatrix', timeToCalculateDistanceMatrix)

        # the actual TA
        # TODO: Handle case when no solution 
        solve_func = getattr(self, 'solve_' + config.taskAllocation.allocationAlgorithm)

        def performCall(costs, starts, goals):
            global res
            res = solve_func(costs, starts, goals)

        iterations, totalTime = timeit.Timer(lambda: performCall(costs, starts, goals)).autorange() 
        allocation = globals()['res']

        self.bm.metrics.updateMetric('timeToCalculateAssignment', totalTime/iterations)
        self.bm.metrics.updateMetric('totalTimeForTA', timeToCalculateDistanceMatrix + totalTime/iterations)


        return allocation

    # Google OR-Tools - Linear Solver
    def solve_or_linear(self, costs, starts, goals):
        num_agents = len(costs)
        num_tasks = len(costs[0])

        solver = pywraplp.Solver.CreateSolver('SCIP')

        # x[i, j] is an array of 0-1 variables, which will be 1
        # if worker i is assigned to task j.
        x = {}
        for i in range(num_agents):
            for j in range(num_tasks):
                x[i, j] = solver.IntVar(0, 1, '')

        # Each worker is assigned to at most 1 task.
        for i in range(num_agents):
            solver.Add(solver.Sum([x[i, j] for j in range(num_tasks)]) <= 1)

        # Each task is assigned to exactly one worker.
        for j in range(num_tasks):
            solver.Add(solver.Sum([x[i, j] for i in range(num_agents)]) == 1)


        objective_terms = []
        for i in range(num_agents):
            for j in range(num_tasks):
                objective_terms.append(costs[i][j] * x[i, j])
        solver.Minimize(solver.Sum(objective_terms))

        status = solver.Solve()

        startsAndGoals = [] # self.startsAndGoals.append(([start.cell_x, start.cell_y], [sample_goals[i].cell_x, sample_goals[i].cell_y]))
        if status == pywraplp.Solver.OPTIMAL or status == pywraplp.Solver.FEASIBLE:
            print(f'status:', status)
            print(f'Total cost = {solver.Objective().Value()}\n')
            for i in range(num_agents):
                for j in range(num_tasks):
                    # Test if x[i,j] is 1 (with tolerance for floating point arithmetic).
                    if x[i, j].solution_value() > 0.5:
                        print(f'Worker {i} assigned to task {j}.' +
                            f' Cost: {costs[i][j]}')

                        startsAndGoals.append(((starts[i][0], starts[i][1]), (goals[j][0], goals[j][1])))
            return startsAndGoals
        else:
            print('No solution found.')
            return None


    def solve_or_sat(self, costs, starts, goals):
        num_agents = len(costs)
        num_tasks = len(costs[0])

        model = cp_model.CpModel()
        x = []
        for i in range(num_agents):
            t = []
            for j in range(num_tasks):
                t.append(model.NewBoolVar(f'x[{i},{j}]'))
            x.append(t)

        # Each worker is assigned to at most one task.
        for i in range(num_agents):
            model.AddAtMostOne(x[i][j] for j in range(num_tasks))

        # Each task is assigned to exactly one worker.
        for j in range(num_tasks):
            model.AddExactlyOne(x[i][j] for i in range(num_agents))
        objective_terms = []

        # objective function
        for i in range(num_agents):
            for j in range(num_tasks):
                objective_terms.append(costs[i][j] * x[i][j])
        model.Minimize(sum(objective_terms))

        solver = cp_model.CpSolver()
        status = solver.Solve(model)


        if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
            print(f'Total cost = {solver.ObjectiveValue()}')
            print()
            startsAndGoals = [] 
            for i in range(num_agents):
                for j in range(num_tasks):
                    if solver.BooleanValue(x[i][j]):
                        print(
                            f'Worker {i} assigned to task {j} Cost = {costs[i][j]}')
                        startsAndGoals.append(((starts[i][0], starts[i][1]), (goals[j][0], goals[j][1])))
            return startsAndGoals
        else:
            print('No solution found.')
            return None