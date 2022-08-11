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


from dataclasses import dataclass # basically a struct
import yaml
from ament_index_python.packages import get_package_share_directory
import os
from dotmap import DotMap
import math
import numpy as np
import random
from std_msgs.msg import String, Bool
 

with open(os.path.join(get_package_share_directory('bench_pkg'), 'param/config.yaml'), 'r') as stream:
    try:
        config = DotMap(yaml.safe_load(stream), _dynamic=False)
    except yaml.YAMLError as exc:
        print(exc)

class CellGrid:
    # Assumption: graph is planar, no lines cross when represented in x-y system
    # stores necessary information to convert from grid to graph to real-world coordinates and vice versa
    # starting from (0, 0), cells of size cellSize spread into all direction (also negative if necessary)
    # each point in the x-y space belongs exactly into one cell. Points exactly at the corner of cells belong to the highest possible cell number.
    # each cell covers an area in x-y space. A cell is defined by its lower-left corner
    # example: given cellSize 0.5, the cell (2,3) is defined as the square with the corners (1, 1.5), (1, 2), (1.5, 2), (1.5, 1.5)

    def __init__(self, graph, cellSize, bm) -> None:
        self.graph = graph
        self.cellSize = cellSize
        self.nodes = list()
        self.bm = bm
        self.convertNxGraphToCellGrid(debug=True)

    # provide the center of a cell in world coordinates
    def cellToWorldCos(self, cell_x, cell_y):
        return (cell_x * self.cellSize + config.common.cellSize / 2 + self.minX, cell_y * self.cellSize + config.common.cellSize / 2 + self.minY)

    # given a point in world coordinate system, return the cell that it is in
    def worldCosToCell(self, world_x, world_y):
        return (math.floor((world_x - self.minX) / self.cellSize), math.floor((world_y - self.minY) / self.cellSize))
    
    def getNodeByName(self, nodeName):
        for node in self.nodes:
            if node.name == nodeName:
                return node
        return None

    def getNodeNameByGrid(self, x, y):
        for node in self.nodes:
            if node.cell_x == x and node.cell_y == y:
                return node.name
        return None

    def convertNxGraphToCellGrid(self, debug=False):
        # TODO: perform planarity test, as this is an assumed attribute of the graph

        # choose something else for debugging maybe
        self.freeSymbol = 0
        self.occupiedSymbol = 1
        self.nodeSymbol = '#' # use freeSymbol or different symbol here to visualize nodes

        # find max dimensions of graph, create matrix
        #   all fields default to occupied (1)
        self.minX = list(self.graph.nodes(data=True))[0][1]['x']
        self.minY = list(self.graph.nodes(data=True))[0][1]['y']
        self.maxX = list(self.graph.nodes(data=True))[0][1]['x']
        self.maxY = list(self.graph.nodes(data=True))[0][1]['y']

        for node in list(self.graph.nodes(data=True))[1:]:
            self.minX = min(self.minX, node[1]['x']);
            self.minY = min(self.minY, node[1]['y']);
            self.maxX = max(self.maxX, node[1]['x']);
            self.maxY = max(self.maxY, node[1]['y']);

        cells_x = math.ceil(abs(self.minX - self.maxX) / self.cellSize)
        cells_y = math.ceil(abs(self.minY - self.maxY) / self.cellSize)

        self.cellGrid = [[self.occupiedSymbol] * (cells_x + 1) for y in range(cells_y + 1)]
        
        for node in list(self.graph.nodes(data=True)):
            cell = self.worldCosToCell(node[1]['x'], node[1]['y'])
            self.nodes.append(CellGridNode(
                id = node[0],
                world_x = node[1]['x'],
                world_y = node[1]['y'],
                cell_x = cell[0],
                cell_y = cell[1],
                name = node[1]['name']
                ))

        # if debug:
        #     self.printCellGrid()
        #     print('dim_x: ', len(self.cellGrid[0]))
        #     print('dim_y: ', len(self.cellGrid))
            # for node in list(self.graph.nodes(data=True)):
                # print(node)
                # cell = self.worldCosToCell(node[1]['x'], node[1]['y'])
                # print(cell)
                # print(self.cellToWorldCos(cell[0], cell[1]))
                # print('---------------')
                
        # for each edge in graph:    
        # perform custom distance metric minimization
        # set visited fields to 0 in matrix
        for n1, n2 in self.graph.edges():
            cell1 = self.worldCosToCell(self.graph.nodes[n1]['x'], self.graph.nodes[n1]['y'])
            c1 = np.array(cell1)
            cell2 = self.worldCosToCell(self.graph.nodes[n2]['x'], self.graph.nodes[n2]['y'])
            c2 = np.array(cell2)

            self.cellGrid[cell1[1]][cell1[0]] = self.nodeSymbol
            self.cellGrid[cell2[1]][cell2[0]] = self.nodeSymbol

            # if debug:
            #     print(cell1, cell2)

            currentCell = cell1

            while not currentCell == cell2:
                # print('----------------')
                cc = np.array(currentCell)
                # print("currentCell: ", currentCell)
                # print("cell2: ", cell2)
                # next visited cell (out of the four options) must fulfill two requirements:
                # 1) Euclidean distance to goal gets smaller (only one or two cells possible)
                # 2) Distance to line connecting start and goal is minimal
                neighbors = [(currentCell[0], currentCell[1] + 1), (currentCell[0] + 1, currentCell[1]), (currentCell[0], currentCell[1] - 1), (currentCell[0] - 1, currentCell[1])]
                # print('neighbors: ', neighbors)
                valid_neighbors = [n for n in neighbors if not np.linalg.norm(c2 - np.array(n)) > np.linalg.norm(c2 - cc)]
                # print('valid_neighbors: ', valid_neighbors)
                # only 1 option? then color and move on
                if len(valid_neighbors) == 1:
                    currentCell = valid_neighbors[0]
                else: # two options: get norm https://stackoverflow.com/questions/39840030/distance-between-point-and-a-line-from-two-points
                    d1 = np.linalg.norm(np.cross(c2-c1, c1-np.array(valid_neighbors[0])))/np.linalg.norm(c2-c1)
                    d2 = np.linalg.norm(np.cross(c2-c1, c1-np.array(valid_neighbors[1])))/np.linalg.norm(c2-c1)
                    # print("d1, d2: ", d1, d2)
                    if d1 < d2:
                        currentCell = valid_neighbors[0]
                    else:
                        currentCell = valid_neighbors[1]
                
                if self.cellGrid[currentCell[1]][currentCell[0]] != self.nodeSymbol:
                    self.cellGrid[currentCell[1]][currentCell[0]] = self.freeSymbol

        if debug:    
            self.printCellGrid()

        # post processing step: make "edges" in grid bidirectional
        def getNumFreeNeighbors(x, y):
            # return the number of free cells in the 4-neighborhood of the given cell
            return sum([self.cellGrid[j][i] != self.occupiedSymbol for i in [max(0, x-1), min(x+1, len(self.cellGrid[0])-1)] for j in [max(0, y-1), min(y+1, len(self.cellGrid)-1)]])

        if config.postProcessing.dilateGrid:
            for y in range(len(self.cellGrid)):
                for x in range(len(self.cellGrid[0])):
                    if self.cellGrid[y][x] == self.freeSymbol:
                        # check the 4-neighbourhood. If not at least 3 are passable, add free cells left and below
                        # the idea is that left and below have already been checked, so they won't lead to a double expansion
                        if getNumFreeNeighbors(x, y) < 3:
                            # only set if occupied to not change node cells
                            if x != 0 and self.cellGrid[y][x-1] == self.occupiedSymbol:
                                self.cellGrid[y][x-1] = self.freeSymbol
                            if y != 0 and self.cellGrid[y-1][x] == self.occupiedSymbol:
                               self.cellGrid[y-1][x] = self.freeSymbol

            # handle first row and column specifically in an extra step
            # first row:
            for x in range(len(self.cellGrid[0])):
                if self.cellGrid[0][x] == self.freeSymbol:
                    if getNumFreeNeighbors(x, y) < 3:
                        # only set if occupied to not change node cell
                        # however, now we go above instead of below
                        if self.cellGrid[1][x] == self.occupiedSymbol:
                            self.cellGrid[1][x] = self.freeSymbol
            # first column:
            for y in range(len(self.cellGrid)):
                if self.cellGrid[y][0] == self.freeSymbol:
                    if getNumFreeNeighbors(x, y) < 3:
                        # only set if occupied to not change node cell
                        # however, now we go right instead of left
                        if self.cellGrid[y][1] == self.occupiedSymbol:
                            self.cellGrid[y][1] = self.freeSymbol
        
        if debug:    
            self.printCellGrid()

    def getFreeSymbol(self):
        return self.freeSymbol

    def getOccupiedSymbol(self):
        return self.occupiedSymbol

    def getNodeSymbol(self):
        return self.nodeSymbol

    def getGridStatics(self):
        gridStats = {
            'numTotal': 0,
            'numFree': 0,
            'numOccupied': 0,
        }
        for y in range(len(self.cellGrid)):
            for x in range(len(self.cellGrid[0])):
                if self.cellGrid[y][x] == self.freeSymbol or self.cellGrid[y][x] == self.nodeSymbol:
                    gridStats['numFree'] += 1
                else:
                    gridStats['numOccupied'] += 1
        
        gridStats['numTotal'] = gridStats['numFree'] + gridStats['numOccupied']

        return gridStats

    # goals are random, allocation is not necessarily
    def generateRandomStartsAndGoals(self, randomSeed=config.common.random.randomSeed, numAgents=config.common.random.numAgents):
        # generate the random starts and goals
        if numAgents > len(self.nodes):
            raise AssertionError(f'numAgents ({numAgents}) is greater than the number of available nodes ({len(self.nodes)})')
        
        self.startsAndGoals = []
        random.seed(randomSeed)

        sample_start = random.sample(self.nodes, numAgents)
        random.seed(randomSeed+1)
        sample_goals = random.sample(self.nodes, numAgents)

        # random allocation or built-in TA 
        # built-in TA:
            # we will still use this data structure to pass through starts and goals for use with built-in TA,
            # but he assignment listed here is not used. Just a hack.
        for i, start in enumerate(sample_start):
            self.startsAndGoals.append(([start.cell_x, start.cell_y], [sample_goals[i].cell_x, sample_goals[i].cell_y]))
        if not config.taskAllocation.randomAllocation and not config.algorithms[config.pathPlanning.algorithmToCall].builtinTA:
            # separate TA assignment
            starts = [(start.world_x, start.world_y) for start in sample_start]
            goals = [(goal.world_x, goal.world_y) for goal in sample_goals]

            alloc = self.bm.taskAllocator.performAllocation(starts, goals)
            self.startsAndGoals = []
            for sg in alloc:
                s = list(self.worldCosToCell(sg[0][0], sg[0][1]))
                g = list(self.worldCosToCell(sg[1][0], sg[1][1]))
                self.startsAndGoals.append((s, g))
        # if randomAllocation was true, the random assignment from the for-loop stays.
        # if builtinTA was true, the library being called will take the random assignment to generate a new one. 

    # list of starts and goals is fixed, assignment is not necessarily also fixed:
    def generateStartsAndGoalsFromConfig(self):
        self.startsAndGoals = []

        if config.taskAllocation.fixedAllocation:
            for i, task in enumerate(config.maps[config.common.mapName].tasks):
                startNode = self.getNodeByName(task[0])
                goalNode = self.getNodeByName(task[1])

                if startNode is None:
                    print('ERROR: Node ' + task[0] + ' does not exist!')
                    exit()

                if goalNode is None:
                    print('ERROR: Node ' + task[1] + ' does not exist!')
                    exit()

                self.startsAndGoals.append(([startNode.cell_x, startNode.cell_y], [goalNode.cell_x, goalNode.cell_y]))
        else:
            # prep data
            starts = []
            goals = []
            for task in config.maps[config.common.mapName].tasks:
                starts.append(self.getNodeByName(task[0]))
                goals.append(self.getNodeByName(task[1]))

            if config.taskAllocation.randomAllocation or config.algorithms[config.pathPlanning.algorithmToCall].builtinTA:
                # the given list is used, but starts and goals are mixed.
                # built-in TA:
                    # we will still use this data structure to pass through starts and goals for use with built-in TA,
                    # but he assignment listed here is not used. Just a hack.
                # mix randomly
                random.Random(config.common.random.randomSeed).shuffle(starts)
                random.Random(config.common.random.randomSeed+1).shuffle(goals)
                for i, start in enumerate(starts):
                    self.startsAndGoals.append(([start.cell_x, start.cell_y], [goals[i].cell_x, goals[i].cell_y]))
            else:
                # separate TA assignment
                alloc = self.bm.taskAllocator.performAllocation(starts, goals)
                self.startsAndGoals = []
                for sg in alloc:
                    s = list(self.worldCosToCell(sg[0][0], sg[0][1]))
                    g = list(self.worldCosToCell(sg[1][0], sg[1][1]))
                    self.startsAndGoals.append((s, g))


    def storeStartsAndGoalsMetricsAndRobotsList(self):
        # store agent data for use when spawning robots in simulation
        # store assigned starts and goals in metrics file
        agents = []
        # sgDict = {
        #     'assignedAgent': 'agent0',
        #     'start_name': 'cleaning_cabinet',
        #     'goal_name': 'tiger',
        #     'start_grid': [5, 57],
        #     'goal_grid': [87, 15],
        #     'start_wcos': [8.512312, -17.35213],
        #     'goal_wcos': [12.6512, -4.1823],
        # }

        # if TA is part of algorithm, this will be called twice.
        # reset list in metrics for that case
        self.bm.metrics.updateMetric('startsAndGoals', [])


        for i, sg in enumerate(self.startsAndGoals):
            start_wcos_x, start_wcos_y = self.cellToWorldCos(sg[0][0], sg[0][1])
            goal_wcos_x, goal_wcos_y = self.cellToWorldCos(sg[1][0], sg[1][1])
            start_name = self.getNodeNameByGrid(sg[0][0], sg[0][1])
            goal_name = self.getNodeNameByGrid(sg[1][0], sg[1][1])

            agents.append({
                'name': config.common.agentPrefix + str(i),
                'x_pose': start_wcos_x,
                'y_pose': start_wcos_y,
                'z_pose': config.common.agentZPose,
            })

            # store metrics
            sgDict = {
                'assignedAgent': config.common.agentPrefix + str(i),
                'start_name': start_name,
                'goal_name': goal_name,
                'start_grid': sg[0],
                'goal_grid': sg[1],
                'start_wcos': [start_wcos_x, start_wcos_y],
                'goal_wcos': [goal_wcos_x, goal_wcos_y],
            }

            self.bm.metrics.addStartAndGoal(sgDict)

        
        # save agents on disk
        f = open(config.common.pathToRobotsList, 'w')
        f.write(yaml.dump(agents, sort_keys=False, default_style=None))
        f.close()


    def getDimensionsAsList(self):
        # [x, y]
        return [len(self.cellGrid[0]), len(self.cellGrid)]

    def getGrid(self):
        return self.cellGrid

    def getOccupiedSymbol(self):
        return self.occupiedSymbol

    def saveToFileForLibMRP(self, filePath):
        f = open(filePath, 'w')
        for line in reversed(self.cellGrid):
            for e in line:
                if e == self.freeSymbol or e == self.nodeSymbol:
                    f.write('.')
                if e == self.occupiedSymbol:
                    f.write('#')
        f.close()
        print('CellGrid saved for use with LibMRP to ' + filePath)

    def printCellGrid(self):
        print('------------')
        for line in reversed(self.cellGrid):
            for e in line:
                print(e, end='')
            print('')
        print('------------')



@dataclass
class CellGridNode:
    id: int
    world_x: float
    world_y: float
    cell_x: int
    cell_y: int
    name: String
