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

from ament_index_python.packages import get_package_share_directory
import os
from dotmap import DotMap
import yaml
import subprocess


with open(os.path.join(get_package_share_directory('bench_pkg'), 'param/config.yaml'), 'r') as stream:
    try:
        config = DotMap(yaml.safe_load(stream), _dynamic=False)
    except yaml.YAMLError as exc:
        print(exc)

class LibMultiRobotPlanning:
    @staticmethod
    def callLibMultiRobotPlanning(binary, args, timeout=config.pathPlanning.timeout):
        # returns: (success|stdout, timeoutReached)
        print(f'Calling libMultiRobotPlanning with timout {timeout}, binary is ' + binary)
        call = [binary]
        call.extend(args)
        try:
            output = subprocess.run(call, capture_output=True, timeout=timeout)
            print(output)
        except subprocess.TimeoutExpired:
            print('Planning failed. Timeout reached.')
            return (False, True)
        if 'Planning NOT successful' in str(output.stdout):
            print('Planning NOT successful')
            return (False, False)
        else:
            print('Found solution')
            return (output.stdout, False)

    @staticmethod
    def drawSolutionOnTxt(pathToOriginalTxt, pathToSolutionYaml, pathToDrawnSolution):
        print('Drawing solution')
        f = open(pathToOriginalTxt, 'r')
        gridTxt = f.readlines()
        f.close()
        with open(pathToSolutionYaml, 'r') as stream:
            try:
                solution = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
        
        # use equal sized chars
        gridTxt =  [line.replace('.', '□') for line in gridTxt]
        gridTxt =  [line.replace('#', '■') for line in gridTxt]
        for step in solution['schedule']['agent1']:
            replaceChar = '▦'
            # draw start and goal in special char
            if step['t'] == 0:
                replaceChar = '◎'
            if step['t'] == len(solution['schedule']['agent1']) - 1:
                replaceChar = '◉'
            # replace single character
            gridTxt[step['y']] = gridTxt[step['y']][:step['x']] + replaceChar + gridTxt[step['y']][step['x'] + 1:]

        f = open(pathToDrawnSolution, 'w')
        f.writelines(gridTxt)
        f.close()
        print('Solution drawn and saved to ' + pathToDrawnSolution)

    @staticmethod
    def convertCellGridToObstacleMap(cellGrid, startsAndGoals, pathToInputFile, builtinTA):
        o = {
            'map': {
                'dimensions': cellGrid.getDimensionsAsList(),
                'obstacles': []
            },
            'agents': [] 
        }

        for y, line in enumerate(cellGrid.getGrid()):
            for x, value in enumerate(line):
                if value == cellGrid.getOccupiedSymbol():
                    o['map']['obstacles'].append([x, y])
        if builtinTA:
            goals = [sag[1] for sag in startsAndGoals]
            for i, sg in enumerate(startsAndGoals):
                agent = {
                    'name': config.common.agentPrefix + str(i),
                    'start': sg[0],
                    'potentialGoals': goals
                }
                o['agents'].append(agent)
        else:
            for i, sg in enumerate(startsAndGoals):
                agent = {
                    'name': config.common.agentPrefix + str(i),
                    'start': sg[0],
                    'goal': sg[1] 
                }
                o['agents'].append(agent)
            
        # save on disk
        class NoAliasDumper(yaml.SafeDumper):
            def ignore_aliases(self, data):
                return True
        f = open(pathToInputFile, 'w')
        f.write(yaml.dump(o, Dumper=NoAliasDumper, sort_keys=False, default_style=None))
        f.close()
        print('Input map for cellGrid saved to ' + pathToInputFile)


    @staticmethod
    def convertNxGraphToRoadmap(graph, startsAndGoals, pathToRoadmap):
        o = {
            'roadmap': {
                'undirected': True,
                'edges': []
            },
            'agents': [] 
        }

        for e in graph.edges:
            o['roadmap']['edges'].append(list(e))

        for i, sg in enumerate(startsAndGoals):
            agent = {
                'name': 'agent' + str(i),
                'start': sg[0],
                'goal': sg[1] 
            }
            o['agents'].append(agent)
        
        # save on disk
        f = open(pathToRoadmap, 'w')
        f.write(yaml.dump(o, sort_keys=False))
        f.close()
        print('CBS Roadmap saved to ' + pathToRoadmap)


    @staticmethod
    def convertAndStoreGridAsTxt(cellGrid, pathToGridTmpTxtFile):
        print('Saving grid to ' + pathToGridTmpTxtFile)
        width, height = cellGrid.getDimensionsAsList()
        print('width: ' + str(width))
        print('height: ' + str(height))

        txt = open(pathToGridTmpTxtFile, 'w')
        i = 0
        symbolFree = '.' # for use in lib
        symbolObstructed = '#' # for use in lib

        cellGridFree = cellGrid.getFreeSymbol()
        cellGridNode = cellGrid.getNodeSymbol()

        for line in cellGrid.getGrid():
            for e in line:
                if e in [cellGridFree, cellGridNode]:
                    txt.write(symbolFree)
                else:
                    txt.write(symbolObstructed)
            txt.write('\n')

        txt.close()
        print('Grid saved to ' + pathToGridTmpTxtFile)



    @staticmethod
    def getSchedulesFromOutfile(pathToFile):
        # schedules: [agent0 sched, agent1 sched, ..., agent_i sched, ..., agent_N sched]
        #  agent_i sched: [(cell x, cell y), (cell x, cell y), ..., (cell x, cell y)]
        with open(pathToFile, 'r') as stream:
            try:
                solution = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
        
        schedules = []

        for agent in solution['schedule']:
            schedule = []
            for step in solution['schedule'][agent]:
                schedule.append((step['x'], step['y']))

            schedules.append(schedule)

        return schedules
