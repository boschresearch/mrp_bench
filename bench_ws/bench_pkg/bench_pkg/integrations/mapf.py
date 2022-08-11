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

class MAPF:
    @staticmethod
    def callLib(binary, args, timeout=config.pathPlanning.timeout):
        # returns: (success|stdout, timeoutReached)
        print(f'Calling MAPF with timout {timeout}, binary is ' + binary)
        call = [binary]
        call.extend(args)
        try:
            output = subprocess.run(call, capture_output=True, timeout=timeout)
            print(output)
        except subprocess.TimeoutExpired:
            print('Planning failed. Timeout reached.')
            return (False, True)

        # check for success
        if output.returncode != 0:
            print('Planning NOT successful')
            return (False, False)
        else:
            print('Found solution')
            return (output.stdout, False)

    @staticmethod
    def convertCellGridToMAPFFormat(cellGrid, pathToTmpMapFile):
        # format documented at: https://movingai.com/benchmarks/formats.html
        print('Saving grid as MAPF map to ' + pathToTmpMapFile)
        width, height = cellGrid.getDimensionsAsList()
        print('width: ' + str(width))
        print('height: ' + str(height))

        txt = open(pathToTmpMapFile, 'w')

        # put metadata
        txt.write('type octile\n')
        txt.write(f'height {str(height)}\n')
        txt.write(f'width {str(width)}\n')
        txt.write('map\n')

        i = 0
        symbolFree = '.' # for use in lib
        symbolObstructed = '@' # for use in lib

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
        print('Grid saved to ' + pathToTmpMapFile)

    @staticmethod
    def convertStartsAndGoalsToMAPFScenario(cellGrid, startsAndGoals, pathToTmpScenarioFile, pathToMap):
        # format documented at: https://movingai.com/benchmarks/formats.html
        print('Saving starts and goals as MAPF scenario to ' + pathToTmpScenarioFile)
        width, height = cellGrid.getDimensionsAsList()

        txt = open(pathToTmpScenarioFile, 'w')
        txt.write('version 1\n')
        for sag in startsAndGoals:
            # startsAndGoals be like: [([18, 17], [16, 0]), ([58, 31], [3, 31]), ([62, 4], [62, 4])]
            # optimalLength = eucl dist - but do we even need this?
            txt.write(f'0\t{pathToMap}\t{str(width)}\t{str(height)}\t{sag[0][0]}\t{sag[0][1]}\t{sag[1][0]}\t{sag[1][1]}\t1.0\n')


    @staticmethod
    def getSchedulesFromOutfile(pathToFile):
        # schedules: [agent0 sched, agent1 sched, ..., agent_i sched, ..., agent_N sched]
        #  agent_i sched: [(cell x, cell y), (cell x, cell y), ..., (cell x, cell y)]

        # example content onf pathToFile:
        # Agent 0: (16,5)->(17,5)->(18,5)->(19,5)->(20,5)->(20,6)->(20,7)->(20,8)->(20,9)->(20,10)->
        # Agent 1: (29,21)->(28,21)->(27,21)->(27,22)->(27,23)->(26,23)->(26,24)->(25,24)->(24,24)->(24,25)->(23,25)->(22,25)->(22,24)->

        file = open(pathToFile, 'r')
        lines =  file.readlines()
        
        schedules = []

        for line in lines:
            schedule = []
            steps = line.split(': ')[1]
            for step in steps.split('->')[:-1]:
                # order is row, column!
                y = int(step.split(',')[0][1:])
                x = int(step.split(',')[1][:-1])
                schedule.append((x, y))

            schedules.append(schedule)

        return schedules