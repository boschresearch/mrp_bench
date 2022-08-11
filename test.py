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



import timeit
from math import sqrt
import time
import subprocess
import re
from dotmap import DotMap
import yaml
import os
import json


class Util:
    # replace char at index in string. From https://stackoverflow.com/a/41753038/1811821
    @staticmethod
    def stringReplacer(s, newstring, index):
        return s[:index] + newstring + s[index + 1:]

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
            gridTxt[step['y']] = gridTxt[step['y']][:step['x']] + replaceChar + gridTxt[step['y']][step['x'] + 1:]

        f = open(pathToDrawnSolution, 'w')
        f.writelines(gridTxt)
        f.close()
        print('Solution drawn and saved to ' + pathToDrawnSolution)



Util.drawSolutionOnTxt('/tmp/in.yaml', '/tmp/out.yaml', '/tmp/a.txt')


