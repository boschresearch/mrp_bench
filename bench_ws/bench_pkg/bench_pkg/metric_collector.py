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


import datetime
import os
from tracemalloc import start
from turtle import update
from ament_index_python.packages import get_package_share_directory
import yaml
from dotmap import DotMap
import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import math
import numpy as np
from itertools import combinations
import sys

# load configuration
print(os.getcwd() )
with open(os.path.join(get_package_share_directory('bench_pkg'), 'param/config.yaml'), 'r') as stream:
    try:
        config = DotMap(yaml.safe_load(stream), _dynamic=False)
    except yaml.YAMLError as exc:
        print(exc)

class MetricCollector:
    def __init__(self, benchManager) -> None:
        self.benchManager = benchManager

        datestring = datetime.datetime.now().strftime('%Y-%m-%d--%H-%M-%S')
        numAgents = self.benchManager.getNumAgents()
        baseFileString = f'{datestring}_{config.common.mapName}_{numAgents}_{config.pathPlanning.algorithmToCall}_{config.common.random.randomSeed}'
        
        # get suboptimal algos and add extra string to them
        suboptimalAlgos = []
        for algoName, algoDict in config.algorithms.items():
            if 'suboptimalityFactor' in algoDict:
                suboptimalAlgos.append(algoName)
        if config.pathPlanning.algorithmToCall in suboptimalAlgos:
            baseFileString = f'{datestring}_{config.common.mapName}_{numAgents}_{config.pathPlanning.algorithmToCall}_{config.common.random.randomSeed}_{config.algorithms[config.pathPlanning.algorithmToCall].suboptimalityFactor}'
       
        fileName = f'{baseFileString}.yaml'
        self.filePath = os.path.join(config.common.metrics.basePathToOutFiles, fileName)

        self.metrics = {
            # meta
            'baseFileString': baseFileString, #done
            'benchStartTime': datetime.datetime.now(), #done
            'finalAnalysisTime': None, #done
            'taskAllocationDescription': None,
            # basics
            'startsAndGoals': [], #done
            # TA computation
            'timeToCalculateDistanceMatrix': 0.0, #done
            'timeToCalculateAssignment': 0.0, #done 
            'totalTimeForTA': 0.0, #done
            # path planning computation
            'pathPlanningTimeoutReached': False,
            'validScheduleFound': False, #done
            'timeToCalculateSchedule': 0, #done
            'makespanPreSmoothing': 0, #done
            'makespanPostSmoothing': 0, #done
            'costPreSmoothing': 0, #done
            'costPostSmoothing': 0, #done
            # execution
            'executionTimeoutReached': False,
            'agentsAtGoal': [], #done
            'goalsReached': 0, #done
            'numGoals': 0,
            'totalTimeForScheduleExecution': 0, #done
            'minDistanceBetweenTwoAgents': 0, #done
            'minDistanceBetweenTwoAgentsAtStart': 0, # to be able to use a delta metric
            'minDistanceBetweenTwoAgentsDelta': 0, # starts - min => positive value means agents have gotten closer than they were at start.
            'timeBlockedPerAgent': [], # when an agent has started moving, is not at the goal and is not making progress, he is considered blocked
            'timeBlockedTotal': 0
        }

        self.updateFileOnDisk()

    def updateMetric(self, keyName, newValue, increment=False):
        if increment:
            self.metrics[keyName] += newValue
        else:    
            self.metrics[keyName] = newValue

    def addStartAndGoal(self, sgDict):
        self.metrics['startsAndGoals'].append(sgDict)
        # sgDict must look like:
        # sgDict = {
        #     'assignedAgent': 'agent0',
        #     'start_name': 'cleaning_cabinet',
        #     'goal_name': 'tiger',
        #     'start_grid': [5, 57],
        #     'goal_grid': [87, 15],
        #     'start_wcos': [8.512312, -17.35213],
        #     'goal_wcos': [12.6512, -4.1823],
        # }

    def getAgentGoal(self, agentName):
        for sg in self.metrics['startsAndGoals']:
            if sg['assignedAgent'] == agentName:
                return (sg['goal_wcos'][0], sg['goal_wcos'][1])
        return None

    def getAgentStart(self, agentName):
        for sg in self.metrics['startsAndGoals']:
            if sg['assignedAgent'] == agentName:
                return (sg['start_wcos'][0], sg['start_wcos'][1])
        return None

    # def getStartsAndGoalsWCOSAsListOfTuples(self):
    #     starts = []
    #     goals = []
    #     for sg in self.metrics['startsAndGoals']:


    # TODO: Put into some static Util class for both
    def updateFileOnDisk(self):
        # save on disk
        o = {'config': config.toDict()}
        o['metrics'] = self.metrics
        f = open(self.filePath, 'w')
        f.write(yaml.dump(o, sort_keys=False, default_style=None))
        f.close()


class BagAnalyzer:
    # some code copied from https://answers.ros.org/question/358686/how-to-read-a-bag-file-in-ros2/
    def __init__(self, baseFileString) -> None:
        self.baseFileString = baseFileString
        self.metricsFileName = f'{baseFileString}.yaml'
        self.filePath = os.path.join(os.path.dirname(baseFileString), self.metricsFileName)
        print(f'Metrics file is {self.filePath}')

        # load metrics yaml for editing
        with open(self.filePath, 'r') as stream:
            try:
                self.fullFile = DotMap(yaml.safe_load(stream), _dynamic=False)
                self.metrics = self.fullFile.metrics
            except yaml.YAMLError as exc:
                print(exc)
                sys.exit()

        # get the respective config
        self.config = self.fullFile.config

        bagFileFolder = self.baseFileString

        db3files = []
        for file in os.listdir(bagFileFolder):
            fullPath = os.path.join(bagFileFolder, file)
            if fullPath.endswith('.db3') and os.path.getsize(fullPath) > 0:
                db3files.append(fullPath)


        if len(db3files) > 1:
            print(f'ERROR! More than one non-empty bag file found. This is not supported.')
            sys.exit()

        
        if len(db3files) == 0:
            print(f'No bags for this yaml')
        else:
            print(f'Analyzing {db3files[0]}') # there must be exactly one, otherwise we would have exited already.

            self.conn = sqlite3.connect(db3files[0])

            self.cursor = self.conn.cursor()

            # create a message type map
            topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
            self.topic_type = {name_of:type_of for id_of,name_of,type_of in topics_data}
            self.topic_id = {name_of:id_of for id_of,name_of,type_of in topics_data}
            self.topic_msg_message = {name_of:get_message(type_of) for id_of,name_of,type_of in topics_data}


    def __del__(self):
        try:
            self.conn.close()
        except:
            pass
            # seems like there was no conn yet

    def updateMetric(self, keyName, newValue, increment=False): # duplicate from method in Collector... Refactor?
        if increment:
            self.metrics[keyName] += newValue
        else:    
            self.metrics[keyName] = newValue

    def getMetric(self, keyName):
        return self.metrics[keyName]

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):

        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        # Deserialise all and timestamp them
        return [ (timestamp,deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp,data in rows]

    def updateFileOnDisk(self):
        # save on disk
        o = self.fullFile.toDict()
        o['metrics'] = self.metrics.toDict()
        f = open(self.filePath, 'w')
        f.write(yaml.dump(o, sort_keys=False, default_style=None))
        f.close()

    # called by analyze_bags script
    def calculateMetrics(self): 
        print('test')
        self.updateMetric('finalAnalysisTime', datetime.datetime.now())

        # check if a schedule was found at all
        if self.getMetric('timeToCalculateSchedule') == 0:
            self.updateMetric('pathPlanningTimeoutReached', True) # this was not stored properly in older experiments
            self.updateMetric('timeToCalculateSchedule', self.config.pathPlanning.timeout * 1.5) # with penalty
        else:
            self.traceRobots()
        
        self.updateFileOnDisk()

    def traceRobots(self):
        # scan for progress
        fleet_states = self.get_messages('/fleet_states')

        # time between two messages
        self.period = ((fleet_states[-1][0] - fleet_states[0][0]) / 10**9) / len(fleet_states)

        sag = {} # starts and goals dict
        for sg in self.metrics.startsAndGoals:
            sag[sg.assignedAgent] = {}
            sag[sg.assignedAgent]['start'] = np.array(sg.start_wcos)
            sag[sg.assignedAgent]['goal'] = np.array(sg.goal_wcos)

        self.updateMetric('numGoals', len(sag))

        # determine when the first robots moved (recording starts earlier)
        # determine when the last robot reached its goal

        startOfMovement = None
        endOfMovement = None
        agentsAtGoal = set()
        distanceInitValue = 99999.9
        minDistanceBetweenTwoAgents = distanceInitValue
        minDistanceBetweenTwoAgentsAtStart = distanceInitValue

        locationTolerance = self.config.common.locationTolerance # 0.5 fixed

        distancesToGoal = {} # each agents gets a list of max length 10
        windowSize = 10
        stepsBlocked = {}

        maxRobotsSeen = 0

        isFullFirstFleetState = True
        for fs in fleet_states:
            locs = []
            maxRobotsSeen =  max(maxRobotsSeen, len(fs[1].robots))
            for robot in fs[1].robots:
                loc = np.array((robot.location.x, robot.location.y))
                locs.append(loc)
                # check distance to start
                dist_start = np.linalg.norm(loc - sag[robot.name]['start'])
                # note the first start of movement
                if not startOfMovement:
                    if dist_start > locationTolerance:
                        startOfMovement = fs[0]

                # check distance to goal
                if robot.name not in agentsAtGoal:
                    dist_goal = np.linalg.norm(loc - sag[robot.name]['goal'])
                    if dist_goal < locationTolerance:
                        agentsAtGoal.add(robot.name)
                        # print(f'{robot.name} seen at goal.')
                    if len(agentsAtGoal) == len(sag):
                        endOfMovement = fs[0]

                    # check if agent is blocked
                    # this is assumed if they do not progress by cellSize within windowSize steps
                    # one time step is assumed to be 0.5s (as frequency of the fleetState is 2Hz)
                    if dist_start > self.config.common.cellSize or dist_goal < self.config.common.cellSize:

                    # they may get close enough to their goal before getting far enough from their start
                        if not robot.name in distancesToGoal:
                            distancesToGoal[robot.name] = []
                        if not robot.name in stepsBlocked:
                            stepsBlocked[robot.name] = 0

                        distancesToGoal[robot.name].append(dist_goal)
                        distancesToGoal[robot.name] = distancesToGoal[robot.name][-windowSize:]
                        if len(distancesToGoal[robot.name]) >= windowSize:
                            # alternative formulation
                            # if (abs(sum(distancesToGoal[robot.name][:-1]) / (windowSize-1) - distancesToGoal[robot.name][(windowSize-1)])) <= self.config.common.cellSize:
                            if (abs(distancesToGoal[robot.name][0] - distancesToGoal[robot.name][(windowSize-1)])) <= self.config.common.cellSize:
                                stepsBlocked[robot.name] += 1


            # calculate min distance between all robots
            for loc_pair in combinations(locs, 2):
                minDistanceBetweenTwoAgents = min(np.linalg.norm(loc_pair[0] - loc_pair[1]), minDistanceBetweenTwoAgents)
            
            # save the initial min distance once
            if minDistanceBetweenTwoAgents != distanceInitValue and isFullFirstFleetState and maxRobotsSeen == self.getMetric('numGoals'):
                minDistanceBetweenTwoAgentsAtStart = float(minDistanceBetweenTwoAgents)
                self.updateMetric('minDistanceBetweenTwoAgentsAtStart', minDistanceBetweenTwoAgentsAtStart)
                isFullFirstFleetState = False

                # print(f'{robot.name} distance to start: {dist_start}')
        minDistanceBetweenTwoAgentsDelta =  float(minDistanceBetweenTwoAgentsAtStart) - float(minDistanceBetweenTwoAgents)

        # time blocked calcs
        for key in stepsBlocked:
            stepsBlocked[key] *= self.period

        timeBlockedTotal = sum(stepsBlocked.values())

        if maxRobotsSeen != len(stepsBlocked.values()):
            print('\n\n\n\n\n\n\n\n\n\n\n\n')
            print('WARNING NOT EVERYONE MOVED!')
            print('maxRobotsSeen:', maxRobotsSeen)
            print('len(stepsBlocked.values()):', len(stepsBlocked.values()))
            print('stepsBlocked:', stepsBlocked)
            print('\n\n\n\n\n\n\n\n\n\n\n\n')

            self.updateMetric('invalidRun', True)
        else:
            self.updateMetric('invalidRun', False)

        print('timeBlockedPerAgent', stepsBlocked)
        print('timeBlockedTotal', timeBlockedTotal)
        print('agentsAtGoal', agentsAtGoal)
        print('maxRobotsSeen', maxRobotsSeen)
        print('minDistanceBetweenTwoAgents', minDistanceBetweenTwoAgents)
        print('minDistanceBetweenTwoAgentsAtStart', minDistanceBetweenTwoAgentsAtStart)
        print('minDistanceBetweenTwoAgentsDelta', minDistanceBetweenTwoAgentsDelta)

        # update in metric yaml
        self.updateMetric('timeBlockedPerAgent', stepsBlocked)
        self.updateMetric('timeBlockedTotal', timeBlockedTotal)
        self.updateMetric('maxRobotsSeen', maxRobotsSeen)
        self.updateMetric('minDistanceBetweenTwoAgents', float(minDistanceBetweenTwoAgents))
        self.updateMetric('minDistanceBetweenTwoAgentsDelta', minDistanceBetweenTwoAgentsDelta)
        self.updateMetric('agentsAtGoal', list(agentsAtGoal))
        self.updateMetric('goalsReached', len(agentsAtGoal))
        if endOfMovement is not None:
            timeTaken = (endOfMovement - startOfMovement) / 1e9
            print('timeTaken', timeTaken)
            self.updateMetric('allGoalReached', True)
            self.updateMetric('totalTimeForScheduleExecution', timeTaken)
        else:
            self.updateMetric('allGoalReached', False)
            self.updateMetric('executionTimeoutReached', True)
            self.updateMetric('totalTimeForScheduleExecution', self.config.common.recordingTimeout)