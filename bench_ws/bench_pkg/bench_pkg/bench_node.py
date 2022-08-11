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

from matplotlib.pyplot import grid
import rclpy
import numpy as np
import math
# import matplotlib.pyplot as plt
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from rmf_fleet_msgs.msg import FleetState, PathRequest, Location
from visualization_msgs.msg import MarkerArray
from ament_index_python.packages import get_package_share_directory

from .metric_collector import MetricCollector
from .task_allocation import TaskAllocation
from .util import Util
from .structures.cellGrid import CellGrid
from .integrations.libMultiRobotPlanning import LibMultiRobotPlanning
from .integrations.mapf import MAPF

import time
import yaml
from dotmap import DotMap
import networkx as nx
import os
import timeit
import re
import copy
import sys

# load configuration
print(os.getcwd())
with open(os.path.join(get_package_share_directory('bench_pkg'), 'param/config.yaml'), 'r') as stream:
    try:
        config = DotMap(yaml.safe_load(stream), _dynamic=False)
    except yaml.YAMLError as exc:
        print(exc)


class BenchManager(Node):
    def __init__(self):
        super(BenchManager, self).__init__('bench_manager')

        # create the subs
        qos_profile_latched = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        qos_profile_volatile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        qos_profile_pub_keep_all = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_ALL,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.graph = None
        self.grid = None
        self.cellGrid = None
        self.readyToExecWaypoints = False
        self.waypointsTaskId = 1000
        self.metrics = MetricCollector(self)
        self.taskAllocator = TaskAllocation(self)
        self.fleetStateTimer = None
        self.allMovementStarted = False
        self.lastWaypointSentInitially = None

        # Graph sub converting viz published by RMF
        graph_sub = self.create_subscription(
            MarkerArray, topic='/map_markers', qos_profile=qos_profile_latched, callback=self.graph_callback)
        # Grid sub - grid generated and published by custom gazebo plugin
        grid_sub = self.create_subscription(
            OccupancyGrid, topic='/grid', qos_profile=qos_profile_latched, callback=self.grid_callback)
        print('Subscribers created')

        fleet_sub = self.create_subscription(
            FleetState, topic='/fleet_states', qos_profile=qos_profile_volatile, callback=self.fleet_state_callback)

        # publisher to signal readiness
        self.pathRequestPub = self.create_publisher(
            PathRequest, config.common.fleet.pathRequestTopicName, qos_profile=qos_profile_pub_keep_all)
        # self.ready_to_send_waypoints_pub = self.create_publisher(Bool, 'mrp_bench/ready', qos_profile=qos_profile_latched)

        # evaluate the allocation fields in config to easier to understand result
        tasks = f'Manually defined {self.getNumAgents()} starts and goals.'
        if config.common.useRandomTasks:
            tasks = f'Randomly generated {self.getNumAgents()} starts and goals.'
        alloc = 'Allocation fixed as defined.'
        if not config.taskAllocation.fixedAllocation:
            if config.taskAllocation.randomAllocation and not config.algorithms[config.pathPlanning.algorithmToCall].builtinTA:
                alloc = 'Allocation random.'
            else:
                if config.algorithms[config.pathPlanning.algorithmToCall].builtinTA:
                    alloc = f'Built-in allocation of {config.pathPlanning.algorithmToCall} used.'
                else:
                    alloc = f'Allocation done using {config.taskAllocation.allocationAlgorithm} with cost function {config.taskAllocation.costFunction}.'
        taText = f'{tasks} {alloc}'
        print(taText)
        self.metrics.updateMetric('taskAllocationDescription', taText)

        # check subs status
        print('Wait for subscriptions to be saved')

    def fleet_state_callback(self, msg):
        # initialize timeout
        if not self.fleetStateTimer:
            self.fleetStateTimer = time.time()
            print('Set self.fleetStateTimer.')
            self.agentsVisible = 0
        # check if all robots are present
        if not self.readyToExecWaypoints:
            if msg.name == config.common.fleet.name:
                if len(msg.robots) == self.getNumAgents():
                    print(
                        f'Seeing all {len(msg.robots)} of {self.getNumAgents()} agents.')
                    self.readyToExecWaypoints = True
                else:
                    if len(msg.robots) > self.agentsVisible:
                        self.agentsVisible = len(msg.robots)
                        self.fleetStateTimer = time.time()
                        print(
                            f'Seeing more robots! Now there are {self.agentsVisible}.')
                    print(
                        f'Seeing {self.agentsVisible} of {self.getNumAgents()} agents.')
                    # check if timer has expired
                    timeSinceLastSpawn = time.time() - self.fleetStateTimer
                    print(
                        f'Seconds since last spawn: {str(timeSinceLastSpawn)}')
                    if timeSinceLastSpawn > config.common.nextRobotTimeout:
                        # set flag
                        txt = open(
                            config.common.pathToFlagFileRobotsMissing, 'w')
                        txt.write(str(time.time()))
                        txt.close()
                        print('Robots missing signalled (due to spawn timeout).')
        else:
            # check if robot started moving
            if not self.allMovementStarted:
                robotsMoving = 0
                robotsNotMoving = []
                for robot in msg.robots:
                    loc = np.array((robot.location.x, robot.location.y))
                    # check distance to start and goal (maybe they don't need to move)
                    dist_start = np.linalg.norm(
                        loc - self.metrics.getAgentStart(robot.name))
                    dist_goal = np.linalg.norm(
                        loc - self.metrics.getAgentGoal(robot.name))
                    if dist_start > config.common.locationTolerance or dist_goal < config.common.locationTolerance:
                        robotsMoving += 1
                    else:
                        print(f'{robot.name} not moving yet.')
                        robotIndex = int((robot.name).replace(
                            config.common.agentPrefix, ''))
                        robotsNotMoving.append(
                            (robotIndex, robot.location.x, robot.location.y))

                if robotsMoving == self.getNumAgents():
                    self.allMovementStarted = True
                    print('All agents seen moving!')
                else:
                    print(f'{robotsMoving} of {self.getNumAgents()} are moving.')
                    # should never be getting here if it is None, but just in case
                    if self.lastWaypointSentInitially is not None:
                        timePassedSinceStart = time.time() - self.lastWaypointSentInitially
                        print(
                            f'{str(timePassedSinceStart)} seconds have passed since the waypoints have been sent and not everyone is moving.')
                        if timePassedSinceStart > config.common.firstMoveTimeout:
                            print(
                                f'Not all robots started moving on time (within {config.common.firstMoveTimeout} seconds).')
                            # set flag
                            txt = open(
                                config.common.pathToFlagFileRobotsMissing, 'w')
                            txt.write(str(time.time()))
                            txt.close()
                            print('Robots missing signalled (due to move timeout).')

            # check if goal is reached by each robot
            robotsAtGoal = 0
            for robot in msg.robots:
                loc = np.array((robot.location.x, robot.location.y))
                # check distance to goal
                dist_goal = np.linalg.norm(
                    loc - self.metrics.getAgentGoal(robot.name))
                if dist_goal < config.common.locationTolerance:
                    robotsAtGoal += 1
                else:
                    print(f'{robot.name} dist to goal: ', dist_goal)
            # all done?
            print('Robots at goal: ', robotsAtGoal)
            if robotsAtGoal == self.getNumAgents():
                # store current timestamp in done flag file
                txt = open(config.common.pathToFlagFileAllGoalsReached, 'w')
                txt.write(str(time.time()))
                txt.close()
                print('All goals reached signalled.')

    def signalFailure(self):
        txt = open(config.common.pathToFlagFileFailure, 'w')
        txt.write(str(time.time()))
        txt.close()

    def outputReady(self):
        return self.readyToExecWaypoints

    def inputReady(self):
        if self.graph is not None:  # and self.grid is not None: Grid not needed right now
            return True
        return False

    def getNumAgents(self):
        if config.common.useRandomTasks:
            return config.common.random.numAgents
        return len(config.maps[config.common.mapName].tasks)

    def getLongestPath(self):
        list_len = [len(i.path) for i in self.pathRequests]
        return max(list_len)

    def graph_callback(self, msg):
        self.get_logger().info('Graph msg received')
        self.graph = nx.Graph()

        # store everything first temporarily
        nodes = None
        edges = []
        labels = []
        nodeDict = {}  # nodes have no ID anymore, but we need one

        for marker in msg.markers:
            if marker.type == 8:  # points aka vertices aka nodes
                nodes = marker.points
            if marker.type == 5:  # unconnected lines that are transformed to edges
                for edge in marker.points:
                    # need to be collected first, because we need the points first (order in yaml not guaranteed)
                    edges.append(edge)
            if marker.type == 9:  # label with name
                # see ScheduleMarkerPublisher.hpp line 283 for offset
                node_x = marker.pose.position.x - 0.4 * math.cos(0.7853)
                node_y = marker.pose.position.y - 0.4 * math.sin(0.7853)
                label = {'x': node_x, 'y': node_y, 'text': marker.text}
                labels.append(label)

        for id, node in enumerate(nodes):
            nodeDict[(node.x, node.y)] = id
            # get label:
            text = ''
            for label in labels:
                if math.isclose(label['x'], node.x, rel_tol=1e-09) and math.isclose(label['y'], node.y, rel_tol=1e-09):
                    text = label['text']
                    break
            self.graph.add_node(id, x=node.x, y=node.y, name=text)

        for i in range(0, len(edges), 2):
            e_from = nodeDict[(edges[i].x, edges[i].y)]
            e_to = nodeDict[(edges[i+1].x, edges[i+1].y)]
            self.graph.add_edge(e_from, e_to)

        self.get_logger().info('Graph saved')

        # draw graph for sanity check
        # nx.draw_networkx(self.graph, with_labels=True, font_weight='bold')
        # plt.show()

    def grid_callback(self, msg):
        self.get_logger().info('Grid msg received')
        self.grid = msg
        self.get_logger().info('Grid saved')

    def generatePathRequests(self):
        self.get_logger().info('Generating waypoints from solution')
        # convert schedule to waypoints member variable

        # schedules: [agent0 sched, agent1 sched, ..., agent_i sched, ..., agent_N sched]
        #  agent_i sched: [(cell x, cell y), (cell x, cell y), ..., (cell x, cell y)]

        if config.algorithms[config.pathPlanning.algorithmToCall].libMRP:
            schedules = LibMultiRobotPlanning.getSchedulesFromOutfile(
                config.common.pathToTmpSolution)
        elif config.algorithms[config.pathPlanning.algorithmToCall].useMAPFMap:
            schedules = MAPF.getSchedulesFromOutfile(
                config.common.pathToTmpSolution)
        else:
            print('No valid algorithm for generatePathRequests()')
            sys.exit(1)

        self.pathRequests = []

        # collect cost + makespan pre and post smooth
        costPre = 0
        costPost = 0
        makespanPre = 0
        makespanPost = 0

        # smoothing if wanted by removing stairs
        if config.postProcessing.smoothPath:
            for j, schedule in enumerate(schedules):
                steps = len(schedule)
                print('Schedule length pre-smooth: ', steps)
                costPre += steps
                makespanPre = max(makespanPre, steps)

                scheduleNew = []
                it = enumerate(schedule)
                for i, wp in it:
                    scheduleNew.append(wp)
                    if i+2 < len(schedule):
                        dist_x = abs(schedule[i+2][0] - schedule[i][0])
                        dist_y = abs(schedule[i+2][1] - schedule[i][1])
                        if dist_x == dist_y:  # in cells, so working with integers
                            next(it, None)

                schedules[j] = scheduleNew
                stepsPost = len(schedules[j])
                print('Schedule length post-smooth: ', stepsPost)
                costPost += stepsPost
                makespanPost = max(makespanPost, stepsPost)

        # collect cost + makespan post smooth
        self.metrics.updateMetric('makespanPreSmoothing', makespanPre)
        self.metrics.updateMetric('makespanPostSmoothing', makespanPost)
        self.metrics.updateMetric('costPreSmoothing', costPre)
        self.metrics.updateMetric('costPostSmoothing', costPost)
        self.metrics.updateFileOnDisk()

        # convert to RMF Locations
        for j, schedule in enumerate(schedules):
            waypts = []  # Location{x, y, yaw, level_name}
            for i, wp in enumerate(schedule):
                yaw = 0.0  # safe default in case we never need to calc yaw
                x, y = self.cellGrid.cellToWorldCos(wp[0], wp[1])
                # generate yaw such that we point towards the next pose
                if i+1 < len(schedule):
                    x_next, y_next = self.cellGrid.cellToWorldCos(
                        schedule[i+1][0], schedule[i+1][1])
                    yaw = math.atan2(y_next-y, x_next-x)
                # else keep old yaw
                # generate complete location
                new_wp = Location()
                new_wp.x = x
                new_wp.y = y
                new_wp.yaw = yaw
                new_wp.level_name = config.common.levelName
                waypts.append(new_wp)

            msg = PathRequest()
            msg.fleet_name = config.common.fleet.name
            msg.robot_name = config.common.agentPrefix + str(j)
            # self.waypointsTaskId += 1
            # msg.task_id = 'wps_' + str(self.waypointsTaskId)
            msg.task_id = Util.generateRandomString(20)
            msg.path = waypts

            self.pathRequests.append(msg)

        self.get_logger().info('Generated waypoints from solution')

    def signalReadiness(self):
        # signal readiness to launcher

        # true = Bool()
        # true.data = True
        # self.ready_to_send_waypoints_pub.publish(true)

        # store current timestamp in readiness flag file
        txt = open(config.common.pathToFlagFileReadyToSpawn, 'w')
        txt.write(str(time.time()))
        txt.close()
        print('Readiness signalled.')

    # def publishWakeup(self):
    #     print(f'************************* Publishing wakeup *************************')
    #     msg = PathRequest()
    #     msg.fleet_name = config.common.fleet.name
    #     msg.robot_name = 'agent007'
    #     msg.task_id = Util.generateRandomString(20)
    #     msg.path = []
    #     self.pathRequestPub.publish(msg)

    def executeWaypoints(self, indexWP, indexAgent):
        print(
            f'************************* Going to send waypoints for index {str(indexAgent)} *************************')
        # print(self.pathRequests[indexAgent])
        if config.pathRequests.sendPathWithBreaks:
            print('indexWP: ', indexWP)
            if indexWP < len(self.pathRequests[indexAgent].path):
                indexedPR = copy.deepcopy(self.pathRequests[indexAgent])
                indexedPR.path = [indexedPR.path[indexWP]]
                indexedPR.task_id += str(indexWP)
                self.pathRequestPub.publish(indexedPR)
        else:
            self.pathRequestPub.publish(self.pathRequests[indexAgent])

        self.lastWaypointSentInitially = time.time()
        print(
            f'************************* Sent Waypoints for index {str(indexAgent)} *************************')

    def generateGoalsAndCallLibrary(self):
        print('************************* Generatig Starts + Goals and Calling Lib *************************')
        lib = config.pathPlanning.algorithmToCall

        if lib in ['CBS', 'ECBS', 'ECBS-TA', 'CBS-TA', 'AStar', 'EECBS']:
            self.cellGrid = CellGrid(self.graph, config.common.cellSize, self)
            if config.common.useRandomTasks:
                self.cellGrid.generateRandomStartsAndGoals(
                    randomSeed=config.common.random.randomSeed, numAgents=config.common.random.numAgents)
            else:
                self.cellGrid.generateStartsAndGoalsFromConfig()
            self.cellGrid.storeStartsAndGoalsMetricsAndRobotsList()

            # TODO: Put this matching of algorithms to functions somewhere more visible
            if lib in ['CBS', 'ECBS', 'ECBS-TA', 'CBS-TA']:
                res = self.callLibMRP(lib)
            elif lib == 'AStar':  # AStar needs extra wrapper for separate calls
                res = self.callAStar()
            elif lib == 'EECBS':
                res = self.callMAPF(lib)

            print(f'{lib} called.')
            return res

        print("Invalid input")

    def callLibMRP(self, algoName):
        print('Calling ' + algoName)

        inFile = config.common.pathToTmpInput
        LibMultiRobotPlanning.convertCellGridToObstacleMap(
            self.cellGrid, self.cellGrid.startsAndGoals, inFile, builtinTA=config.algorithms[algoName].builtinTA)
        args = ['-i', inFile, '-o', config.common.pathToTmpSolution]

        # if the algorithm has a suboptimality factor, add it
        try:
            factor = str(config.algorithms[algoName].suboptimalityFactor)
            args.extend(
                ['-w', str(config.algorithms[algoName].suboptimalityFactor)])
            print(
                f'For algorithm {algoName}, added a suboptimality factor of {factor}.')
        except AttributeError:
            print(
                f'Algorithm {algoName} does not have a suboptimality factor.')

        def performCall(algoName):
            global res
            global timeoutReached
            res, timeoutReached = LibMultiRobotPlanning.callLibMultiRobotPlanning(
                config.algorithms[algoName].pathToBinary, args, timeout=config.pathPlanning.timeout)

        # autorange to get valid results for small calc time
        iterations, totalTime = timeit.Timer(
            lambda: performCall(algoName)).autorange()
        res = globals()['res']
        timeoutReached = globals()['timeoutReached']

        self.metrics.updateMetric('pathPlanningTimeoutReached', timeoutReached)
        self.metrics.updateMetric('validScheduleFound', bool(res))
        self.metrics.updateMetric(
            'timeToCalculateSchedule', totalTime/iterations)

        # print('s&g before: ', self.cellGrid.startsAndGoals)
        # extra TA if necessary
        if config.algorithms[config.pathPlanning.algorithmToCall].builtinTA:
            # matched line be like: 0->(16,0)
            # startsAndGoals be like: [([18, 17], [16, 0]), ([58, 31], [3, 31]), ([62, 4], [62, 4])]
            startsAndGoalsAssigned = []
            if not res:
                return res

            # there can be multiple assignments aparrently, but the last one matches the output
            assignments = res.decode('UTF-8').split('nextTaskAssignment')
            lastAssignment = assignments[-1]
            for line in lastAssignment.split('\n'):
                if '->(' in line:
                    nums = re.findall(r'\d+', line)
                    source = self.cellGrid.startsAndGoals[int(nums[0])][0]
                    target = [int(s) for s in nums[1:]]
                    startsAndGoalsAssigned.append((source, target))
            self.cellGrid.startsAndGoals = startsAndGoalsAssigned
            self.cellGrid.storeStartsAndGoalsMetricsAndRobotsList()

        # print('s&g after : ', self.cellGrid.startsAndGoals)
        print('Calling done')
        return res

    def callAStar(self):
        # A* has a completely different map format than the rest of libMRP. Just a plain txt file with different chars
        # LibMultiRobotPlanning.drawSolutionOnTxt(pathToGridTmpTxtFile, pathToOutputYAML, pathToGridWithSolution)

        # convert grid
        LibMultiRobotPlanning.convertAndStoreGridAsTxt(
            self.cellGrid, config.common.pathToTmpInput)

        totalTime = 0.0
        schedules = []

        for sg in self.cellGrid.startsAndGoals:
            # [([start.cell_x, start.cell_y], [sample_goals[i].cell_x, sample_goals[i].cell_y]), ...]

            args = ['--startX', str(sg[0][0]), '--startY', str(sg[0][1]), '--goalX', str(sg[1][0]), '--goalY', str(sg[1][1]),
                    '-m', config.common.pathToTmpInput, '-o', config.common.pathToTmpSolution]

            def performCall():
                global res
                global timeoutReached
                res, timeoutReached = LibMultiRobotPlanning.callLibMultiRobotPlanning(
                    config.algorithms['AStar'].pathToBinary, args)

            # autorange to get valid results for small calc time
            iterations, timeCall = timeit.Timer(
                lambda: performCall()).autorange()
            res = globals()['res']

            totalTime += timeCall/iterations

            if timeoutReached:
                self.metrics.updateMetric('pathPlanningTimeoutReached', True)
                return False

            if not res:  # no solution found
                self.metrics.updateMetric('validScheduleFound', False)
                return False

            # add to schedules list
            with open(config.common.pathToTmpSolution, 'r') as stream:
                try:
                    solution = yaml.safe_load(stream)
                except yaml.YAMLError as exc:
                    print(exc)

            # always agent1 here, no agent name is passed to AStar
            schedules.append(solution['schedule']['agent1'])

        # generate combined schedule
        combined = {'schedule': {}}
        for i, schedule in enumerate(schedules):
            combined['schedule'][config.common.agentPrefix + str(i)] = schedule

        class NoAliasDumper(yaml.SafeDumper):
            def ignore_aliases(self, data):
                return True
        f = open(config.common.pathToTmpSolution, 'w')
        f.write(yaml.dump(combined, Dumper=NoAliasDumper,
                sort_keys=False, default_style=None))
        f.close()

        self.metrics.updateMetric('validScheduleFound', True)
        self.metrics.updateMetric('timeToCalculateSchedule', totalTime)
        return True

    def callMAPF(self, algoName):
        # call an algorithm using the MAPF format.
        # right now this is just EECBS. Might need to be generalized or split if other algorithms using the MAPF format are added.
        print('Calling ' + algoName)

        mapFile = config.common.pathToTmpInput
        scenFile = config.common.pathToTmpScenarioFile
        numAgents = len(self.cellGrid.startsAndGoals)
        MAPF.convertCellGridToMAPFFormat(self.cellGrid, mapFile)
        MAPF.convertStartsAndGoalsToMAPFScenario(
            self.cellGrid, self.cellGrid.startsAndGoals, scenFile, mapFile)
        args = ['-m', mapFile, '-a', scenFile, '-o', config.common.pathToTmpStatistics, '--outputPaths', config.common.pathToTmpSolution,
                '-k', str(numAgents), '-t', '999999999']
        # using unlimited time here, as we have an external logic for limiting run time

        # if the algorithm has a suboptimality factor, add it
        try:
            factor = str(config.algorithms[algoName].suboptimalityFactor)
            args.extend(
                ['--suboptimality', str(config.algorithms[algoName].suboptimalityFactor)])
            print(
                f'For algorithm {algoName}, added a suboptimality factor of {factor}.')
        except AttributeError:
            print(
                f'Algorithm {algoName} does not have a suboptimality factor.')

        def performCall(algoName):
            global res
            global timeoutReached
            res, timeoutReached = MAPF.callLib(
                config.algorithms[algoName].pathToBinary, args, timeout=config.pathPlanning.timeout)

        # autorange to get valid results for small calc time
        iterations, totalTime = timeit.Timer(
            lambda: performCall(algoName)).autorange()
        res = globals()['res']
        timeoutReached = globals()['timeoutReached']

        self.metrics.updateMetric('pathPlanningTimeoutReached', timeoutReached)
        self.metrics.updateMetric('validScheduleFound', bool(res))
        self.metrics.updateMetric(
            'timeToCalculateSchedule', totalTime/iterations)

        # not handling TA here yet

        # print('s&g after : ', self.cellGrid.startsAndGoals)
        print('Calling done')
        return res

    def executionDone(self):
        return False

    def updateExecutionMetrics(self):
        pass


def main(args=None):
    print('Start')
    rclpy.init(args=args)

    bm = BenchManager()

    while rclpy.ok() and not bm.inputReady():
        try:
            rclpy.spin_once(bm)
        except KeyboardInterrupt:
            break

    print('Input ready.')

    print('edges: ', len(bm.graph.edges()))
    print('nodes: ', len(bm.graph.nodes()))

    if bm.generateGoalsAndCallLibrary():
        print('grid stats: ', bm.cellGrid.getGridStatics())
        bm.generatePathRequests()
        bm.signalReadiness()

        print('wait for nav2 up and robots spawned by scanning fleet for expected number of robots')
        while rclpy.ok() and not bm.outputReady():
            try:
                rclpy.spin_once(bm)
            except KeyboardInterrupt:
                break

        print('All robots seen. Now sleeping 5s...')
        # print('Actually we end now...')
        # bm.signalFailure()
        time.sleep(5.0)
        print('Woke up')

        # publish the paths
        sendIndexRange = 1
        if config.pathRequests.sendPathWithBreaks:
            sendIndexRange = bm.getLongestPath()

        for i in range(sendIndexRange):
            for a in range(bm.getNumAgents()):
                # i only relevant if sendPathWithBreaks is true
                bm.executeWaypoints(i, a)
                rclpy.spin_once(bm, timeout_sec=2.5)
            # resend agent0
            bm.executeWaypoints(i, 0)
            rclpy.spin_once(bm, timeout_sec=2.5)

            time.sleep(config.pathRequests.breakInSeconds)

    else:
        print('No successful call to lib. Nothing to do.')
        bm.metrics.updateFileOnDisk()
        bm.signalFailure()
        bm.destroy_node()
        rclpy.shutdown()
        return

    while rclpy.ok() and not bm.executionDone():
        try:
            bm.updateExecutionMetrics()
            rclpy.spin_once(bm)
        except KeyboardInterrupt:
            break

    bm.metrics.updateFileOnDisk()
    bm.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
