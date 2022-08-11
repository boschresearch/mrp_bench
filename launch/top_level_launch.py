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

import os
from ament_index_python.packages import get_package_share_directory
import yaml
import subprocess
import signal
from dotmap import DotMap
import time
import sys
import shutil
import datetime
import random

def main(args=None):
    print('Start')

    config_path = os.path.join(get_package_share_directory('bench_pkg'), 'param/config.yaml')

    with open(config_path, 'r') as stream:
        try:
            config = DotMap(yaml.safe_load(stream))
        except yaml.YAMLErrr as exc:
            print(exc)
            return


    # staggered launch
    startIndex = 0
    stepSize = 3
    sleepTimeAfterSpawnInSec = 15.0

    # launch_dir = os.path.dirname(os.path.realpath(__file__))
    launch_dir = os.path.basename(os.path.dirname(__file__))
     # path needs to be relative, otherwise it is treated as a package name when adding arguments?!

    # for handling reruns if not all agents spawned
    runCounts = False

    try:
        # remove all flags...
        try:
            os.remove(config.common.pathToFlagFileRobotsMissing)
        except OSError:
            pass
        # remove readiness flag from previous launched
        try:
            os.remove(config.common.pathToFlagFileReadyToSpawn)
        except OSError:
            pass
        # remove readiness flag from previous launched
        try:
            os.remove(config.common.pathToFlagFileReadyToSpawn)
        except OSError:
            pass
        # remove done + failure flag from previous launched
        try:
            os.remove(config.common.pathToFlagFileAllGoalsReached)
        except OSError:
            pass
        try:
            os.remove(config.common.pathToFlagFileFailure)
        except OSError:
            pass

        p1 = subprocess.Popen(['ros2', 'launch', os.path.join(launch_dir, 'benchmark_sim_launch.py')]) 

        processes = []
        while not os.path.exists(config.common.pathToFlagFileReadyToSpawn):
            # check if the algorithms failed to calculate a solutions
            if os.path.exists(config.common.pathToFlagFileFailure):
                print('Failure flag sighted. Stopping simulation.')
                os.kill(p1.pid, signal.SIGINT)
                time.sleep(3.0)
                os._exit(0)

            print('Waiting for flag to exist at ' + config.common.pathToFlagFileReadyToSpawn)
            time.sleep(1.0)

        # start block for calculation without execution only
        # comment this in to just calculate schedules but not exec them
        # os.kill(p1.pid, signal.SIGINT)
        # return
        # end block for calculation without execution only

        p2 = subprocess.Popen(['ros2', 'launch', os.path.join(launch_dir, 'single_nodes.py')])


        with open(config.common.pathToRobotsList, 'r') as stream:
            try:
                robots = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
                return

        # # temp debug check:
        # if random.choice([False, True]):
        #     numRobots = len(robots)
        #     print('*****************************************\n')
        #     print('Using correct num robots\n')
        #     txt = open('/tmp/chance.txt', 'a')
        #     txt.write('Correct num\n')
        #     txt.close()
        # else:
        #     numRobots = len(robots) - 1
        #     print('*****************************************\n')
        #     print('Using too few robots\n')
        #     txt = open('/tmp/chance.txt', 'a')
        #     txt.write('Too few\n')
        #     txt.close()
        numRobots = len(robots)
        print('numRobots: ', numRobots)

        while startIndex < numRobots:
            endIndex = min(startIndex + stepSize, numRobots) # first not used index, like in slices
            processes.append(subprocess.Popen(['ros2', 'launch', os.path.join(launch_dir, 'robot_instances.py'), 'startIndex:=' + str(startIndex), 'endIndex:=' + str(endIndex)]))
            startIndex += stepSize
            time.sleep(sleepTimeAfterSpawnInSec)

        # timeout starts now
        startTimeout = time.time()
        
        done = False
        runCounts = False

        while True:
            # here, we only get interrupted by flags set by the BM or if our own countdown expires

            # temporarily only: to forgo execution and just test spawning. Can be removed later because it's checked above
            if os.path.exists(config.common.pathToFlagFileFailure):
                msg = 'Failure flag sighted. Stopping simulation.'
                done = True
                runCounts = True
            # end tmp only

            # check if we never saw all robots that should have been spawned
            print('Checking for spawn timeout flag from BM.')
            if os.path.exists(config.common.pathToFlagFileRobotsMissing):
                msg = 'Missed robots that seem to never have spawned. Will try again.'
                done = True
                runCounts = False
                txt = open('/tmp/spawn_failure_counter.txt', 'a')
                txt.write(datetime.datetime.fromtimestamp(time.time()).isoformat() + '\n')
                txt.close()
                print('Wrote to failure counter.')
                # remove the last metric and bag
                metricsAndMore = []
                for file in os.listdir(config.common.metrics.basePathToOutFiles):
                    fullPath = os.path.join(config.common.metrics.basePathToOutFiles, file)
                    if fullPath.endswith('yaml'):
                        metricsAndMore.append(fullPath)

                metricsAndMore.sort(reverse=True)
                yamlFile = metricsAndMore[0]
                folder = yamlFile[:-5]
                print(f'Deleting {yamlFile} and {folder}.')
                try:
                    os.remove(yamlFile)
                except OSError:
                    print('Could not delete ', yamlFile)
                try:
                    shutil.rmtree(folder)
                except OSError:
                    print('Could not delete ', folder)
                
                # now we'll start again


            print('Checking for finish flag from the MetricCollector.')
            if os.path.exists(config.common.pathToFlagFileAllGoalsReached):
                msg = 'All goals reached. Stopping simulation.'
                done = True
                runCounts = True

            timeSinceStart = time.time() - startTimeout
            if timeSinceStart > config.common.recordingTimeout:
                msg = 'Timeout reached. Stopping simulation.'
                done = True
                runCounts = True

            if done:
                print(msg)
                if not runCounts:
                    # signal to run benchmarks that this was a failure
                    txt = open(config.common.pathToFlagFileRunDoesNotCount, 'w')
                    txt.write(str(time.time()))
                    txt.close()
                    print('Signalled that run does not count.')
                time.sleep(5.0)
                os.kill(p1.pid, signal.SIGINT)
                os.kill(p2.pid, signal.SIGINT)
                for process in processes:
                    os.kill(process.pid, signal.SIGINT)
                    
                time.sleep(5.0)
                os._exit(0)
            else:
                print(f'Not done and timeout still {config.common.recordingTimeout-timeSinceStart} seconds away. Sleeping.')
                time.sleep(5.0)

            # p1.communicate()
            # p2.communicate()
            # for process in processes:
            #     process.communicate()
    except KeyboardInterrupt:
        print('Registered interrupt.')
        os.kill(p1.pid, signal.SIGINT)
        os.kill(p2.pid, signal.SIGINT)
        for process in processes:
            os.kill(process.pid, signal.SIGINT)
        print('All killed.')
        sys.exit()

if __name__ == '__main__':
    main()
