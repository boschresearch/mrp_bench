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
import datetime

def main(args=None):
    print('Start Bench')

    full_path = os.path.realpath(__file__)
    this_script_dir = os.path.dirname(full_path)
    config_path_at_runtime = os.path.join(this_script_dir, '../install/bench_pkg/share/bench_pkg/param/config.yaml')

    with open(config_path_at_runtime, 'r') as stream:
        try:
            config = DotMap(yaml.safe_load(stream))
        except yaml.YAMLError as exc:
            print(exc)
            return

    # manual loops for starters
    randomSeeds = range(3000, 3300)
    algorithms = ['AStar', 'CBS', 'EECBS'] # , ['CBS-TA', 'CBS', 'AStar','ECBS', 'ECBS-TA', 'EECBS']
    numAgents = [5]
    maps = ['office', 'warehouse']
    extra = [1.2] # [1.05, 1.2]
    for r in randomSeeds:
        for n in numAgents:
            for m in maps:
                for a in algorithms:
                    nonSuboptimalHandled = False
                    for x in extra:
                        # no multiple runs for optimal algos
                        if nonSuboptimalHandled:
                            continue
                        # wait for previous run to shut everything down if (just in case)
                        print('Sleeping 10 seconds before starting run.')
                        time.sleep(10.0)
                        # remove possibly remainign run counts flag
                        try:
                            os.remove(config.common.pathToFlagFileRunDoesNotCount)
                        except OSError:
                            pass

                        # set current loop content
                        config.common.random.randomSeed = r
                        config.common.random.numAgents = n
                        config.common.mapName = m
                        config.pathPlanning.algorithmToCall = a
                        # config.common.mapName = x
                        if a in ['ECBS', 'EECBS', 'ECBS-TA']:
                            config.algorithms.ECBS.suboptimalityFactor = x
                            config.algorithms.EECBS.suboptimalityFactor = x
                            config.algorithms['ECBS-TA'].suboptimalityFactor = x
                        else:
                            nonSuboptimalHandled = True

                        # write config to disk
                        f = open(config_path_at_runtime, 'w')
                        f.write(yaml.dump(config.toDict(), sort_keys=False, default_style=None))
                        f.close()

                        # start run
                        runCounts = False
                        interrupted = False
                        while not runCounts and not interrupted:
                            try:
                                p1 = subprocess.Popen(['python3', os.path.join(this_script_dir, '../launch', 'top_level_launch.py')]) 

                                p1.communicate()
                                                
                                # check run counts
                                if os.path.exists(config.common.pathToFlagFileRunDoesNotCount):
                                    print('Run does not count flag sighted. Removing it and repeating loop')
                                    # remove run counts flag from failure
                                    try:
                                        os.remove(config.common.pathToFlagFileRunDoesNotCount)
                                    except OSError:
                                        pass
                                else:
                                    print('Run appears to count.')
                                    runCounts = True

                            except KeyboardInterrupt:
                                txt = open('/tmp/runBenchmarks.txt', 'a')
                                txt.write(datetime.datetime.fromtimestamp(time.time()).isoformat() + '\n')
                                txt.close()
                                print('Registered interrupt.')
                                os.kill(p1.pid, signal.SIGINT)
                                interrupted = True



if __name__ == '__main__':
    main()
