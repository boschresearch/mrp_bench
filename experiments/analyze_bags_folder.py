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

# For how to use, see variable "help" below.

import argparse
import os
import sys

# import local class from the bench_pkg
launch_dir = os.path.basename(os.path.dirname(__file__))
bench_pkg_path = os.path.join(launch_dir, '..', 'build/bench_pkg/build/lib/bench_pkg')
sys.path.append(bench_pkg_path)

try:
    from metric_collector import BagAnalyzer
except ImportError:
    print(f'Failed trying to import the BagAnalyzer class from {bench_pkg_path}.')
    print('Please make sure that the package is built (colcon build).')
    exit()



# pass folder as arg
if __name__ == "__main__":

    # check args
    help = '''
    Analyze all experiments in a certain folder (subfolders are ignored).
    An experiment is defined as a folder and a yaml with the same name, which must both exist.
    No other folder should be in there.
    The content of the yaml file is partly overwritten with the updated, calculated metrics.
    '''
    parser = argparse.ArgumentParser(description=help)
    parser.add_argument('--folder', '-f', type=str, required=True, help='Relative or absolute path to the folder in which the recorded experiments reside.')

    if len(sys.argv)==1:
        parser.print_help(sys.stderr)
        sys.exit(1)
    args = parser.parse_args()


    dirnames = next(os.walk(args.folder), (None, [], None))[1]

    if len(dirnames) == 0:
        print('Passed folder is empty.')
        exit(1)

    for dirname in dirnames:
        fullpath = os.path.join(args.folder, dirname)
        print(f'Analyzing {fullpath}')
        try:
            ba = BagAnalyzer(fullpath)
            ba.calculateMetrics()
            print('Analyzing done.\n---\n')
        except Exception as e:
            print('FAILURE ANALYZING BAG.')
            print(e)