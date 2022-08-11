
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

from posixpath import isabs
import sys
import os
import pandas as pd
import yaml
from dotmap import DotMap
import argparse

# pass folder as arg
if __name__ == "__main__":

    # check args
    help = '''
    Convert all yaml files (i.e. experiments) in a certain folder (subfolders are ignored).
    The multi-layer yaml structure is compressed into a single dimension.
    Some fields that are deemed unnecessary for analysis are left out.
    '''
    parser = argparse.ArgumentParser(description=help)
    parser.add_argument('--folder', '-f', type=str, required=True, help='Relative or absolute path to the folder in which the recorded experiments reside.')
    parser.add_argument('--outfile', '-o', type=str, default='all.csv', help='Path to the desired output file. Relative to the provided input folder, unless an absolute path is given.')

    if len(sys.argv)==1:
        parser.print_help(sys.stderr)
        sys.exit(1)
    args = parser.parse_args()

    path_to_folder = args.folder
    files = next(os.walk(path_to_folder), (None, None, []))[2]
    files = [os.path.join(path_to_folder, file) for file in files if file.endswith('yaml')]

    if len(files) == 0:
        print(f'No YAMLs found in {path_to_folder}.')
        exit(1)

    print(f'Converting {len(files)} files.')

    # detect where we look for suboptimality based on the first config. 
    suboptimalAlgos = None

    df_list = []
    for file in files:
        with open(file, 'r') as stream:
            try:
                d = yaml.safe_load(stream) # d for dict
            except yaml.YAMLError as exc:
                print(exc)

        if suboptimalAlgos == None:
            suboptimalAlgos = []
            for algoName, algoDict in d['config']['algorithms'].items():
                if 'suboptimalityFactor' in algoDict:
                    suboptimalAlgos.append(algoName)
            # this print is for manual verification
            print('Algorithms for which a suboptimality factor is used: ')
            print(suboptimalAlgos)


        # add subopt factor to some algos
        if d['config']['pathPlanning']['algorithmToCall'] in suboptimalAlgos:
            d['config']['pathPlanning']['algorithmToCall'] += '_' + str(d['config']['algorithms'][d['config']['pathPlanning']['algorithmToCall']]['suboptimalityFactor'])

        # tmp fix for eecbs
        if d['metrics']['timeToCalculateSchedule'] < 60.0:
            d['metrics']['validScheduleFound'] = True

        # remove unneeded columns
        # d['config'].pop('algorithms')
        try:
            d['config'].pop('maps') # may not be there if no manual positions are defined
        except:
            pass
        d['config']['common'].pop('pathToTmpInput')
        d['config']['common'].pop('pathToTmpSolution')
        d['config']['common'].pop('pathToFlagFileReadyToSpawn')
        d['config']['common'].pop('pathToFlagFileAllGoalsReached')
        d['config']['common'].pop('pathToFlagFileFailure')
        d['config']['common'].pop('pathToRobotsList')
        d['config']['common'].pop('agentZPose')
        d['config']['common'].pop('levelName')
        d['config']['common']['metrics'].pop('basePathToOutFiles')
        
        # flatten nested config
        df = pd.json_normalize(d, sep='.')

        df_list.append(df)

    # convert the the list of dataframes into a dataframe to be able to use to_csv on it. 
    dfs = pd.concat(df_list)

    # define output path and create necessary dirs
    if os.path.isabs(args.outfile):
        outpath = args.outfile
    else:
        outpath = os.path.join(path_to_folder, args.outfile)
        if not os.path.exists(os.path.dirname(outpath)):
            os.makedirs(os.path.dirname(outpath))

    # actually store the file
    dfs.to_csv(outpath, index = False, header=True)

    print(f'Conversion finished. Results saved to {outpath}.')
