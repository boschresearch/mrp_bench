#!/usr/bin/python3
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

import sys
from pathlib import Path
import os
import random
import yaml

# Provide the path of the building.yaml file as the first argument. All vertices without a name will be assigned a random name.

random_seed = 42
random.seed(random_seed)

output = 'res.yaml'

animals_file = './animals/animals.txt'
script_path = Path(__file__).parent
animals_path = os.path.join(script_path, animals_file)

if len(sys.argv) != 2:
    print("Error: need to provide the file name to change and only the file name.")
    exit()

with open(animals_path) as file:
    # for office
    lines = [line.rstrip().lower().replace(' ', '_') for line in file if len(line) < 12] 
    # for aiport
    # lines = [line.rstrip().lower().replace(' ', '_') for line in file]

# lines_extra = [line + suffix for line in lines for suffix in ['_a', '_b']]
# lines += lines_extra


# open the yaml
with open(sys.argv[1], 'r') as stream:
    building = yaml.safe_load(stream)

print(len(building['levels']['L1']['vertices']))

sample = iter(random.sample(lines, len(building['levels']['L1']['vertices'])))

# replace empty names with animals
for k, vertex in enumerate(building['levels']['L1']['vertices']):
    if vertex[3] == "":
        building['levels']['L1']['vertices'][k][3] = next(sample)

# dump back
f = open(output, 'w')
f.write(yaml.dump(building, sort_keys=True, default_style=None))
f.close()

print('Done')
