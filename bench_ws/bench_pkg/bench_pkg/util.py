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


import string
import random

class Util:
    @staticmethod    
    def generateRandomString(length):
        letters = string.ascii_lowercase
        return ''.join(random.choice(letters) for i in range(length))
