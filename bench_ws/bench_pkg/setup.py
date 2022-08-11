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

from setuptools import setup, find_packages

package_name = 'bench_pkg'

setup(
    name=package_name,
    version='0.0.0',
    # packages=[package_name],
    packages=find_packages(exclude=['test']), # necessary to have imports from subdirs working
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/param', [package_name + '/param/config.yaml']),
        # ('share/' + package_name + '/param', [package_name + '/param/office_map.yaml']),
        # ('share/' + package_name + '/param', [package_name + '/param/airport_terminal_map.yaml']),
        # ('share/' + package_name + '/param', [package_name + '/param/warehouse_map.yaml']),
        # ('share/' + package_name + '/param', [package_name + '/param/turtlebot3_burger_custom.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luigi Palmieri',
    maintainer_email='Luigi.Palmieri@de.bosch.com',
    description='TODO: Package description',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bench_node = bench_pkg.bench_node:main',
        ],
    },
)
