# Installation Instructions Docker
The Dockerfile needs the `mrp_bench` repo to be cloned and available from the current folder F. This means `ls` should show you a folder named `mrp_bench`.

The configuration yaml for this container is defined in the repository at `bench_ws/bench_pkg/bench_pkg/param/docker_config.yaml`. During the docker build process, this is moved so that it becomes the `config.yaml` file.

Create a folder to store the generated data on the host, e.g. `~/experiments`. From the folder F, run

``` 
docker build -f mrp_bench/docker/Dockerfile -t mrbench .
docker run -it -v ~/experiments:/experiments mrbench
```

The benchmark is installed at `/mrp_bench`. From here on, experiments can be run the same way as without docker.

Detach the docker container using Ctrl + P, Ctrl + Q



# Installation Instructions Manually From Source
For manually installing from see, see [these instructions](Installation.md).


# Running Experiments
## Launching a single experiment
To launch simulation suite, from the mrp_bench folder, run
```
source /opt/ros/galactic/setup.bash
source ./install/setup.bash
python3 launch/top_level_launch.py
```

Here, the settings from `bench_ws/bench_pkg/bench_pkg/param/config.yaml` are used. To become effective, the bench_pkg needs to have been recompiled (`colcon build --packages-select bench_pkg`). 

## Launching multiple experiments
TODO: This is being updated to a version where you can provide the config.yaml and a special syntax for iterating over parameters is used.

The script located at `experiments/run_benchmarks.py` can be used and modified to run multiple experiments. 
It contains several lists or ranges that can be used to try out different parameter sets.

The following ones (as set in the script) serve as an example, but can and should be modified for your needs.
 * randomSeed: set a range of random seeds that are used to generate sets of starts and goals. Each seeds represents a different scenario
 * algorithms: list of algorithms to try out (each with each scenario). For a list of possible algorithm, see config.yaml
 * numAgents: we used 5 and 9 often, but depending on your available computation power, other numbers are possible.
 * extra: current used to try out different suboptimality factors, but other values or further iteration layers can be used.

# Analyzing experiments
To analyze you conducted experiments, three steps are needed. For each of these files, call it without a parameter (or with --help) to learn about the syntax for usage.
1. Run `experiments/analyze_bags_folder.py` to calculate the post-execution metrics for all experiments.
2. Run `experiments/yaml_to_csv.py` to combine the experiments into a single csv file.
3. Run `experiments/statistics_playground.py` to find interesting results and observations.

Only the first step is actually mandatory. Steps 2 and 3 can be replaced by your own data analysis pipeline.


# Other important-to-know things
## Grid-creator plugin
The purpose of this plugin is to raytrace a base costmap in gazebo.
Because this is rather calculation-intensive, it is only done if no file named "map.pgm" exists in the main folder.
This map.pgm is not directly used. Instead, for each specific map, the pgm is defined in launch/mapname_map.pgm.
For a new or changed map, remove the map.pgm from the main folder to force a recalculation. Then, copy this new map.pgm to the launch/maps folder, rename to mapname.pgm.


## License

MRP-bench is open-sourced under the Apache-2.0 license. See the
[LICENSE](LICENSE) file for details.

For a list of other open source components included in MRP-bench, see the
file [open_source_licenses.md](open_source_licenses.md).


