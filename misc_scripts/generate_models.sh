#!/bin/bash
# change your path accordingly
export GAZEBO_PLUGIN_PATH=/home/username/bosch/mrp_bench/install/rmf_building_sim_gazebo_plugins/lib/rmf_building_sim_gazebo_plugins

# the list of model for which you need to generate thunbnails 
LIST_OF_MODELS=./models.list
OUTDIR=./thumbs

while IFS= read -r line; do
  printf 'Working on: %s\n' "$line"
  gzserver --verbose -s libthumbnail_generator.so empty.world --input $line --output $OUTDIR
done < $LIST_OF_MODELS