#!/bin/bash

# This script generates a subset of the original dataset and runs the photogrammetry

export DATASET_PATH=$1

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

for DIR in ${DATASET_PATH}/dataset/*; do
    $SCRIPT_DIR/edit_results.sh $DIR
     roslaunch adaptive_viewutility run_compare_mesh.launch path:=$DIR
done
