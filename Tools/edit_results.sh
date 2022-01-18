#!/bin/bash

# This script generates a subset of the original dataset and runs the photogrammetry

export DATASET_PATH=$1

echo $DATASET_PATH

mv ${DATASET_PATH}/*.obj ${DATASET_PATH}/mesh.obj
mv ${DATASET_PATH}/*.xyz ${DATASET_PATH}/offset.xyz
mv ${DATASET_PATH}/*.bag ${DATASET_PATH}/gridmap_0.bag
