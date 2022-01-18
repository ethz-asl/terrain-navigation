#!/bin/bash

# This script generates a subset of the original dataset and runs the photogrammetry

export DATASET_PATH=$1
export OUTPUT_PATH=$2

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

instance=0
num_images=$(ls ${DATASET_PATH}/*.jpeg | wc -l)
min_images=25
increment=25

echo "Total number of images: " $num_images

while [ $(( $((${increment}*${instance})) + ${min_images})) -lt $num_images ]; do
    INSTANCE_DATASET_PATH=${OUTPUT_PATH}/dataset/${instance}
    mkdir -p ${INSTANCE_DATASET_PATH}
    num_subset_images=0
    cp ${DATASET_PATH}/gridmap_${instance}.bag ${INSTANCE_DATASET_PATH}/
    for FILE in ${DATASET_PATH}/*.jpeg ; do
        echo "Copy file ${FILE} to ${INSTANCE_DATASET_PATH}";
        cp $FILE ${INSTANCE_DATASET_PATH}/
        num_subset_images=$(($num_subset_images + 1))
        echo "Num images" $num_images
        echo "Instance" $instance
        echo "increment" $increment
        echo "Number of subsets" $num_subset_images
        echo "Number of target " $(( $((${increment}*${instance})) + ${min_images}))

        if [ $num_subset_images -ge $(( $((${increment}*${instance})) + ${min_images})) ]
        then
            echo "number of subset images: $num_subset_images instance: $instance"
            break;
        fi
    done
    instance=$(($instance + 1))
done
