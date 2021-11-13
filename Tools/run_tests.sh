#!/bin/bash
source $HOME/catkin_ws/devel/setup.bash
for f in $HOME/catkin_ws/devel/lib/*/*-test
do
    $f
done
