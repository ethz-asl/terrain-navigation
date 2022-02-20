benchmark?=output/benchmark.csv
package?=terrain_planner
data?=output
path?=resources/benchmark.yaml

format:
	Tools/fix_code_style.sh .

config:
	catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

build:
	catkin build ${package} mav_planning_rviz

build-test:
	catkin build terrain_planner --no-deps -i --catkin-make-args tests

test: build-test
	Tools/run_tests.sh .

analyze: clean
	roslaunch adaptive_viewutility run_compare_mesh.launch
	python3 Tools/visualize_mapdata.py output/map_data.csv
	rm -f output/*.csv

compare:
	python3 Tools/visualize_comparisons.py ${path}

precision:
	python3 Tools/visualize_precision.py ${path}

pathlength:
	python3 Tools/visualize_pathlength.py ${path}

curate-dataset:
	Tools/curate_dataset.sh output output

process-dataset:
	Tools/process_dataset.sh ${path}

clean-dataset:
	rm -rf output/dataset

clean: clean-dataset
	rm -f output/*.jpeg
	rm -f output/*.bag
	rm -f output/*.png
	rm -f output/*.csv
