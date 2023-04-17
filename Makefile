benchmark?=output/benchmark.csv
package?=terrain_planner
data?=output
path?=resources/benchmark.yaml

format:
	Tools/fix_code_style.sh .

config:
	catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

build: config
	catkin build ${package} mav_planning_rviz

build-test:
	catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCATKIN_ENABLE_TESTING=True
	catkin build terrain_navigation terrain_planner --no-deps -i --catkin-make-args tests

test: build-test
	Tools/run_tests.sh .

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

benchmark-dubins-classifications:
	terrain_planner_benchmark/Tools/run_dubins_classification_benchmark.sh

clean-dataset:
	rm -rf output/dataset

clean: clean-dataset
	rm -rf output/*
