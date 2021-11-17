benchmark?=output/benchmark.csv
package?=terrain_navigation

format:
	Tools/fix_code_style.sh .

config:
	catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

build:
	catkin build ${package}

build-test:
	catkin build terrain_planner --no-deps -i --catkin-make-args tests

test: build-test
	Tools/run_tests.sh .

