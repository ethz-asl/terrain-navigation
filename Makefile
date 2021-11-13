benchmark?=output/benchmark.csv
package?=terrain_nav

format:
	Tools/fix_code_style.sh .

build:
	catkin build ${package}

build-test:
	catkin build terrain_planner --no-deps -v -i --catkin-make-args tests

test: build-test
	Tools/run_tests.sh .

