benchmark?=output/benchmark.csv
package?=terrain_planner

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

clean:
	rm output/*.jpeg