benchmark?=output/benchmark.csv
package?=terrain_planner
data?=output

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

analyze:
	roslaunch adaptive_viewutility run_compare_mesh.launch
	python3 Tools/visualize_mapdata.py output/map_data.csv
	rm -f output/*.csv

clean:
	rm -f output/*.jpeg
	rm -f output/*.bag
	rm -f output/*.png
	rm -f output/*.csv
