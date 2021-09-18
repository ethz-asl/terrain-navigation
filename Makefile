benchmark?=output/benchmark.csv
package?=terrain_nav

format:
	Tools/fix_code_style.sh .

build:
	catkin build ${package}

