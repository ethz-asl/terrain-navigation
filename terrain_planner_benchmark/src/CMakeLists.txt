target_sources(${PROJECT_NAME}
  PRIVATE
    terrain_planner_benchmark.cpp
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    terrain_navigation::terrain_navigation
    terrain_planner::terrain_planner
    ${OpenCV_LIBRARIES} ${GeographicLib_LIBRARIES} ${OMPL_LIBRARIES}
)

target_sources(${PROJECT_NAME}_node
  PRIVATE
    terrain_planner_benchmark_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
    ${PROJECT_NAME}::${PROJECT_NAME} rclcpp::rclcpp ${geometry_msgs_TARGETS}
)

target_sources(ompl_benchmark_node
  PRIVATE
    ompl_benchmark_node.cpp
)

target_sources(test_ics
  PRIVATE
    test_ics.cpp
)

target_sources(test_rrt_replanning_node
  PRIVATE
    test_rrt_replanning_node.cpp
)

target_sources(surface_visualization
  PRIVATE
    surface_visualization.cpp
)

target_sources(test_dubins_classification
  PRIVATE
    test_dubins_classification.cpp
)

target_sources(run_replay
  PRIVATE
    rviz_replay.cpp
)

target_sources(test_rrt_circle_goal
  PRIVATE
    test_rrt_circle_goal.cpp
)
