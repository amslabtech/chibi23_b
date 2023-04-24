cmake_minimum_required(VERSION 3.0.2)
project(chibi23_b)

find_package(catkin REQUIRED COMPONENTS
      roscpp
      std_msgs
      nav_msgs
      sensor_msgs
      geometry_msgs
      tf
      tf2
      roomba_500driver_meiji
      )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES chibi23_b
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(local_goal_creator src/local_goal_creator.cpp)
add_executable(local_map_creator  src/local_map_creator.cpp)
add_executable(obstacle_detector  src/obstacle_detector.cpp)

add_dependencies(local_goal_creator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(local_map_creator   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(obstacle_detector   ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(local_goal_creator ${catkin_LIBRARIES})
target_link_libraries(local_map_creator   ${catkin_LIBRARIES})
target_link_libraries(obstacle_detector   ${catkin_LIBRARIES})

