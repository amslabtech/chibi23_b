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
add_executable(localizer src/localizer.cpp)

add_dependencies(localizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(localizer ${catkin_LIBRARIES})
