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


add_executable(localizer src/localizer.cpp)
