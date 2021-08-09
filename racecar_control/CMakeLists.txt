cmake_minimum_required(VERSION 2.8.3)
project(racecar_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    std_msgs
    ackermann_msgs
    roscpp
)

catkin_package(
    #INCLUDE_DIRS include
    CATKIN_DEPENDS
    std_msgs
    ackermann_msgs
    roscpp
)

include_directories(
    #include
    ${catkin_INCLUDE_DIRS}
)

add_executable(gz_racecar_interface src/gazebo_interface.cpp)
target_link_libraries(gz_racecar_interface
    ${catkin_LIBRARIES}
)
