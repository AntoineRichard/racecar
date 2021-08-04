cmake_minimum_required(VERSION 2.8.3)
project(racecar_description)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS)
find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZRBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
${catkin_INCLUDE_DIRS}
${GAZEBO_INCLUDE_DIRS}
)

add_library(force_plugin SHARED plugins/add_force.cpp)
add_dependencies(force_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(force_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS
  force_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
