cmake_minimum_required(VERSION 3.0.2)
project(gem_manager LANGUAGES CXX
 VERSION 1.0.0
 DESCRIPTION "The gem_manager project"
 HOMEPAGE_URL "https://github.com/georgiosmatzarapis/POLARIS_GEM_e2_MANAGER.git")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
 roscpp
 std_msgs
 roslib
 ros1_lib
 message_generation
)

add_message_files(
 DIRECTORY gem_planner/msg
 FILES Waypoint.msg WaypointsBatch.msg
)

generate_messages(
 DEPENDENCIES
 std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp std_msgs roslib ros1_lib
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(state_manager_node
 gem_state_manager/src/state_manager_node.cpp
 gem_state_manager/src/callback_handlers.cpp
 gem_state_manager/src/state_manager.cpp
 gem_state_manager/src/states/idle.cpp
 gem_state_manager/src/states/running.cpp
 gem_state_manager/src/states/error.cpp
)
target_include_directories(state_manager_node PRIVATE
 gem_state_manager/include
)
target_link_libraries(state_manager_node
 ${catkin_LIBRARIES}
)

add_executable(planner_node
 gem_planner/src/planner_node.cpp
 gem_planner/src/planner.cpp
 gem_planner/src/controllers/pure_pursuit.cpp
 gem_planner/src/controllers/stanley.cpp
)
target_include_directories(planner_node PRIVATE
 gem_planner/include
)
target_link_libraries(planner_node
 ${catkin_LIBRARIES}
)
add_dependencies(planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS state_manager_node planner_node
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
