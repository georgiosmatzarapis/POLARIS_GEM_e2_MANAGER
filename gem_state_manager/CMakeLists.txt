cmake_minimum_required(VERSION 3.0.2)
project(gem_state_manager LANGUAGES CXX
 VERSION 1.0.0
 DESCRIPTION "The gem_state_manager project"
 HOMEPAGE_URL "https://github.com/georgiosmatzarapis/POLARIS_GEM_e2_MANAGER.git")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
 roscpp
 std_msgs
)

catkin_package()

include_directories(
 ${catkin_INCLUDE_DIRS}
)

set(Sources
 src/gem_state_manager_node.cpp
 src/callback_handlers.cpp
 src/state_manager.cpp
)

add_executable(${PROJECT_NAME}_node ${Sources})

target_include_directories(${PROJECT_NAME}_node PRIVATE
 include
)

target_link_libraries(${PROJECT_NAME}_node
 ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
