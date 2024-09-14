cmake_minimum_required(VERSION 3.0.2)
project(gem_mock_sensors LANGUAGES CXX
 VERSION 1.0.0
 DESCRIPTION "The gem_mock_sensors project"
 HOMEPAGE_URL "https://github.com/georgiosmatzarapis/POLARIS_GEM_e2_MANAGER.git")

find_package(catkin REQUIRED COMPONENTS
 roscpp
 std_msgs
)

catkin_package()

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(gem_mock_sensors_node src/gem_mock_sensors_node.cpp)

target_link_libraries(gem_mock_sensors_node
 ${catkin_LIBRARIES}
)

install(TARGETS gem_mock_sensors_node
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
