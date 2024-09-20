cmake_minimum_required(VERSION 3.0.2)
project(gem_mock_sensors LANGUAGES CXX
 VERSION 1.0.0
 DESCRIPTION "The gem_mock_sensors project"
 HOMEPAGE_URL "https://github.com/georgiosmatzarapis/POLARIS_GEM_e2_MANAGER.git")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
 roscpp
 std_msgs
 ros1_lib
)

catkin_package()

include_directories(
 ${catkin_INCLUDE_DIRS}
)

add_executable(battery_node src/battery_node.cpp)
target_link_libraries(battery_node ${catkin_LIBRARIES})

add_executable(temperature_node src/temperature_node.cpp)
target_link_libraries(temperature_node ${catkin_LIBRARIES})

add_executable(gps_node src/gps_node.cpp)
target_link_libraries(gps_node ${catkin_LIBRARIES})

add_executable(signal_node src/signal_node.cpp)
target_link_libraries(signal_node ${catkin_LIBRARIES})

add_executable(emergency_stop_node src/emergency_stop_node.cpp)
target_link_libraries(emergency_stop_node ${catkin_LIBRARIES})

install(TARGETS battery_node temperature_node gps_node signal_node emergency_stop_node
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
