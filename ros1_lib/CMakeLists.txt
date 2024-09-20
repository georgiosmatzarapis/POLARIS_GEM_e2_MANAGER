cmake_minimum_required(VERSION 3.0.2)
project(ros1_lib LANGUAGES CXX
 VERSION 1.0.0
 DESCRIPTION "The ros1_lib project"
 HOMEPAGE_URL "https://github.com/georgiosmatzarapis/POLARIS_GEM_e2_MANAGER.git")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS roscpp core)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ros1_lib
 CATKIN_DEPENDS roscpp core
)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
 ${catkin_INCLUDE_DIRS}
 ${PROJECT_SOURCE_DIR}/include
)
