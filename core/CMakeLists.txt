cmake_minimum_required(VERSION 3.0.2)
project(core LANGUAGES CXX
 VERSION 1.0.0
 DESCRIPTION "The core project"
 HOMEPAGE_URL "https://github.com/georgiosmatzarapis/POLARIS_GEM_e2_MANAGER.git")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

catkin_package(
 INCLUDE_DIRS include
)
