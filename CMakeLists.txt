cmake_minimum_required(VERSION 3.10.2)
project(drake_cmake_installed)

find_package(drake CONFIG REQUIRED)

add_executable(drake_cfs_lib src/RobotModel.cc main.cc)
target_link_libraries(drake_cfs_lib drake::drake)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
