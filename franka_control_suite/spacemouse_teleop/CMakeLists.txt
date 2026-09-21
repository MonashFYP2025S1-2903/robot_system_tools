cmake_minimum_required(VERSION 3.5)
project(spacemouse_cartesian_teleop CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

add_executable(spacemouse_cartesian_teleop spacemouse_cartesian_teleop.cpp)
target_link_libraries(spacemouse_cartesian_teleop Franka::Franka Eigen3::Eigen Threads::Threads)
