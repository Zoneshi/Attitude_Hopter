cmake_minimum_required(VERSION 3.20)

project(HOPTER_ATTITUDE)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

add_executable(main main.cpp hopter.cpp hopter.hpp)

target_link_libraries (main Eigen3::Eigen)
