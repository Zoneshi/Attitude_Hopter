cmake_minimum_required(VERSION 3.20)

project(HOPTER_ATTITUDE)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED)

aux_source_directory(src DIR_SRCS)
add_executable(main ${DIR_SRCS})

target_link_libraries (main Eigen3::Eigen)
