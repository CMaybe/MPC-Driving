# Set the minimum version of CMake required
cmake_minimum_required(VERSION 3.10)

# Project name and language
project(IpoptExample LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED ipopt)

# Add executable target
add_executable(${PROJECT_NAME} ipopt_example.cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${IPOPT_INCLUDE_DIRS})
message(${IPOPT_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${IPOPT_LIBRARIES})


