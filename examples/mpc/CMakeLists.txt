# Set the minimum version of CMake required
cmake_minimum_required(VERSION 3.10)

# Project name and language
project(mpc_example LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Set C++ standard to C++11 or higher
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

message("${CMAKE_PREFIX_PATH}")

# Specify Python version (adjust if needed)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED QUIET)
find_package(Python3 COMPONENTS NumPy QUIET)
find_package(Eigen3 3.4 CONFIG REQUIRED QUIET)
find_package(mpc_driving REQUIRED)

add_executable(${PROJECT_NAME} mpc_example.cpp)


target_link_libraries(${PROJECT_NAME}
	Eigen3::Eigen
	Python3::Python
	Python3::Module
	Python3::NumPy
	mpc_driving
)

# If using matplotlibcpp, ensure that Python is available for linking
if (Python3_FOUND)
    message(STATUS "Python 3 found: ${Python3_VERSION}")
else ()
    message(FATAL_ERROR "Python 3 not found")
endif ()

