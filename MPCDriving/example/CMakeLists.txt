# Set the minimum version of CMake required
cmake_minimum_required(VERSION 3.10)

# Project name and language
project(SplinePlanner LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Set C++ standard to C++11 or higher
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Specify Python version (adjust if needed)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 COMPONENTS NumPy)
find_package(Eigen3 3.4 CONFIG REQUIRED QUIET)

# Add your source files (replace with your actual .cpp files)
set(SOURCES
    spline_example.cpp	
    ../src/CubicSpline.cpp	
    ../src/CubicSpline2D.cpp
)

# Add executable target
add_executable(spline_planner ${SOURCES})
target_include_directories(spline_planner PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../include)
# Link libraries (Python3 and Boost system/filesystem)
target_link_libraries(spline_planner 
	Eigen3::Eigen
	Python3::Python
	Python3::Module
	Python3::NumPy
)

# If using matplotlibcpp, ensure that Python is available for linking
if (Python3_FOUND)
    message(STATUS "Python 3 found: ${Python3_VERSION}")
else ()
    message(FATAL_ERROR "Python 3 not found")
endif ()

