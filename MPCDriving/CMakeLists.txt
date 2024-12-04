# Set the minimum version of CMake required
cmake_minimum_required(VERSION 3.10)

# Project name and language
project(MPCDriving LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Set C++ standard to C++11 or higher
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Specify Python version (adjust if needed)
find_package(Eigen3 3.4 CONFIG REQUIRED QUIET)
find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED ipopt)

# Add your source files (replace with your actual .cpp files)
set(SOURCES
src/MPC.cpp
src/math/CubicSpline.cpp	
src/math/CubicSpline2D.cpp
src/model/State.cpp	
src/model/SystemModel.cpp
src/Optimizer.cpp
)

# Add executable target
add_library(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE 
${CMAKE_CURRENT_SOURCE_DIR}/include
${IPOPT_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} 
	Eigen3::Eigen
	${IPOPT_LIBRARIES}
)


