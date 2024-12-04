# Set the minimum version of CMake required
cmake_minimum_required(VERSION 3.10)

# Project name and language
project(mpc_example LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Set C++ standard to C++11 or higher
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Specify Python version (adjust if needed)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Python3 COMPONENTS NumPy)
find_package(Eigen3 3.4 CONFIG REQUIRED QUIET)
find_package(PkgConfig REQUIRED)
pkg_check_modules(IPOPT REQUIRED ipopt)

set(SOURCES
mpc_example.cpp	
../../MPCDriving/src/math/CubicSpline.cpp	
../../MPCDriving/src/math/CubicSpline2D.cpp
../../MPCDriving/src/model/State.cpp	
../../MPCDriving/src/model/SystemModel.cpp
../../MPCDriving/src/MPC.cpp
../../MPCDriving/src/Optimizer.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})
# Add executable target
target_include_directories(${PROJECT_NAME} PRIVATE 
${CMAKE_CURRENT_SOURCE_DIR}/../../MPCDriving/include
${IPOPT_INCLUDE_DIRS})

# Link libraries (Python3 and Boost system/filesystem)
target_link_libraries(${PROJECT_NAME}
	Eigen3::Eigen
	Python3::Python
	Python3::Module
	Python3::NumPy
	${IPOPT_LIBRARIES}
)

# If using matplotlibcpp, ensure that Python is available for linking
if (Python3_FOUND)
    message(STATUS "Python 3 found: ${Python3_VERSION}")
else ()
    message(FATAL_ERROR "Python 3 not found")
endif ()

