cmake_minimum_required(VERSION 3.10)

project(mpc_driving VERSION 0.1.0 LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

find_package(Eigen3 3.4 CONFIG REQUIRED QUIET)
find_package(PkgConfig REQUIRED QUIET)
pkg_check_modules(IPOPT REQUIRED ipopt)

add_library(${PROJECT_NAME} SHARED)
target_include_directories(
	${PROJECT_NAME} PRIVATE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/include>
		${IPOPT_INCLUDE_DIRS}
)
target_sources(
	${PROJECT_NAME} PRIVATE
		src/MPC.cpp
		src/Optimizer.cpp
		src/Utils.cpp
		src/model/State.cpp	
		src/model/SystemModel.cpp
		src/math/CubicSpline.cpp	
		src/math/CubicSpline2D.cpp
)

target_link_libraries(
	${PROJECT_NAME} PUBLIC
	Eigen3::Eigen
	${IPOPT_LIBRARIES}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
	TARGETS mpc_driving
	EXPORT mpc_driving-targets
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
	EXPORT mpc_driving-targets
	FILE mpc_driving-targets.cmake
	NAMESPACE mpc_driving::
	DESTINATION ${INSTALL_CONFIGDIR}
)

write_basic_package_version_file(
	${CMAKE_CURRENT_BINARY_DIR}/mpc_driving-config-version.cmake
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
	${CMAKE_CURRENT_SOURCE_DIR}/cmake/mpc_driving_config.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/mpc_driving-config.cmake
	INSTALL_DESTINATION $${INSTALL_CONFIGDIR}
)

install(
	FILES
		${CMAKE_CURRENT_BINARY_DIR}/mpc_driving-config.cmake
		${CMAKE_CURRENT_BINARY_DIR}/mpc_driving-config-version.cmake
	DESTINATION ${INSTALL_CONFIGDIR}
)
