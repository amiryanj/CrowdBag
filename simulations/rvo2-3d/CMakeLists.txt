cmake_minimum_required(VERSION 2.6)
		
#Project
project(RVO2-3D)

#Includes
include_directories(include)

#Headers
set(RVO2-3D_HEADERS
	include/Agent.h
	include/API.h
	include/Definitions.h
	include/KdTree.h
	include/RVO.h
	include/RVOSimulator.h
	include/Vector3.h
)

#Source Files
file(
	GLOB
	source_files
	src/Agent.cpp
	src/KdTree.cpp
	src/RVOSimulator.cpp
)

#Output
add_library(
	rvo2-3d STATIC
	${source_files}
)

#Install
install(FILES ${RVO2-3D_HEADERS} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/rvo2-3d)
install(TARGETS rvo2-3d DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/rvo2-3d EXPORT rvo2-3d-targets)
install(EXPORT rvo2-3d-targets DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/rvo2-3d)
install(FILES rvo2-3d-config.cmake DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/rvo2-3d)

