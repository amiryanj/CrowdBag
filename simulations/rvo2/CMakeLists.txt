cmake_minimum_required(VERSION 2.6)
		
#Project
project(RVO2)

#Includes
include_directories(include)

#Headers
set(RVO2_HEADERS
        include/RVOAgent.h
	include/Definitions.h
	include/KdTree.h
	include/Obstacle.h
	include/RVO.h
	include/RVOSimulator.h
	include/Vector2.h
)

#Source Files
file(
	GLOB
        RVO2_SOURCES
        src/RVOAgent.cpp
	src/KdTree.cpp
	src/Obstacle.cpp
	src/RVOSimulator.cpp
)

#Output
add_library(
        rvo2 SHARED
        ${RVO2_SOURCES}
)

#Install
install(FILES ${RVO2_HEADERS} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/rvo2)
install(TARGETS rvo2 DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/rvo2 EXPORT rvo2-targets)
install(EXPORT rvo2-targets DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/rvo2)
install(FILES rvo2-config.cmake DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/rvo2)

