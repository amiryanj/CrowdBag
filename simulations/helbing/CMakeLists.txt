cmake_minimum_required(VERSION 2.6)
		
#Project
project(Helbing)

#Includes
include_directories(include)

#Headers
set(HELBING_HEADERS
	include/Helbing.h
	include/intelkdtree.h
	include/obstacle.h
	include/vec2f.h
)

#Source Files
file(
	GLOB
        HELBING_SOURCES
	src/Helbing.cpp
	src/vec2f.cpp
)

#Output
add_library(
        helbing SHARED
        ${HELBING_SOURCES}
)

#Install
install(FILES ${HELBING_HEADERS} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/helbing)
install(TARGETS helbing DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/helbing EXPORT helbing-targets)
install(EXPORT helbing-targets DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/helbing)
install(FILES helbing-config.cmake DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/helbing)

