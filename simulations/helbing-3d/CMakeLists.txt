cmake_minimum_required(VERSION 2.6)
		
#Project
project(Helbing-3D)

#Includes
include_directories(include)

#Headers
set(HELBING-3D_HEADERS
	include/Helbing.h
	include/intelkdtree.h
	include/obstacle.h
	include/vec2f.h
	include/vec3f.h
)

#Source Files
file(
	GLOB
	source_files
	src/Helbing.cpp
	src/vec2f.cpp
	src/vec3f.cpp
)

#Output
add_library(
	helbing-3d STATIC
	${source_files}
)

#Install
install(FILES ${HELBING-3D_HEADERS} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/helbing-3d)
install(TARGETS helbing-3d DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/helbing-3d EXPORT helbing-3d-targets)
install(EXPORT helbing-3d-targets DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/helbing-3d)
install(FILES helbing-3d-config.cmake DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/helbing-3d)

