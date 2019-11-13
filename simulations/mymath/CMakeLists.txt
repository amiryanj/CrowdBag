cmake_minimum_required(VERSION 2.6)
		
#Project
project(Mymath)

#Includes
include_directories(include)

#Headers
set(MYMATH_HEADERS
	include/angles.h
	include/line.h
	include/mat2.h
	include/mat3.h
	include/mymath.h
	include/seg2.h
	include/seg3.h
	include/vec2.h
	include/vec3.h
)

#Source Files
file(
	GLOB
	source_files
	src/angles.cpp
	src/mat2.cpp
	src/mat3.cpp
	src/mymath.cpp
	src/seg2.cpp
	src/seg3.cpp
	src/vec3.cpp
)

#Output
add_library(
	mymath STATIC
	${source_files}
)

#Install
install(FILES ${MYMATH_HEADERS} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/mymath)
install(TARGETS mymath DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/mymath EXPORT mymath-targets)
install(EXPORT mymath-targets DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/mymath)
install(FILES mymath-config.cmake DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/mymath)

