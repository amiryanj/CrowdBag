cmake_minimum_required(VERSION 2.8.8)

# Project
project(Core)

# Includes
include_directories(${CMAKE_CURRENT_LIST_DIR}/include)

# Headers
set(CORE_HEADERS
        ./include/Calibration.h
	${CMAKE_CURRENT_LIST_DIR}/include/Configurable.h
	${CMAKE_CURRENT_LIST_DIR}/include/ConfigurationParser.h
	${CMAKE_CURRENT_LIST_DIR}/include/Dummy.h
	${CMAKE_CURRENT_LIST_DIR}/include/Experiment.h
	${CMAKE_CURRENT_LIST_DIR}/include/Experiment3D.h
	${CMAKE_CURRENT_LIST_DIR}/include/CraalIncludes.h
	${CMAKE_CURRENT_LIST_DIR}/include/Metric.h
	${CMAKE_CURRENT_LIST_DIR}/include/Parser.h
	${CMAKE_CURRENT_LIST_DIR}/include/Parser3D.h
	${CMAKE_CURRENT_LIST_DIR}/include/PDF.h
	${CMAKE_CURRENT_LIST_DIR}/include/Pedestrian.h
	${CMAKE_CURRENT_LIST_DIR}/include/Record.h
	${CMAKE_CURRENT_LIST_DIR}/include/Record3D.h
	${CMAKE_CURRENT_LIST_DIR}/include/ResourceLister.h
	${CMAKE_CURRENT_LIST_DIR}/include/Simulation.h
	${CMAKE_CURRENT_LIST_DIR}/include/Simulation3D.h
	${CMAKE_CURRENT_LIST_DIR}/include/SimulationGeneric.h
	PARENT_SCOPE
)

# Source Files
file(
	GLOB
        CORE_SOURCES
        ${CMAKE_CURRENT_LIST_DIR}/src/Calibration.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/Configurable.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/ConfigurationParser.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/Dummy.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/Experiment.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/Experiment3D.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/Metric.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/PDF.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/Pedestrian.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/Record.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/Record3D.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/ResourceLister.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/Simulation.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/Simulation3D.cpp
)

# Output
add_library(
        core SHARED
        ${CORE_SOURCES}
)

